#include "PowerMeterApp.h"

#include <nvs_flash.h>
#include <esp_netif_sntp.h>
#include <esp_wifi.h>
#include <esp_sntp.h>
#include <mqtt_client.h>

#include "wifi_creds.h"
#include "mqtt_creds.h"

#define WIFI_CONNECTED_BIT BIT0
#define MQTT_STARTED_BIT   BIT1
#define MQTT_CONNECTED_BIT BIT2

void PowerMeterApp::telemetry_task()
{
    const TickType_t xTimeout = pdMS_TO_TICKS(100);
    ESP_LOGI(TAG, "telemetry_task: started");

    MqttMessage qmsg;
    while (!m_stop_tasks) {
        if (xQueueReceive(m_telemetry_queue, &qmsg, xTimeout)) {
            publish_mqtt_message(qmsg);
            ESP_LOGI(TAG, "MQTT: Published message");
        }
    }

    ESP_LOGI(TAG, "display_task: finished");
    vTaskDelete(nullptr);
}

void PowerMeterApp::setup_telemetry(bool disposing)
{
    if (disposing) {
        setup_wifi(disposing);
        setup_nvs(disposing);
        vQueueDelete(m_telemetry_queue), m_telemetry_queue = nullptr;
    }
    else {
        m_telemetry_queue = xQueueCreate(10, sizeof(MqttMessage));
        setup_nvs(disposing);
        setup_wifi(disposing);
    }
}

void PowerMeterApp::setup_nvs(bool disposing)
{
    if (!disposing) {
        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_ERROR_CHECK(nvs_flash_erase());
            ESP_ERROR_CHECK(nvs_flash_init());
        }
        else {
            ESP_ERROR_CHECK(ret);
        }
    }
}

void PowerMeterApp::setup_wifi(bool disposing)
{
    if (disposing) {
        esp_netif_sntp_deinit();

        if (m_mqtt_handle) {
            esp_mqtt_client_stop(m_mqtt_handle);
            esp_mqtt_client_unregister_event(m_mqtt_handle, MQTT_EVENT_ANY, member_cast<esp_event_handler_t>(&PowerMeterApp::on_mqtt_event));
            esp_mqtt_client_destroy(m_mqtt_handle), m_mqtt_handle = nullptr;
        }

        esp_wifi_stop();
        esp_wifi_deinit();

        esp_netif_destroy_default_wifi(m_netif), m_netif = nullptr;
        esp_netif_deinit();

        vEventGroupDelete(m_wifi_event_group), m_wifi_event_group = nullptr;
    }
    else {
        m_wifi_event_group = xEventGroupCreate();

        ESP_ERROR_CHECK(esp_netif_init());
        m_netif = esp_netif_create_default_wifi_sta();

        wifi_init_config_t wifi_init_cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_cfg));

        ESP_ERROR_CHECK(esp_event_handler_register(
            WIFI_EVENT, ESP_EVENT_ANY_ID,
            member_cast<esp_event_handler_t>(&PowerMeterApp::on_wifi_event), this
        ));
        ESP_ERROR_CHECK(esp_event_handler_register(
            IP_EVENT, IP_EVENT_STA_GOT_IP,
            member_cast<esp_event_handler_t>(&PowerMeterApp::on_wifi_event), this
        ));

        wifi_config_t wifi_config = {
            .sta = {
                .ssid = WIFI_SSID,
                .password = WIFI_PASS,
            },
        };

        ESP_LOGI(TAG, "Starting Wi-Fi");
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
        ESP_ERROR_CHECK(esp_wifi_start());

        esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
        esp_netif_sntp_init(&config);
    }
}

void PowerMeterApp::on_mqtt_event(esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t) event_data;

    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            //esp_mqtt_client_subscribe(m_mqtt_handle, MQTT_SUB_TOPIC, 0);
            xEventGroupSetBits(m_wifi_event_group, MQTT_CONNECTED_BIT);
            post_log({ "MQTT ready" });
            break;
        case MQTT_EVENT_DISCONNECTED:
            xEventGroupClearBits(m_wifi_event_group, MQTT_CONNECTED_BIT);
            post_log({ "MQTT disconnect" });
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "Unexpected MQTT event: '%*s' --> '%*s'", event->topic_len, event->topic, event->data_len, event->data);
            break;
        default:
            break;
    }
}

void PowerMeterApp::on_wifi_event(esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(m_wifi_event_group, WIFI_CONNECTED_BIT | MQTT_CONNECTED_BIT);
        esp_wifi_connect();
        post_log({ "Wi-Fi connecting" });
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(m_wifi_event_group, WIFI_CONNECTED_BIT);
        post_log({ "Wi-Fi ready" });
        esp_netif_sntp_sync_wait(pdMS_TO_TICKS(3000));

        auto uxBits = xEventGroupWaitBits(m_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, 0);
        if (!(uxBits & MQTT_STARTED_BIT)) {
            esp_mqtt_client_config_t mqtt_cfg {};
            mqtt_cfg.broker.address.uri = MQTT_BROKER_URI;

            m_mqtt_handle = esp_mqtt_client_init(&mqtt_cfg);
            ESP_ERROR_CHECK(esp_mqtt_client_register_event(
                m_mqtt_handle, MQTT_EVENT_ANY, member_cast<esp_event_handler_t>(&PowerMeterApp::on_mqtt_event), this));

            ESP_ERROR_CHECK(esp_mqtt_client_start(m_mqtt_handle));
            xEventGroupSetBits(m_wifi_event_group, MQTT_STARTED_BIT);
        }
        else {
            esp_mqtt_client_reconnect(m_mqtt_handle);
        }
    }
}

void PowerMeterApp::publish_mqtt_message(const MqttMessage& msg)
{
    auto uxBits = xEventGroupWaitBits(m_wifi_event_group, WIFI_CONNECTED_BIT | MQTT_CONNECTED_BIT, pdFALSE, pdFALSE, 0);
    if (uxBits & WIFI_CONNECTED_BIT) {
        if (uxBits & MQTT_CONNECTED_BIT) {
            char buf[sizeof(display_line_t)];
            int buf_len;

            buf_len = snprintf(buf, sizeof(buf), "%.0f", msg.f_v);
            esp_mqtt_client_publish(m_mqtt_handle, MQTT_PUB_TOPIC "/voltage", buf, buf_len, 0, 0);

            buf_len = snprintf(buf, sizeof(buf), "%.2f", msg.f_i);
            esp_mqtt_client_publish(m_mqtt_handle, MQTT_PUB_TOPIC "/current", buf, buf_len, 0, 0);

            buf_len = snprintf(buf, sizeof(buf), "%.1f", msg.f_va);
            esp_mqtt_client_publish(m_mqtt_handle, MQTT_PUB_TOPIC "/full_power", buf, buf_len, 0, 0);

            buf_len = snprintf(buf, sizeof(buf), "%.1f", msg.f_w);
            esp_mqtt_client_publish(m_mqtt_handle, MQTT_PUB_TOPIC "/real_power", buf, buf_len, 0, 0);

            buf_len = snprintf(buf, sizeof(buf), "%.2f", msg.f_wh);
            esp_mqtt_client_publish(m_mqtt_handle, MQTT_PUB_TOPIC "/energy", buf, buf_len, 0, 0);

            buf_len = snprintf(buf, sizeof(buf), "%.2f", msg.f_pf);
            esp_mqtt_client_publish(m_mqtt_handle, MQTT_PUB_TOPIC "/power_factor", buf, buf_len, 0, 0);

            buf_len = snprintf(buf, sizeof(buf), "%.2f", msg.f_hz);
            esp_mqtt_client_publish(m_mqtt_handle, MQTT_PUB_TOPIC "/frequency", buf, buf_len, 0, 0);
        }
        else {
            post_log({ "MQTT not ready" });
        }
    }
    else {
        post_log({ "Wi-Fi disconnect" });
    }
}
