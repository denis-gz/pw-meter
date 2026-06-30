#include "PowerMeterApp.h"

#include <nvs_flash.h>
#include <esp_netif_sntp.h>
#include <esp_sntp.h>
#include <esp_wifi.h>
#include <mqtt_client.h>
#include <optional>

#define POWER_TOPIC  "/metrics/power"
#define DEVICE_TOPIC "/metrics/device"

void PowerMeterApp::telemetry_task()
{
    ESP_LOGI(TAG, "telemetry_task: started");
    setup_telemetry();

    MqttMessage qmsg;
    while (!m_stop_tasks) {
        if (xQueueReceive(m_telemetry_queue, &qmsg, pdMS_TO_TICKS(100))) {
            publish_mqtt_message(qmsg);
            //ESP_LOGI(TAG, "MQTT: Published message");
        }
    }

    setup_telemetry(kDisposing);
    ESP_LOGI(TAG, "telemetry_task: finished");
    vTaskDelete(nullptr);
}

void PowerMeterApp::setup_telemetry(bool disposing)
{
    if (disposing) {
        sntp_set_time_sync_notification_cb(nullptr);
        setup_wifi(disposing);
        vQueueDelete(m_telemetry_queue), m_telemetry_queue = nullptr;
    }
    else {
        ESP_LOGI(TAG, "telemetry setup");
        m_telemetry_queue = xQueueCreate(10, sizeof(MqttMessage));
        setup_wifi(disposing);
    }
}

void PowerMeterApp::setup_wifi(bool disposing)
{
    if (disposing) {
        esp_netif_sntp_deinit();

        setup_mqtt(disposing);

        esp_event_handler_unregister(IP_EVENT, ESP_EVENT_ANY_ID, member_cast<esp_event_handler_t>(&PowerMeterApp::on_wifi_event));
        esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, member_cast<esp_event_handler_t>(&PowerMeterApp::on_wifi_event));

        if (m_netif) {
            esp_wifi_disconnect();
            esp_wifi_stop();
            esp_wifi_deinit();

            esp_netif_destroy_default_wifi(m_netif);
            m_netif = nullptr;

            esp_netif_deinit();
        }

        vEventGroupDelete(m_wifi_event_group), m_wifi_event_group = nullptr;
    }
    else {
        m_wifi_event_group = xEventGroupCreate();

        ESP_ERROR_CHECK(esp_netif_init());
        m_netif = esp_netif_create_default_wifi_sta();

        ESP_ERROR_CHECK(esp_event_handler_register(
            WIFI_EVENT, ESP_EVENT_ANY_ID,
            member_cast<esp_event_handler_t>(&PowerMeterApp::on_wifi_event), this
        ));
        ESP_ERROR_CHECK(esp_event_handler_register(
            IP_EVENT, IP_EVENT_STA_GOT_IP,
            member_cast<esp_event_handler_t>(&PowerMeterApp::on_wifi_event), this
        ));

        esp_event_post(WIFI_EVENT, WIFI_EVENT_USER_INIT, nullptr, 0, pdMS_TO_TICKS(10));
    }
}

void PowerMeterApp::setup_mqtt(bool disposing)
{
    if (disposing) {
        if (m_mqtt) {
            esp_mqtt_client_stop(m_mqtt);
            esp_mqtt_client_unregister_event(m_mqtt, MQTT_EVENT_ANY, member_cast<esp_event_handler_t>(&PowerMeterApp::on_mqtt_event));
            esp_mqtt_client_destroy(m_mqtt);
            xEventGroupClearBits(m_wifi_event_group, MQTT_STARTED_BIT);
            m_mqtt = nullptr;
        }
    }
    else {
        if (!m_settings.mqtt_uri[0]) {
            ESP_LOGW(TAG, "MQTT URI not set");
            return;
        }

        esp_mqtt_client_config_t mqtt_cfg {};
        mqtt_cfg.broker.address.uri = m_settings.mqtt_uri.data();

        decltype(m_settings.mqtt_creds) creds(m_settings.mqtt_creds);
        if (creds[0]) {
            mqtt_cfg.credentials.username = creds.data();
            char* sep = strchr(creds.data(), ':');
            if (sep) {
                *sep = '\0';
                mqtt_cfg.credentials.authentication.password = sep + 1;
            }
        }

        m_mqtt = esp_mqtt_client_init(&mqtt_cfg);
        if (m_mqtt) {
            esp_mqtt_client_register_event(m_mqtt, MQTT_EVENT_ANY, member_cast<esp_event_handler_t>(&PowerMeterApp::on_mqtt_event), this);
            esp_mqtt_client_start(m_mqtt);
            xEventGroupSetBits(m_wifi_event_group, MQTT_STARTED_BIT);
        }
        else {
            ESP_LOGW(TAG, "MQTT client init failed!");
        }
    }
}

void PowerMeterApp::on_wifi_event(esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_USER_INIT: {
                wifi_init_config_t wifi_init_cfg = WIFI_INIT_CONFIG_DEFAULT();
                ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_cfg));

                wifi_config_t wifi {};
                memcpy(wifi.sta.ssid, m_settings.wifi_ssid.data(), std::min(sizeof(wifi.sta.ssid), strlen(m_settings.wifi_ssid.data())));
                memcpy(wifi.sta.password, m_settings.wifi_pass.data(), std::min(sizeof(wifi.sta.password), strlen(m_settings.wifi_pass.data())));
                ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
                ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi));

                ESP_LOGI(TAG, "Starting Wi-Fi...");
                ESP_ERROR_CHECK(esp_wifi_start());
                set_indicator(LED_STATE_WIFI_SEARCHING);

                esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
                config.sync_cb = &on_time_sync;
                esp_netif_sntp_init(&config);
                break;
            }
            case WIFI_EVENT_USER_UPDATE_CREDS: {
                wifi_config_t wifi {};
                esp_wifi_get_config(WIFI_IF_STA, &wifi);

                memset(wifi.sta.ssid, 0, sizeof(wifi.sta.ssid));
                memcpy(wifi.sta.ssid, m_settings.wifi_ssid.data(), std::min(sizeof(wifi.sta.ssid), strlen(m_settings.wifi_ssid.data())));

                memset(wifi.sta.password, 0, sizeof(wifi.sta.password));
                memcpy(wifi.sta.password, m_settings.wifi_pass.data(), std::min(sizeof(wifi.sta.password), strlen(m_settings.wifi_pass.data())));

                esp_wifi_stop();
                esp_wifi_set_config(WIFI_IF_STA, &wifi);
                esp_wifi_start();

                set_indicator(LED_STATE_WIFI_SEARCHING);
                break;
            }
            case WIFI_EVENT_STA_START: {
                esp_wifi_connect();
                break;
            }
            case WIFI_EVENT_STA_DISCONNECTED: {
                xEventGroupClearBits(m_wifi_event_group, WIFI_CONNECTED_BIT);
                esp_wifi_connect();
                set_indicator(LED_STATE_WIFI_SEARCHING);
                post_log({ "Wi-Fi connecting" });
                break;
            }
            default:
                break;
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(m_wifi_event_group, WIFI_CONNECTED_BIT);
        set_indicator(LED_STATE_CONNECTED);
        post_log({ "Network ready" });

        auto uxBits = xEventGroupWaitBits(m_wifi_event_group, MQTT_STARTED_BIT, pdFALSE, pdFALSE, 0);
        if (uxBits & MQTT_STARTED_BIT) {
            esp_mqtt_client_reconnect(m_mqtt);
        }
        else {
            setup_mqtt();
        }
    }
}

void PowerMeterApp::on_mqtt_event(esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t) event_data;

    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            //esp_mqtt_client_subscribe(m_mqtt_handle, MQTT_SUB_TOPIC, 0);
            post_log({ "MQTT ready" });
            break;
        case MQTT_EVENT_DISCONNECTED:
            post_log({ "MQTT disconnect" });
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "Unexpected MQTT event: '%*s' --> '%*s'", event->topic_len, event->topic, event->data_len, event->data);
            break;
        default:
            break;
    }
}

static constexpr const char* POWER_JSON_TEMPLATE = R"({
  "timestamp": %llu,
  "voltage": %s,
  "current": %s,
  "frequency": %s,
  "full_power": %s,
  "real_power": %s,
  "power_factor": %s,
  "energy": %s
})";

static constexpr const char* DEVICE_JSON_TEMPLATE = R"({
  "timestamp": %llu,
  "uptime": %llu,
  "cpu0": %s,
  "cpu1": %s,
  "heap": %u,
  "lblk": %u
})";

void PowerMeterApp::publish_mqtt_message(const MqttMessage& msg)
{
    static constexpr auto format_float = [] (char* buf, size_t size, const char* fmt, float val) {
        if (std::isnan(val))
            strncpy(buf, "null", size);
        else
            snprintf(buf, size, fmt, val);
        return buf;
    };

    auto uxBits = xEventGroupWaitBits(m_wifi_event_group, WIFI_CONNECTED_BIT | MQTT_STARTED_BIT, pdFALSE, pdFALSE, 0);
    if (!(uxBits & MQTT_STARTED_BIT))
        return;
    if (uxBits & WIFI_CONNECTED_BIT) {
        char buf[7][64];
        char json[512];
        std::optional<int> ret;

        do {
            if (msg.time) {
                size_t len = snprintf(json, sizeof(json), POWER_JSON_TEMPLATE,
                    msg.time,
                    format_float(buf[0], sizeof(buf[0]), "%.1f", msg.power.f_v),
                    format_float(buf[1], sizeof(buf[1]), "%.2f", msg.power.f_i),
                    format_float(buf[2], sizeof(buf[2]), "%.2f", msg.power.f_hz),
                    format_float(buf[3], sizeof(buf[3]), "%.1f", msg.power.f_va),
                    format_float(buf[4], sizeof(buf[4]), "%.1f", msg.power.f_w),
                    format_float(buf[5], sizeof(buf[5]), "%.2f", msg.power.f_pf),
                    format_float(buf[6], sizeof(buf[6]), "%.1f", msg.power.f_wh)
                );
                decltype(m_settings.mqtt_topic) topic(m_settings.mqtt_topic);
                strlcat(topic.data(), POWER_TOPIC, sizeof(topic));
                ret = esp_mqtt_client_publish(m_mqtt, topic.data(), json, len, 0, 0);
                if (ret < 0)
                    break;
            }
            if (msg.device.uptime) {
                size_t len = snprintf(json, sizeof(json), DEVICE_JSON_TEMPLATE,
                    msg.time,
                    msg.device.uptime,
                    format_float(buf[0], sizeof(buf[0]), "%4.1f", msg.device.cpu0),
                    format_float(buf[1], sizeof(buf[1]), "%4.1f", msg.device.cpu1),
                    msg.device.heap,
                    msg.device.lblk
                );
                decltype(m_settings.mqtt_topic) topic(m_settings.mqtt_topic);
                strlcat(topic.data(), DEVICE_TOPIC, sizeof(topic));
                ret = esp_mqtt_client_publish(m_mqtt, topic.data(), json, len, 0, 0);
            }
        }
        while (0);

        if (ret.has_value()) {
            if (ret < 0) {
                set_indicator(LED_STATE_MQTT_ERROR);
                post_log({ "MQTT error" });
            }
            else {
                set_indicator(LED_STATE_MQTT_TX);
            }
        }
    }
    else if (m_netif) {
        post_log({ "Wi-Fi disconn" });
    }
}

void PowerMeterApp::set_wifi_ssid(const string_t& value)
{
    ESP_LOGI(TAG, "Setting new Wi-Fi SSID: %s", value.data());
    m_settings.wifi_ssid.fill(0);
    SettingsManager::save(KEY_WIFI_SSID, strncpy(m_settings.wifi_ssid.data(), value.data(), m_settings.wifi_ssid.size() - 1));
    esp_event_post(WIFI_EVENT, WIFI_EVENT_USER_UPDATE_CREDS, nullptr, 0, pdMS_TO_TICKS(10));
}

void PowerMeterApp::set_wifi_pass(const string_t& value)
{
    ESP_LOGI(TAG, "Setting new Wi-Fi passphrase: %s", value.data());
    m_settings.wifi_pass.fill(0);
    SettingsManager::save(KEY_WIFI_PASS, strncpy(m_settings.wifi_pass.data(), value.data(), m_settings.wifi_pass.size() - 1));
    esp_event_post(WIFI_EVENT, WIFI_EVENT_USER_UPDATE_CREDS, nullptr, 0, pdMS_TO_TICKS(10));
}

void PowerMeterApp::set_mqtt_uri(const string_t& value)
{
    setup_mqtt(kDisposing);
    ESP_LOGI(TAG, "Setting new MQTT URI: %s", value.data());
    m_settings.mqtt_uri.fill(0);
    SettingsManager::save(KEY_MQTT_URI, strncpy(m_settings.mqtt_uri.data(), value.data(), m_settings.mqtt_uri.size() - 1));
    setup_mqtt();
}

void PowerMeterApp::set_mqtt_creds(const string_t& value)
{
    setup_mqtt(kDisposing);
    ESP_LOGI(TAG, "Setting new MQTT creds: %s", value.data());
    m_settings.mqtt_creds.fill(0);
    SettingsManager::save(KEY_MQTT_CREDS, strncpy(m_settings.mqtt_creds.data(), value.data(), m_settings.mqtt_creds.size() - 1));
    setup_mqtt();
}

void PowerMeterApp::set_mqtt_topic(const string_t& value)
{
    ESP_LOGI(TAG, "Setting new MQTT topic: %s", value.data());
    m_settings.mqtt_topic.fill(0);
    SettingsManager::save(KEY_MQTT_TOPIC, strncpy(m_settings.mqtt_topic.data(), value.data(), m_settings.mqtt_topic.size() - 1));
}

void PowerMeterApp::on_time_sync(timeval* tv)
{
    if (!static_cast<timeval>(s_start_time).tv_sec) {
        s_start_time = *tv;
    }
    std::tm local;
    if (localtime_r(&tv->tv_sec, &local)) {
        char buf[64];
        strftime(buf, sizeof(buf), "%c", &local);
        ESP_LOGI(TAG, "Time is %s", buf);
    }
}
