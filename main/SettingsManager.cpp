#include "SettingsManager.h"
#include "common.h"

#include <esp_log.h>

void SettingsManager::setup(bool disposing)
{
    if (disposing) {
        nvs_flash_deinit();
    }
    else {
        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_LOGW(TAG, "NVS partition corrupted or version mismatch. Erasing...");
            ESP_ERROR_CHECK(nvs_flash_erase());
            ESP_ERROR_CHECK(nvs_flash_init());
        }
        else {
            ESP_ERROR_CHECK(ret);
        }
    }
}

void SettingsManager::load(DeviceSettings& settings)
{
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK)
        return;

    size_t len;
    len = settings.wifi_ssid.size();
    if (nvs_get_str(handle, KEY_WIFI_SSID, settings.wifi_ssid.data(), &len) != ESP_OK)
        settings.wifi_ssid.fill(0);

    len = settings.wifi_pass.size();
    if (nvs_get_str(handle, KEY_WIFI_PASS, settings.wifi_pass.data(), &len) != ESP_OK)
        settings.wifi_pass.fill(0);

    len = settings.mqtt_uri.size();
    if (nvs_get_str(handle, KEY_MQTT_URI, settings.mqtt_uri.data(), &len) != ESP_OK)
        settings.mqtt_uri.fill(0);

    len = settings.mqtt_creds.size();
    if (nvs_get_str(handle, KEY_MQTT_CREDS, settings.mqtt_creds.data(), &len) != ESP_OK)
        settings.mqtt_creds.fill(0);

    len = settings.mqtt_topic.size();
    if (nvs_get_str(handle, KEY_MQTT_TOPIC, settings.mqtt_topic.data(), &len) != ESP_OK)
        settings.mqtt_topic.fill(0);

    nvs_get_u32(handle, KEY_MQTT_PERIOD, reinterpret_cast<uint32_t*>(&settings.mqtt_period));
    nvs_get_u32(handle, KEY_I_NOISE_FLOOR, reinterpret_cast<uint32_t*>(&settings.i_noise_floor));
    nvs_get_u32(handle, KEY_I_COEF, reinterpret_cast<uint32_t*>(&settings.i_coef));
    nvs_get_u32(handle, KEY_V_COEF, reinterpret_cast<uint32_t*>(&settings.v_coef));
    nvs_get_u32(handle, KEY_V_ACC_ENERGY_SAVE, reinterpret_cast<uint32_t*>(&settings.v_acc_energy_save));
    nvs_get_u64(handle, KEY_ACC_ENERGY, reinterpret_cast<uint64_t*>(&settings.acc_energy));
    nvs_close(handle);
}

void SettingsManager::save(const char* key, const char* value) {
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK) {
        nvs_set_str(handle, key, value);
        nvs_commit(handle);
        nvs_close(handle);
        ESP_LOGI(TAG, "Saved string [%s]", key);
    }
}

void SettingsManager::save(const char* key, float value) {
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK) {
        nvs_set_u32(handle, key, *reinterpret_cast<uint32_t*>(&value));
        nvs_commit(handle);
        nvs_close(handle);
        ESP_LOGI(TAG, "Saved float [%s]: %f", key, value);
    }
}

void SettingsManager::save(const char* key, double value) {
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK) {
        nvs_set_u64(handle, key, *reinterpret_cast<uint64_t*>(&value));
        nvs_commit(handle);
        nvs_close(handle);
        ESP_LOGI(TAG, "Saved double [%s]: %f", key, value);
    }
}

void SettingsManager::save(const char* key, uint8_t value) {
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK) {
        nvs_set_u8(handle, key, value);
        nvs_commit(handle);
        nvs_close(handle);
        ESP_LOGI(TAG, "Saved byte [%s]: %u", key, value);
    }
}
