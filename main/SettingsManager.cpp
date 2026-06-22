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

    get_float(handle, KEY_I_NOISE_FLOOR, settings.i_noise_floor);
    get_float(handle, KEY_I_COEF, settings.i_coef);
    get_float(handle, KEY_V_COEF, settings.v_coef);

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
        set_float(handle, key, value);
        nvs_commit(handle);
        nvs_close(handle);
        ESP_LOGI(TAG, "Saved float [%s]: %f", key, value);
    }
}

void SettingsManager::save(const char* key, uint32_t value) {
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK) {
        nvs_set_u32(handle, key, value);
        nvs_commit(handle);
        nvs_close(handle);
        ESP_LOGI(TAG, "Saved integer [%s]: %u", key, value);
    }
}

esp_err_t SettingsManager::set_float(nvs_handle_t handle, const char* key, float value)
{
    union {
        float val_f32;
        uint32_t val_u32;
    };
    val_f32 = value;
    return nvs_set_u32(handle, key, val_u32);
}

esp_err_t SettingsManager::get_float(nvs_handle_t handle, const char* key, float& value)
{
    union {
        float val_f32;
        uint32_t val_u32;
    };
    esp_err_t err = nvs_get_u32(handle, key, &val_u32);
    if (err == ESP_OK)
        value = val_f32;
    return err;
}
