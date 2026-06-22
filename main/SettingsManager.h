#pragma once

#include <array>
#include <nvs_flash.h>

constexpr size_t MAX_SSID_LEN = 33;
constexpr size_t MAX_PASS_LEN = 65; 

constexpr const char* KEY_WIFI_SSID = "wifi_ssid";
constexpr const char* KEY_WIFI_PASS = "wifi_pass";
constexpr const char* KEY_V_COEF = "v_coef";
constexpr const char* KEY_I_COEF = "i_coef";
constexpr const char* KEY_I_NOISE_FLOOR = "i_noise_floor";

struct DeviceSettings
{
    static constexpr const float V_COEF = 0.48565f;
    static constexpr const float I_COEF = 0.02699f;
    static constexpr const float I_NOISE_FLOOR = 0.10f; // 100 mA

    std::array<char, MAX_SSID_LEN> wifi_ssid {};
    std::array<char, MAX_PASS_LEN> wifi_pass {};
    
    float v_coef = V_COEF;
    float i_coef = I_COEF;
    float i_noise_floor = I_NOISE_FLOOR;
};

class SettingsManager
{
public:
    static void setup(bool disposing = false);
    static void load(DeviceSettings& settings);
    static void save(const char* key, const char* value);
    static void save(const char* key, float value);
    static void save(const char* key, uint32_t value);

private:
    static constexpr const char* NVS_NAMESPACE = "pw-meter";
    
    // Helpers to safely pack/unpack floats into NVS integers
    static esp_err_t set_float(nvs_handle_t handle, const char* key, float value);
    static esp_err_t get_float(nvs_handle_t handle, const char* key, float& value);
};
