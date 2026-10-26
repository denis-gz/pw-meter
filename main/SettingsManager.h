#pragma once

#include <nvs_flash.h>
#include <esp_wifi_types_generic.h>

#include "common.h"

#define MAX_SSID_LEN_Z  (MAX_SSID_LEN + 1)
#define MAX_PASS_LEN_Z  (MAX_PASSPHRASE_LEN + 1)
#define MAX_URI_LEN_Z   (256)
#define MAX_TOPIC_LEN_Z (128)
#define MAX_CREDS_LEN_Z (132)

constexpr const char* KEY_WIFI_SSID = "wifi_ssid";
constexpr const char* KEY_WIFI_PASS = "wifi_pass";
constexpr const char* KEY_MQTT_URI = "mqtt_uri";
constexpr const char* KEY_MQTT_CREDS = "mqtt_creds";
constexpr const char* KEY_MQTT_TOPIC = "mqtt_topic";
constexpr const char* KEY_MQTT_PERIOD = "mqtt_period";
constexpr const char* KEY_I_NOISE_FLOOR = "i_noise_floor";
constexpr const char* KEY_V_NOISE_FLOOR = "v_noise_floor";
constexpr const char* KEY_I_COEF = "i_coef";
constexpr const char* KEY_V_COEF = "v_coef";
constexpr const char* KEY_V_ACC_ENERGY_SAVE = "v_acc_energy_save";
constexpr const char* KEY_ACC_ENERGY = "acc_energy";

struct DeviceSettings
{
    static constexpr const float I_NOISE_FLOOR = 0.10f;         // 100 mA
    static constexpr const float V_NOISE_FLOOR = 5.0f;          // 5 V
    static constexpr const float I_COEF = 0.02699f;
    static constexpr const float V_COEF = 0.48565f;
    static constexpr const float V_HYSTERESIS = 15.0f;          // 15 V
    static constexpr const float MIN_FREQ_HZ = 15.0f;           // 15 Hz
    static constexpr const float MAX_FREQ_HZ = 65.0f;           // 65 Hz
    static constexpr const float V_ACC_ENERGY_SAVE = 100.0f;    // 100 V
    static constexpr const uint8_t MQTT_PERIOD = 1;             // 1 sec
    static constexpr const uint8_t MIN_MQTT_PERIOD = 1;         // 1 sec
    static constexpr const uint8_t MAX_MQTT_PERIOD = 255;       // 255 sec
    static constexpr const uint16_t WAVE_RECORD_TIME = 180;     // 3 h, in minutes
    static constexpr const uint16_t WAVE_RECORD_STEP = 15;      // 15 min
    static constexpr const uint16_t MIN_WAVE_RECORD_TIME = 15;  // 15 min
    static constexpr const uint16_t MAX_WAVE_RECORD_TIME = 1440;// 24 h

    char_array_t<MAX_SSID_LEN_Z>  wifi_ssid {};
    char_array_t<MAX_PASS_LEN_Z>  wifi_pass {};
    char_array_t<MAX_URI_LEN_Z>   mqtt_uri {};
    char_array_t<MAX_CREDS_LEN_Z> mqtt_creds {};                // Use ':' char as a separator, like "user:pass"
    char_array_t<MAX_TOPIC_LEN_Z> mqtt_topic {};

    uint8_t mqtt_period = MQTT_PERIOD;
    float i_noise_floor = I_NOISE_FLOOR;
    float v_noise_floor = V_NOISE_FLOOR;
    float i_coef = I_COEF;
    float v_coef = V_COEF;
    float v_acc_energy_save = V_ACC_ENERGY_SAVE;
    double acc_energy {};
};

class SettingsManager
{
public:
    static void setup(bool disposing = false);
    static void load(DeviceSettings& settings);
    static void save(const char* key, const char* value);
    static void save(const char* key, float value);
    static void save(const char* key, double value);
    static void save(const char* key, uint8_t value);

private:
    static constexpr const char* NVS_NAMESPACE = "pw-meter";
};
