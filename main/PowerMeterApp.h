#pragma once

#include <freertos/FreeRTOS.h>
#include <esp_log.h>
#include <esp_event.h>
#include <esp_netif.h>
#include <mqtt_client.h>

#include "common.h"

#include "Reader.h"
#include "Compute.h"
#include "Input.h"
#include "Display.h"

// 4000 Hz per channel * 2 channels = 8000 Hz total sampling frequency
#define SAMPLE_FREQ_HZ  8000
#define CHANNEL_FREQ_HZ 4000.0

// Number of bytes the DMA will push to our task at once.
// 1600 bytes = 400 samples (200 voltage + 200 current) per frame.
#define FRAME_BUF_SIZE  1600
#define SAMPLES_TO_READ (FRAME_BUF_SIZE / SOC_ADC_DIGI_RESULT_BYTES)

// We want to process exactly 10 AC cycles (at 50Hz and 4000 sampling rate)
const int CHUNK_SIZE = 800;     // 4000 / 50 * 10

// The coarse estimation of sampling shift introduced by the LM358 op-amp on V channel (samples)
#define LM_COARS_SAMPLES_SHIFT  (4)

// Actual sub-fraction sampling shift (measured samples)
#define LM_FINE_SAMPLES_SHIFT   (4.35f)

// Set noise floor level to avoid spurious readings
const float CURRENT_NOISE_FLOOR = 0.1f; // 100 mA

const float V_COEF = 0.55f;
const float I_COEF = 0.0262f;

typedef std::array<char, 17> display_line_t;

class PowerMeterApp
{
public:
    PowerMeterApp();
    ~PowerMeterApp();

    bool is_stop_tasks() const {
        return m_stop_tasks;
    }

private:
    enum class MessageType {
        ResultMessage,
        InputMessage,
        LogMessage,
    };

    struct ResultMessage {
        float v_rms;
        float i_rms;
        float apparent_power;
        float real_power;
        double energy;
        float frequency;
        //float vi_shift;
    };

    struct MqttMessage {
        float f_v;
        float f_i;
        float f_va;
        float f_w;
        float f_hz;
        float f_wh;
        float f_pf;
    };

    enum class InputAction {
        Next,
        Prev,
        Confirm,
        Back,
    };

    struct InputMessage {
        InputAction action;
    };

    struct LogMessage {
        display_line_t message;
    };

    struct DisplayTaskMessage {
        MessageType type;
        union {
            ResultMessage result;
            InputMessage input;
            LogMessage log;
        };
    };

    void setup_reader(bool disposing = false);
    void setup_display(bool disposing = false);
    void setup_input(bool disposing = false);
    void setup_tasks();

    void setup_telemetry(bool disposing = false);
    void setup_nvs(bool disposing = false);
    void setup_wifi(bool disposing = false);

    void display_task();
    void compute_task();
    void reader_task();
    void telemetry_task();

    void mqtt_start();
    void mqtt_stop();
    void on_mqtt_event(esp_event_base_t event_base, int32_t event_id, void* event_data);
    void on_wifi_event(esp_event_base_t event_base, int32_t event_id, void* event_data);
    void publish_mqtt_message(const MqttMessage& msg);

    void process_result(const ResultMessage& result);
    void process_input(const InputMessage& result);
    void process_log(const LogMessage& result);


    void on_rotate(bool is_ccw);
    void on_click(bool is_long);

    void post_log(display_line_t message);

private:
    adc_channel_t m_adc_v_channel_id {};
    adc_channel_t m_adc_i_channel_id {};
    adc_cali_handle_t m_adc_v_cali_handle {};
    adc_cali_handle_t m_adc_i_cali_handle {};
    adc_continuous_handle_t m_adc_handle {};

    LockFreeRingBuffer<float, 4096> m_v_ring_buffer;
    LockFreeRingBuffer<float, 4096> m_i_ring_buffer;

    // Will compute the AC center value to rip off the DC bias (in a case estimated 1.65 mV bias will drift eventually)
    DCBlocker m_v_dc_blocker;
    DCBlocker m_i_dc_blocker;

    SSD1306_t m_oled {};
    RotaryEncoder m_encoder;
    Button m_encoder_key;

    CPULoadMeter m_cpu_meter;

    esp_netif_t* m_netif = nullptr;
    EventGroupHandle_t m_wifi_event_group {};
    esp_mqtt_client_handle_t m_mqtt_handle {};

    float m_v_rms = 0;
    float m_i_rms = 0;
    float m_apparent_power = 0;
    float m_real_power = 0;
    //float m_cos_phi = 0;
    float m_frequency = 0;
    //float m_vi_sample_shift = 0;
    double m_accumulated_energy = 0;

    volatile bool m_stop_tasks = false;

    TaskHandle_t m_adc_task {};
    TaskHandle_t m_compute_task {};
    TaskHandle_t m_display_task {};

    QueueHandle_t m_display_queue {};
    QueueHandle_t m_telemetry_queue {};
};
