#pragma once

#include <freertos/FreeRTOS.h>
#include <esp_log.h>
#include <esp_event.h>
#include <esp_netif.h>
#include <mqtt_client.h>
#include <hal/adc_ll.h>
#include <soc/soc.h>
#include <cmath>

#include "common.h"
#include "RotaryEncoder.h"
#include "Button.h"
#include "LED.h"
#include "Indicator.h"
#include "SettingsManager.h"

#include "Reader.h"
#include "Compute.h"
#include "ConsoleInput.h"
#include "Interface.h"
#include "Telemetry.h"

// 4000 Hz per channel * 2 channels = 8000 Hz total sampling frequency
#define SAMPLE_FREQ_HZ     8000

// Actual rate per channel, mirrors adc_hal_digi_sample_freq_config(): 80 MHz APB / 16 = 5 MHz,
// trigger interval truncated to whole cycles (312.5 -> 312), giving 4006.41 Hz instead of 4000
constexpr uint32_t ADC_CLKM_DIV = (ADC_LL_CLKM_DIV_NUM_DEFAULT + 1) * ADC_LL_CLKM_DIV_B_DEFAULT + ADC_LL_CLKM_DIV_A_DEFAULT;
constexpr uint32_t ADC_TRIGGER_INTERVAL = APB_CLK_FREQ / (ADC_CLKM_DIV * 2 * SAMPLE_FREQ_HZ);
constexpr double CHANNEL_FREQ_HZ = double(APB_CLK_FREQ) / ADC_CLKM_DIV / (2 * ADC_TRIGGER_INTERVAL) / 2;

// We want to process exactly 10 AC cycles (at 50Hz and 4000 sampling rate)
const int CHUNK_SIZE = 800;     // 4000 / 50 * 10

// Number of bytes the DMA will push to our task at once.
// 6400 bytes = 1600 samples (800 voltage + 800 current) per frame (4 bytes)
const size_t SAMPLES_TO_READ = CHUNK_SIZE * 2;
const size_t FRAME_BUF_SIZE = SAMPLES_TO_READ * SOC_ADC_DIGI_DATA_BYTES_PER_CONV;

// Sampling shift between V and I channels introduced by the LM358 op-amp (measured samples)
#define LM_SAMPLES_SHIFT (4.60f)

// Define the absolute maximum real-world voltage before considering it a glitch
const float MAX_EXPECTED_V_RMS = 270.0f;

constexpr bool kDisposing = true;

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
        EncoderInputMessage,
        ConsoleInputMessage,
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
        struct PowerMetrics {
            float f_v;
            float f_i;
            float f_va;
            float f_w;
            float f_hz;
            float f_wh;
            float f_pf;
        };
        struct DeviceMetrics {
            time_t uptime;
            float  cpu0;
            float  cpu1;
            size_t heap;
            size_t lblk;
        };

        time_t        time;
        PowerMetrics  power;
        DeviceMetrics device;
    };

    enum class EncoderInputAction {
        Next,
        Prev,
        Confirm,
        Back,
    };

    struct EncoderInputMessage {
        EncoderInputAction action;
    };

    enum class ConsoleInputAction {
        None,
        Decimal,
        Float,
        String,
    };

    struct ConsoleInputMessage {
        ConsoleInputAction action;
        union {
            int decimal_value;
            float float_value;
            string_t string_value;
        };
    };

    struct LogMessage {
        display_line_t message;
    };

    struct InterfaceTaskMessage {
        MessageType type;
        union {
            ResultMessage result;
            EncoderInputMessage encoder_input;
            ConsoleInputMessage console_input;
            LogMessage log;
        };
    };

    friend struct DisplayContext;

    // Set by each task as its very last action, before it deletes itself
    static constexpr EventBits_t READER_EXIT_BIT    = BIT0;
    static constexpr EventBits_t COMPUTE_EXIT_BIT   = BIT1;
    static constexpr EventBits_t INTERFACE_EXIT_BIT = BIT2;
    static constexpr EventBits_t TELEMETRY_EXIT_BIT = BIT3;
    static constexpr EventBits_t INDICATOR_EXIT_BIT = BIT4;
    static constexpr EventBits_t ALL_TASKS_EXIT_BITS = BIT0 | BIT1 | BIT2 | BIT3 | BIT4;

    static void on_time_sync(timeval* tv);

    void setup_ipc(bool disposing = false);
    void setup_reader(bool disposing = false);
    void setup_interface(bool disposing = false);
    void start_tasks();
    void stop_tasks();

    void setup_telemetry(bool disposing = false);
    void setup_wifi(bool disposing = false);
    void setup_mqtt(bool disposing = false);

    void interface_task();
    void compute_task();
    void reader_task();
    void telemetry_task();
    void indicator_task();

    void mqtt_start();
    void mqtt_stop();
    void on_mqtt_event(esp_event_base_t event_base, int32_t event_id, void* event_data);
    void on_wifi_event(esp_event_base_t event_base, int32_t event_id, void* event_data);
    void publish_mqtt_message(const MqttMessage& msg);

    void process_result(const ResultMessage& result);
    void process_encoder_input(const EncoderInputMessage& encoder);
    void process_console_input(const ConsoleInputMessage& input);
    void process_log(const LogMessage& log);

    void on_encoder_rotate(bool is_ccw);
    void on_encoder_click(bool is_long);
    void on_console_input_event(esp_event_base_t event_base, int32_t event_id, void* event_data);

    void setup_console_input(bool disposing = false);
    bool get_console_input(char* buffer, size_t max_len);
    bool input_value(int& value);
    bool input_value(float& value);
    bool input_value(string_t& value);

    void set_wifi_ssid(const string_t& value);
    void set_wifi_pass(const string_t& value);
    void set_mqtt_uri(const string_t& value);
    void set_mqtt_creds(const string_t& value);
    void set_mqtt_topic(const string_t& value);

    void get_device_metrics(MqttMessage& m);
    void post_log(display_line_t message);
    void set_indicator(led_mode_t mode);

private:
    adc_channel_t m_adc_v_channel_id {};
    adc_channel_t m_adc_i_channel_id {};
    adc_cali_handle_t m_adc_v_cali {};
    adc_cali_handle_t m_adc_i_cali {};
    adc_continuous_handle_t m_adc {};

    LockFreeRingBuffer<float, CHUNK_SIZE * 4> m_v_ring_buffer;
    LockFreeRingBuffer<float, CHUNK_SIZE * 4> m_i_ring_buffer;

    // Will compute the AC center value to rip off the DC bias (in a case estimated 1.65 mV bias will drift eventually)
    DCBlocker m_v_dc_blocker;
    DCBlocker m_i_dc_blocker;

    SSD1306_t m_oled {};
    RotaryEncoder m_encoder;
    Button m_encoder_key;
    CPULoadMeter m_cpu_meter;
    LED m_led;

    esp_netif_t* m_netif = nullptr;
    EventGroupHandle_t m_wifi_event_group {};
    esp_mqtt_client_handle_t m_mqtt {};

    DeviceSettings m_settings;

    float m_v_rms = 0;
    float m_i_rms = 0;
    float m_apparent_power = 0;
    float m_real_power = 0;
    //float m_cos_phi = 0;
    float m_frequency = 0;
    //float m_vi_sample_shift = 0;

    // ***** DEBUG *****
    //volatile uint32_t m_error_count = 0;
    //volatile uint32_t m_phase_shift = 0;
    // *****

    std::atomic<double> m_acc_energy {};
    std::atomic_bool m_acc_energy_saved {};
    std::atomic_bool m_stop_tasks {};

    TaskHandle_t m_compute_task {};

    QueueHandle_t m_interface_queue {};
    QueueHandle_t m_telemetry_queue {};
    QueueHandle_t m_indicator_queue {};
    EventGroupHandle_t m_task_exit_group {};

    esp_event_loop_handle_t m_console_event_loop {};
};
