#pragma once

#include <freertos/FreeRTOS.h>
#include <esp_log.h>

#include "common.h"

#include "Reader.h"
#include "Compute.h"
#include "Input.h"
#include "Display.h"

// 4096 Hz per channel * 2 channels = 8192 Hz total sampling frequency
#define SAMPLE_FREQ_HZ  8192
#define CHANNEL_FREQ_HZ 4096

// Number of bytes the DMA will push to our task at once.
// 1024 bytes = 256 samples (128 voltage + 128 current) per frame.
#define FRAME_BUF_SIZE  1024
#define SAMPLES_TO_READ (FRAME_BUF_SIZE / SOC_ADC_DIGI_RESULT_BYTES)

// The coarse estimation of sampling shift introduced by the LM358 op-amp on V channel (samples)
#define LM_COARS_SAMPLES_SHIFT  (4)

// Actual sub-fraction sampling shift (measured samples)
#define LM_FINE_SAMPLES_SHIFT   (4.45f)

const float V_COEF = 0.55f;
const float I_COEF = 0.0262f;

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
        ComputeResultMessage,
        InputMessage,
    };

    struct ComputeResultMessage {
        float v_rms;
        float i_rms;
        float apparent_power;
        float real_power;
        double energy;
        float cos_phi;
        float frequency;
        float m_vi_shift;
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

    struct DisplayTaskMessage {
        MessageType type;
        union {
            ComputeResultMessage compute_result;
            InputMessage input;
        };
    };

    void setup_reader(bool disposing = false);
    void setup_display(bool disposing = false);
    void setup_input(bool disposing = false);
    void setup_tasks();

    void display_task();
    void compute_task();
    void reader_task();

    void on_rotate(bool is_ccw);
    void on_click(bool is_long);

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

    volatile float m_v_rms = 0;
    volatile float m_i_rms = 0;
    volatile float m_apparent_power = 0;
    volatile float m_real_power = 0;
    volatile float m_cos_phi = 0;
    volatile float m_frequency = 0;
    volatile float m_vi_sample_shift = 0;
    volatile double m_accumulated_energy = 0;

    volatile bool m_stop_tasks = false;

    TaskHandle_t m_adc_task {};
    TaskHandle_t m_compute_task {};
    TaskHandle_t m_display_task {};

    QueueHandle_t m_display_queue {};
};
