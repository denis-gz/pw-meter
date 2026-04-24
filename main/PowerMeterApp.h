#pragma once

#include <esp_adc/adc_continuous.h>
#include <esp_adc/adc_cali.h>

#include <freertos/FreeRTOS.h>

#include "RingBuffer.h"
#include "DCBlocker.h"
#include "Button.h"
#include "CPULoadMeter.h"

// 4096 Hz per channel * 2 channels = 8192 Hz total sampling frequency
#define SAMPLE_FREQ_HZ  8192

// Number of bytes the DMA will push to our task at once.
// 1024 bytes = 256 samples (128 voltage + 128 current) per frame.
#define FRAME_BUF_SIZE  1024
#define SAMPLES_TO_READ (FRAME_BUF_SIZE / SOC_ADC_DIGI_RESULT_BYTES)

// The coarse estimation of sampling shift introduced by the LM358 op-amp on V channel (samples)
#define LM_COARS_SAMPLES_SHIFT  (4)

// Actual sub-fraction sampling shift (measured samples)
#define LM_FINE_SAMPLES_SHIFT   (4.2f)

const float V_COEF = 0.55f;
const float I_COEF = 0.0264f;

class PowerMeterApp
{
public:
    PowerMeterApp();
    ~PowerMeterApp();

    bool is_stop_tasks() const {
        return m_stop_tasks;
    }

private:
    void setup_adc();
    void setup_tasks();

    void reader_task();
    void compute_task();
    void extracted();
    void display_task();

    struct ComputeResultMessage {
        float v_rms;
        float i_rms;
        float apparent_power;
        float real_power;
        float cos_phi;
        float frequency;
    };

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

    Button m_button;
    CPULoadMeter m_cpu_meter;

    volatile float m_v_rms = 0;
    volatile float m_i_rms = 0;
    volatile float m_apparent_power = 0;
    volatile float m_real_power = 0;
    volatile float m_cos_phi = 0;
    volatile float m_frequency = 0;
    volatile float m_vi_sample_shift = 0;

    volatile bool m_stop_tasks = false;

    TaskHandle_t m_adc_task {};
    TaskHandle_t m_compute_task {};
    TaskHandle_t m_display_task {};

    QueueHandle_t m_display_queue {};
};
