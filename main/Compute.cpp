#include "PowerMeterApp.h"

#include <float.h>
#include <esp_log.h>
#include <esp_dsp.h>

#include "common.h"

void PowerMeterApp::compute_task()
{
    ESP_LOGI(TAG, "compute_task: started");

    // We want to process exactly 10 AC cycles (at 50Hz and 4096 sampling rate)
    const int CHUNK_SIZE = 819;     // 4096 / 50 * 10

    // These will contain the centered AC_V and AC_I values
    float v_array[CHUNK_SIZE] = {};
    float i_array[CHUNK_SIZE] = {};

    // We will also build a new, perfectly aligned current array of exactly 819 samples,
    // to accomodate 4 samples phase shift
    float aligned_v_array[CHUNK_SIZE] = {};

    // This is used to save the last 4 samples of the previous chunk since the current
    // goes ahead of voltage due to LM358 singal processing delay
    float prev_v_history[LM_COARS_SAMPLES_SHIFT] = {};

    uint64_t display_task_trigger = 0;

    while (!m_stop_tasks) {
        // Check if we have enough samples in BOTH buffers to do a full calculation
        if (m_v_ring_buffer.available() >= CHUNK_SIZE &&
            m_i_ring_buffer.available() >= CHUNK_SIZE) {

            // Sacrifice the first chunk to perform initial DC bias learning
            if (std::isnan(m_v_dc_blocker.get_current_dc_bias())) {
                for (int i = 0; i < CHUNK_SIZE; i++) {
                    float raw_v, raw_i;
                    m_v_ring_buffer.pop(raw_v);
                    m_i_ring_buffer.pop(raw_i);
                    v_array[i] = raw_v;
                    i_array[i] = raw_i;
                }
                {   float maxV = -FLT_MAX;
                    float minV = FLT_MAX;
                    for (int i = 0; i < CHUNK_SIZE; i++) {
                        if (maxV < v_array[i])
                            maxV = v_array[i];
                        if (minV > v_array[i])
                            minV = v_array[i];
                        //ESP_LOGI(TAG, "Initial V: %04d %0.1f", i, v_array[i]);
                        //if (i % 100 == 0)
                        //    vTaskDelay(pdMS_TO_TICKS(100));
                    }
                    float v_bias = (maxV + minV) / 2.0f;
                    m_v_dc_blocker.set_current_dc_bias(v_bias);
                    ESP_LOGI(TAG, "V DC bias: minV = %0.1f, maxV = %0.1f, bias = %0.1f\n", minV, maxV, v_bias);
                }
                {   float maxV = -FLT_MAX;
                    float minV = FLT_MAX;
                    for (int i = 0; i < CHUNK_SIZE; i++) {
                        if (maxV < i_array[i])
                            maxV = i_array[i];
                        if (minV > i_array[i])
                            minV = i_array[i];
                        //ESP_LOGI(TAG, "Initial I: %04d %0.1f", i, i_array[i]);
                        //if (i % 100 == 0)
                        //    vTaskDelay(pdMS_TO_TICKS(100));
                    }
                    float i_bias = (maxV + minV) / 2.0f;
                    m_i_dc_blocker.set_current_dc_bias(i_bias);
                    ESP_LOGI(TAG, "I DC bias: minV = %0.1f, maxV = %0.1f, bias = %0.1f\n", minV, maxV, i_bias);
                }
                // Save 4 last centered bytes for the next chunk
                for (int i = 0; i < LM_COARS_SAMPLES_SHIFT; i++) {
                    prev_v_history[i] = m_v_dc_blocker.process(v_array[CHUNK_SIZE - LM_COARS_SAMPLES_SHIFT + i]);
                }
                continue;
            }

            // Pull the data out of the lock-free buffers and strip the DC bias
            for (int i = 0; i < CHUNK_SIZE; i++) {
                float raw_v, raw_i;
                m_v_ring_buffer.pop(raw_v);
                m_i_ring_buffer.pop(raw_i);
                v_array[i] = m_v_dc_blocker.process(raw_v);
                i_array[i] = m_i_dc_blocker.process(raw_i);
            }

            // Get first 4 samples from the previous chunk
            memcpy(&aligned_v_array[0], &prev_v_history[0], LM_COARS_SAMPLES_SHIFT * sizeof(float));
            // Fill the rest 815 samples from the current chunk
            memcpy(&aligned_v_array[LM_COARS_SAMPLES_SHIFT], &v_array[0], (CHUNK_SIZE - LM_COARS_SAMPLES_SHIFT) * sizeof(float));
            // Save the last 4 samples of the current chunk for the next reading
            memcpy(&prev_v_history[0], &v_array[CHUNK_SIZE - LM_COARS_SAMPLES_SHIFT], LM_COARS_SAMPLES_SHIFT * sizeof(float));

            // **** DEBUG *****
        /*
            //ESP_ERROR_CHECK(adc_continuous_stop(m_adc_handle));
            {   float maxV = -std::numeric_limits<float>::max();
                float minV = std::numeric_limits<float>::max();
                ESP_LOGI(TAG, "Voltages chunks:");
                for (int i = 0; i < CHUNK_SIZE / 3; ++i) {
                    auto v = v_array[i];
                    if (maxV < v)
                        maxV = v;
                    if (minV > v)
                        minV = v;
                    ESP_LOGI(TAG, "V: %04d %0.1f", i, v_array[i]);
                    if (i % 100 == 0)
                        vTaskDelay(pdMS_TO_TICKS(100));
                }
                ESP_LOGI(TAG, "minV: %0.1f, maxV: %0.1f, swing: %0.1f, DC bias: %0.2f\n", minV, maxV, maxV-minV, m_v_dc_blocker.get_current_dc_bias());
            }
            {   float maxI = -std::numeric_limits<float>::max();
                float minI = std::numeric_limits<float>::max();
                ESP_LOGI(TAG, "Currents chunks:");
                for (int i = 0; i < CHUNK_SIZE / 3; ++i) {
                    auto v = i_array[i];
                    if (maxI < v)
                        maxI = v;
                    if (minI > v)
                        minI = v;
                    ESP_LOGI(TAG, "I: %04d %0.1f", i, i_array[i]);
                    if (i % 100 == 0)
                        vTaskDelay(pdMS_TO_TICKS(100));
                }
                ESP_LOGI(TAG, "minI: %0.1f, maxI: %0.1f, swing: %0.1f, DC bias: %0.2f\n", minI, maxI, maxI-minI, m_i_dc_blocker.get_current_dc_bias());
            }
            break;
         */
            // **** DEBUG *****

            // Calculate the grid frequency

            float v_first_crossing_index = NAN;
            float v_last_crossing_index = NAN;
            int v_cycle_count = 0;

            float i_first_crossing_index = NAN;
            float i_last_crossing_index = NAN;
            int i_cycle_count = 0;

            // Iterate through the voltage array to find rising edges (negative to positive)
            for (int i = 1; i < CHUNK_SIZE; i++) {
                // Find zero-crossing indices for voltage sine wave
                if (aligned_v_array[i - 1] < 0.0f && aligned_v_array[i] >= 0.0f) {
                    // Calculate the exact sub-sample fractional index of the crossing
                    float fraction = (0.0f - aligned_v_array[i - 1]) / (aligned_v_array[i] - aligned_v_array[i - 1]);
                    float exact_index = (i - 1) + fraction;
                    if (v_cycle_count == 0) {
                        v_first_crossing_index = exact_index;
                    }
                    else {
                        v_last_crossing_index = exact_index;
                    }
                    v_cycle_count++;
                }

                // Do the same for current as well
                if (i_array[i - 1] < 0.0f && i_array[i] >= 0.0f) {
                    // Calculate the exact sub-sample fractional index of the crossing
                    float fraction = (0.0f - i_array[i - 1]) / (i_array[i] - i_array[i - 1]);
                    float exact_index = (i - 1) + fraction;
                    if (i_cycle_count == 0) {
                        i_first_crossing_index = exact_index;
                    }
                    else {
                        i_last_crossing_index = exact_index;
                    }
                    i_cycle_count++;
                }
            }

            // We need at least 2 crossings to measure the time of 1 full cycle
            if (v_cycle_count >= 2) {
                int total_full_cycles_measured = v_cycle_count - 1;
                float delta_index = v_last_crossing_index - v_first_crossing_index;

                // Frequency = (Cycles / Samples) * Sampling_Rate
                m_frequency = (total_full_cycles_measured / delta_index) * CHANNEL_FREQ_HZ;

                // ***** Calculate the power factor *****
                if (v_first_crossing_index >= 0.0f && i_first_crossing_index >= 0.0f) {

                    // Calculate the raw, uncorrected delay in samples (if current lags voltage, this will be positive)
                    float measured_sample_delay = i_first_crossing_index - v_first_crossing_index;

                    // The LC Tech board artificially delays the voltage, making current LOOK like it's ahead.
                    // We subtracted the integral part when we aligned the chunk, so do it now for the fraction part
                    // to finally fix the reality.
                    float true_sample_delay = measured_sample_delay - (LM_FINE_SAMPLES_SHIFT - LM_COARS_SAMPLES_SHIFT);

                    // Convert the true sample delay into Radians
                    // Samples_Per_Cycle = Sampling_Rate / Grid_Frequency
                    // float samples_per_cycle = 4096.0f / m_frequency;
                    float samples_per_cycle = delta_index / total_full_cycles_measured;     // optimized

                    // Phase Angle (in Radians) = (Delay / Samples per Cycle) * 2π
                    float true_angle_radians = (true_sample_delay / samples_per_cycle) * (2.0f * M_PI);

                    // Calculate True Power Factor
                    float cos_phi = std::cos(true_angle_radians);

                    // Optional: Constrain floating point quirks
                    // (std::cos can rarely spit out 1.000001 due to float imprecision)
                    if (cos_phi > 1.0f)
                        cos_phi = 1.0f;
                    if (cos_phi < -1.0f)
                        cos_phi = -1.0f;

                    m_cos_phi = cos_phi;
                }
            }

            m_vi_sample_shift = (i_first_crossing_index - v_first_crossing_index) + (i_last_crossing_index - v_last_crossing_index);
            m_vi_sample_shift /= 2.0f;

            // --- DATA IS NOW READY FOR ESP-DSP ---
            float v_sq_sum = 0;
            float i_sq_sum = 0;

            dsps_dotprod_f32(aligned_v_array, aligned_v_array, &v_sq_sum, CHUNK_SIZE);
            dsps_dotprod_f32(i_array, i_array, &i_sq_sum, CHUNK_SIZE);

            m_v_rms = sqrt(v_sq_sum / CHUNK_SIZE) * V_COEF;
            m_i_rms = sqrt(i_sq_sum / CHUNK_SIZE) * I_COEF;

            if (m_i_rms < 0.05f) {
                m_i_rms = 0.0f;
                m_apparent_power = 0.0f;
                m_real_power = 0.0f;
                m_cos_phi = NAN;
            }
            else {
                m_apparent_power = m_v_rms * m_i_rms;       // VA

                float real_pwr = 0;
                dsps_dotprod_f32(aligned_v_array, i_array, &real_pwr, CHUNK_SIZE);
                real_pwr /= CHUNK_SIZE;
                real_pwr *= V_COEF * I_COEF;                // Watts
                m_real_power = real_pwr;

                // 819 samples at 4096Hz is exactly 0.199951 seconds (0.00005554 hours)
                const double CHUNK_TIME_HOURS = (CHUNK_SIZE / (double) CHANNEL_FREQ_HZ) / 3600.0;
                m_accumulated_energy += (double) m_real_power * CHUNK_TIME_HOURS;
            }
        }
        else {
            // Yield to the RTOS scheduler for a few milliseconds while buffers fill
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        DisplayTaskMessage qmsg {
            .type = MessageType::ComputeResultMessage,
            .compute_result = {
                .v_rms = m_v_rms,
                .i_rms = m_i_rms,
                .apparent_power = m_apparent_power,
                .real_power = m_real_power,
                .energy = m_accumulated_energy,
                .cos_phi = m_cos_phi,
                .frequency = m_frequency,
                .m_vi_shift = m_vi_sample_shift,
            },
        };
        if (xQueueSend(m_display_queue, &qmsg, 0) != pdPASS) {
            ESP_LOGI(TAG, "Warning: Display queue is full, dropping message\n");
        }
    }

    ESP_LOGI(TAG, "compute_task: finished");
    vTaskDelete(nullptr);
}
