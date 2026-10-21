#include "PowerMeterApp.h"

#include <esp_log.h>
#include <esp_dsp.h>
#include <esp_timer.h>
#include <algorithm>
#include <array>
#include <cmath>

#include "common.h"

void PowerMeterApp::compute_task()
{
    ESP_LOGI(TAG, "compute_task: started");

    // These will contain the centered AC_V and AC_I values
    auto v_array = new float[CHUNK_SIZE];
    auto i_array = new float[CHUNK_SIZE];
    auto aligned_v_array = new float[CHUNK_SIZE];

    // This is used to save the last 4 samples of the previous chunk since the current
    // goes ahead of voltage due to LM358 signal processing delay
    float prev_v_history[LM_COARS_SAMPLES_SHIFT] {};

    // Track exact wall-clock time for zero-drift energy accumulation
    int64_t last_energy_update_us = 0;

    while (!m_stop_tasks) {
        // Check if we have enough samples in BOTH buffers to do a full calculation
        if (m_v_ring_buffer.available() >= CHUNK_SIZE &&
            m_i_ring_buffer.available() >= CHUNK_SIZE) {

            // Pull the data out of the lock-free buffers
            m_v_ring_buffer.pop_burst(v_array, CHUNK_SIZE);
            m_i_ring_buffer.pop_burst(i_array, CHUNK_SIZE);

            const int64_t now_us = esp_timer_get_time();

            // Sacrifice the first chunk to perform initial DC bias learning
            if (std::isnan(m_v_dc_blocker.get_current_dc_bias())) {
                float v_max = v_array[0];
                float v_min = v_array[0];
                float i_max = i_array[0];
                float i_min = i_array[0];
                double v_sum = 0;
                double i_sum = 0;
                for (int i = 0; i < CHUNK_SIZE; i++) {
                    v_sum += v_array[i];
                    i_sum += i_array[i];
                    if (v_max < v_array[i])
                        v_max = v_array[i];
                    if (v_min > v_array[i])
                        v_min = v_array[i];
                    if (i_max < i_array[i])
                        i_max = i_array[i];
                    if (i_min > i_array[i])
                        i_min = i_array[i];
                }
                float v_mean = v_sum / CHUNK_SIZE;
                float i_mean = i_sum / CHUNK_SIZE;

                m_v_dc_blocker.set_current_dc_bias(v_mean);
                ESP_LOGI(TAG, "ADC_V: DC bias %0.1f, max %0.1f, min %0.1f", v_mean, v_max, v_min);

                m_i_dc_blocker.set_current_dc_bias(i_mean);
                ESP_LOGI(TAG, "ADC_I: DC bias %0.1f, max %0.1f, min %0.1f", i_mean, i_max, i_min);

                // Save 4 last centered samples for the next chunk
                for (int i = 0; i < LM_COARS_SAMPLES_SHIFT; i++) {
                    prev_v_history[i] = m_v_dc_blocker.process(v_array[CHUNK_SIZE - LM_COARS_SAMPLES_SHIFT + i]);
                }
                last_energy_update_us = now_us;
                continue;
            }

            // --- SURGICAL RF SPIKE FILTER & OVERVOLTAGE PASS-THROUGH ---
            // Instead of dropping the entire 800-sample chunk (which loses 200ms of energy metering
            // and blocks >270V overvoltage readings), we check how many samples exceed SPIKE_THRESHOLD_ADC:
            // - 1..5 samples (< 1.25 ms): It is a fast RF/switching needle -> despike in-place without corrupting DCBlocker.
            // - > 5 samples: It is a genuine high-voltage AC sine wave (e.g. 280V-400V neutral fault) -> process normally!

            const float SPIKE_THRESHOLD_ADC = (MAX_EXPECTED_V_RMS * M_SQRT2) / m_settings.v_coef;
            const float current_v_bias = m_v_dc_blocker.get_current_dc_bias();

            int spike_samples_count = 0;
            for (int i = 0; i < CHUNK_SIZE; i++) {
                if (std::abs(v_array[i] - current_v_bias) > SPIKE_THRESHOLD_ADC) {
                    spike_samples_count++;
                }
            }

            const bool is_short_rf_transient = (spike_samples_count > 0 && spike_samples_count <= 5);
            if (is_short_rf_transient) {
                ESP_LOGW(TAG, "Short RF transient (%d samples) despiked in-place.", spike_samples_count);
            }

            for (int i = 0; i < CHUNK_SIZE; i++) {
                if (is_short_rf_transient && std::abs(v_array[i] - current_v_bias) > SPIKE_THRESHOLD_ADC) {
                    // Replace the corrupted impulse sample with the last clean centered sample
                    // without feeding the spike into m_v_dc_blocker
                    v_array[i] = (i > 0) ? v_array[i - 1] : prev_v_history[LM_COARS_SAMPLES_SHIFT - 1];
                }
                else {
                    v_array[i] = m_v_dc_blocker.process(v_array[i]);
                }
                i_array[i] = m_i_dc_blocker.process(i_array[i]);
            }
            // -----------------------------------------------------------

            // Get first 4 samples from the previous chunk
            memcpy(&aligned_v_array[0], &prev_v_history[0], LM_COARS_SAMPLES_SHIFT * sizeof(float));
            // Fill the rest samples from the new chunk
            memcpy(&aligned_v_array[LM_COARS_SAMPLES_SHIFT], &v_array[0], (CHUNK_SIZE - LM_COARS_SAMPLES_SHIFT) * sizeof(float));
            // Save the last 4 samples of the new chunk for the next reading
            memcpy(&prev_v_history[0], &v_array[CHUNK_SIZE - LM_COARS_SAMPLES_SHIFT], LM_COARS_SAMPLES_SHIFT * sizeof(float));

            // Read the dynamically measured ADC sampling rate per channel
            const float current_fs_hz = m_actual_sample_freq_hz.load(std::memory_order_relaxed);

            // --- CALCULATE THE GRID FREQUENCY (15 Hz .. 65 Hz) ---

            const float HYSTERESIS_THRESHOLD = DeviceSettings::V_HYSTERESIS / m_settings.v_coef;

            // Broad hardware bounds: 15.0 Hz (~266.7 samples) to 65.0 Hz (~61.5 samples)
            const float MIN_FREQ_HZ = 15.0f;
            const float MAX_FREQ_HZ = 65.0f;
            const float MIN_VALID_PERIOD = current_fs_hz / MAX_FREQ_HZ; // ~61.5 samples
            const float MAX_VALID_PERIOD = current_fs_hz / MIN_FREQ_HZ; // ~266.7 samples

            // Deadtime lockout: ~75% of the minimum period at 65 Hz (~45 samples / 11.2 ms)
            const int DEADTIME_SAMPLES = static_cast<int>(MIN_VALID_PERIOD * 0.75f);

            bool is_positive_half = (aligned_v_array[0] > 0.0f);
            int last_rising_edge_index = -100;
            int last_negative_index = (aligned_v_array[0] <= 0.0f) ? 0 : -1;

            float prev_crossing_index = NAN;
            int first_crossing_sample = -1;
            int last_crossing_sample = -1;

            // At up to 65 Hz in an 800-sample chunk, there can be at most 13 cycles
            std::array<float, 16> cycle_periods {};
            size_t cycle_periods_count = 0;

            for (int i = 1; i < CHUNK_SIZE; i++) {
                // Track the latest sample <= 0 before the positive hysteresis threshold is crossed
                if (!is_positive_half && aligned_v_array[i] <= 0.0f) {
                    last_negative_index = i;
                }

                // Signal must securely exceed the positive threshold to register a cycle
                if (!is_positive_half && aligned_v_array[i] > HYSTERESIS_THRESHOLD) {
                    is_positive_half = true;

                    if (i - last_rising_edge_index > DEADTIME_SAMPLES && last_negative_index >= 0) {
                        last_rising_edge_index = i;

                        // Advance from last_negative_index to find the exact sample pair straddling 0V
                        int z = last_negative_index;
                        while (z + 1 < i && aligned_v_array[z + 1] <= 0.0f) {
                            z++;
                        }

                        // Calculate the exact sub-sample fractional index of the crossing
                        const float denom = aligned_v_array[z + 1] - aligned_v_array[z];
                        const float fraction = (denom > 1e-3f) ? (-aligned_v_array[z] / denom) : 0.0f;
                        const float exact_index = static_cast<float>(z) + fraction;

                        if (first_crossing_sample < 0) {
                            first_crossing_sample = z;
                        }
                        last_crossing_sample = z;

                        if (!std::isnan(prev_crossing_index)) {
                            const float period_samples = exact_index - prev_crossing_index;

                            // Stage 1: Validate broad physical bounds [15 Hz .. 65 Hz]
                            if (period_samples >= MIN_VALID_PERIOD &&
                                period_samples <= MAX_VALID_PERIOD &&
                                cycle_periods_count < cycle_periods.size()) {
                                cycle_periods[cycle_periods_count++] = period_samples;
                            }
                        }
                        prev_crossing_index = exact_index;
                    }
                }
                // Signal must securely fall below the negative threshold to arm for the next cycle
                else if (is_positive_half && aligned_v_array[i] < -HYSTERESIS_THRESHOLD) {
                    is_positive_half = false;
                }
            }

            if (cycle_periods_count > 0) {
                // Stage 2: Find median period in this chunk and average only inliers (within +/- 1.5% of median)
                std::array<float, 16> sorted = cycle_periods;
                std::sort(sorted.begin(), sorted.begin() + cycle_periods_count);
                const float median_period = sorted[cycle_periods_count / 2];
                const float tolerance = median_period * 0.015f;

                double inlier_sum = 0.0;
                int inlier_count = 0;

                for (size_t k = 0; k < cycle_periods_count; ++k) {
                    if (std::abs(cycle_periods[k] - median_period) <= tolerance) {
                        inlier_sum += cycle_periods[k];
                        inlier_count++;
                    }
                }

                if (inlier_count > 0) {
                    const double avg_period_samples = inlier_sum / inlier_count;
                    m_frequency = static_cast<float>(current_fs_hz / avg_period_samples);
                }
            }
            else {
                m_frequency = NAN;
            }
            // -----------------------------------------------------

            // --- DATA IS NOW READY FOR ESP-DSP ---
            // Use cycle-synchronized window [first_crossing_sample .. last_crossing_sample)
            // if we have at least 1 valid full cycle, so V_rms, I_rms, and Power don't wobble off-50Hz.
            int dsp_start_idx = 0;
            int dsp_sample_count = CHUNK_SIZE;

            if (cycle_periods_count >= 1 &&
                first_crossing_sample >= 0 &&
                last_crossing_sample > first_crossing_sample) {
                dsp_start_idx = first_crossing_sample;
                dsp_sample_count = last_crossing_sample - first_crossing_sample;
            }

            float v_sq_sum = 0;
            float i_sq_sum = 0;

            dsps_dotprod_f32(&aligned_v_array[dsp_start_idx], &aligned_v_array[dsp_start_idx], &v_sq_sum, dsp_sample_count);
            dsps_dotprod_f32(&i_array[dsp_start_idx], &i_array[dsp_start_idx], &i_sq_sum, dsp_sample_count);

            m_v_rms = sqrt(v_sq_sum / dsp_sample_count) * m_settings.v_coef;
            m_i_rms = sqrt(i_sq_sum / dsp_sample_count) * m_settings.i_coef;

            if (m_v_rms < m_settings.v_noise_floor) {
                m_v_rms = 0.0f;
                m_frequency = NAN;
            }
            if (m_i_rms < m_settings.i_noise_floor) {
                m_i_rms = 0.0f;
            }

            // Calculate exact wall-clock elapsed time since the previous chunk
            double dt_seconds = static_cast<double>(CHUNK_SIZE) / static_cast<double>(current_fs_hz);
            if (last_energy_update_us > 0) {
                const int64_t elapsed_us = now_us - last_energy_update_us;
                // Use hardware wall-clock delta if within reasonable bounds (up to 2.0s to bridge any brief hiccup)
                if (elapsed_us > 0 && elapsed_us < 2000000) {
                    dt_seconds = static_cast<double>(elapsed_us) * 1e-6;
                }
            }
            last_energy_update_us = now_us;

            if (m_v_rms == 0.0f || m_i_rms == 0.0f) {
                m_apparent_power = 0.0f;
                m_real_power = 0.0f;
                //m_cos_phi = NAN;
            }
            else {
                m_apparent_power = m_v_rms * m_i_rms;               // VA

                float real_pwr = 0;
                dsps_dotprod_f32(&aligned_v_array[dsp_start_idx], &i_array[dsp_start_idx], &real_pwr, dsp_sample_count);
                real_pwr /= dsp_sample_count;
                real_pwr *= m_settings.v_coef * m_settings.i_coef;  // Watts
                m_real_power = real_pwr;

                // Integrate energy using true wall-clock elapsed time (zero lost chunks!)
                m_acc_energy += m_real_power * dt_seconds;          // Yields watt-seconds
            }

            InterfaceTaskMessage qmsg {
                .type = MessageType::ResultMessage,
                .result = {
                    .v_rms = m_v_rms,
                    .i_rms = m_i_rms,
                    .apparent_power = m_apparent_power,
                    .real_power = m_real_power,
                    .energy = m_acc_energy / 3600.0,        // Convert to watt-hours
                    //.cos_phi = m_cos_phi,
                    .frequency = m_frequency,
                    //.vi_shift = m_vi_sample_shift,
                },
            };
            if (xQueueSend(m_interface_queue, &qmsg, 0) != pdPASS) {
                ESP_LOGW(TAG, "Interface queue is full, dropping ResultMessage");
            }
        }

        // Wait notification from reader_task
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
    }

    delete[] aligned_v_array;
    delete[] i_array;
    delete[] v_array;

    ESP_LOGI(TAG, "compute_task: finished");
    vTaskDelete(m_compute_task = nullptr);
}
