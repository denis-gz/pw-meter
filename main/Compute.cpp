#include "PowerMeterApp.h"

#include <esp_log.h>
#include <esp_dsp.h>

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

    while (!m_stop_tasks) {
        // Check if we have enough samples in BOTH buffers to do a full calculation
        if (m_v_ring_buffer.available() >= CHUNK_SIZE &&
            m_i_ring_buffer.available() >= CHUNK_SIZE) {

            // Pull the data out of the lock-free buffers
            m_v_ring_buffer.pop_burst(v_array, CHUNK_SIZE);
            m_i_ring_buffer.pop_burst(i_array, CHUNK_SIZE);

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

                // Save 4 last centered bytes for the next chunk
                for (int i = 0; i < LM_COARS_SAMPLES_SHIFT; i++) {
                    prev_v_history[i] = m_v_dc_blocker.process(v_array[CHUNK_SIZE - LM_COARS_SAMPLES_SHIFT + i]);
                }
                continue;
            }

            // --- THE RF CHUNK REJECTION ---

            bool rf_spike_detected = false;

            for (int i = 0; i < CHUNK_SIZE; i++) {
                // Strip the DC bias
                v_array[i] = m_v_dc_blocker.process(v_array[i]);
                i_array[i] = m_i_dc_blocker.process(i_array[i]);

                // If the centered value exceeds physical possibility, flag it
                if (v_array[i] > SPIKE_THRESHOLD_ADC || v_array[i] < -SPIKE_THRESHOLD_ADC) {
                    rf_spike_detected = true;
                }
            }

            if (rf_spike_detected) {
                // Update the history buffer so the *next* chunk has a clean seam
                for (int i = 0; i < LM_COARS_SAMPLES_SHIFT; i++) {
                    prev_v_history[i] = v_array[CHUNK_SIZE - LM_COARS_SAMPLES_SHIFT + i];
                }
                ESP_LOGW(TAG, "Hardware transient detected. Dropping chunk.");
                continue;
            }
            // ------------------------------

            // Get first 4 samples from the previous chunk
            memcpy(&aligned_v_array[0], &prev_v_history[0], LM_COARS_SAMPLES_SHIFT * sizeof(float));
            // Fill the rest samples from the new chunk
            memcpy(&aligned_v_array[LM_COARS_SAMPLES_SHIFT], &v_array[0], (CHUNK_SIZE - LM_COARS_SAMPLES_SHIFT) * sizeof(float));
            // Save the last 4 samples of the new chunk for the next reading
            memcpy(&prev_v_history[0], &v_array[CHUNK_SIZE - LM_COARS_SAMPLES_SHIFT], LM_COARS_SAMPLES_SHIFT * sizeof(float));

            // --- CALCULATE THE GRID FREQUENCY ---

            float v_first_crossing_index = NAN;
            float v_last_crossing_index = NAN;
            int v_cycle_count = 0;

            // Software Schmitt Trigger state
            bool is_positive_half = (aligned_v_array[0] > 0.0f);
            const float HYSTERESIS_THRESHOLD = 30.0f;

            // Track the index of the last valid cycle to enforce Deadtime
            int last_rising_edge_index = -100;

            // Iterate through the voltage array to find rising edges
            for (int i = 1; i < CHUNK_SIZE; i++) {

                // Signal must securely exceed the positive threshold to register a cycle
                if (!is_positive_half && aligned_v_array[i] > HYSTERESIS_THRESHOLD) {
                    is_positive_half = true;

                    // The 50-sample deadtime Lockout
                    if (i - last_rising_edge_index > 50) {
                        last_rising_edge_index = i;

                        // Trace backwards to find the exact zero-crossing points
                        int z = i;
                        while (z > 0 && aligned_v_array[z] > 0.0f) {
                            z--;
                        }

                        // Now, z is the index just before 0, and z+1 is just after 0.
                        // Calculate the exact sub-sample fractional index of the crossing
                        float fraction = (0.0f - aligned_v_array[z]) / (aligned_v_array[z+1] - aligned_v_array[z]);
                        float exact_index = (float)z + fraction;

                        if (v_cycle_count == 0) {
                            v_first_crossing_index = exact_index;
                        }
                        else {
                            v_last_crossing_index = exact_index;
                        }
                        v_cycle_count++;
                    }
                }
                // Signal must securely fall below the negative threshold to arm for the next cycle
                else if (is_positive_half && aligned_v_array[i] < -HYSTERESIS_THRESHOLD) {
                    is_positive_half = false;
                }
            }

            // We need at least 2 crossings to measure the time of 1 full cycle
            if (v_cycle_count >= 2) {
                const int total_full_cycles_measured = v_cycle_count - 1;
                const float delta_index = v_last_crossing_index - v_first_crossing_index;

                // Frequency = (Cycles / Samples) * Sampling_Rate
                m_frequency = (total_full_cycles_measured / delta_index) * CALIBRATED_FREQ_HZ;

                // If the calculated frequency jumps above 52 Hz, dump the chunk to the console!
                if (m_frequency > 52.0f) {
                    ESP_LOGE(TAG, "ANOMALY DETECTED! Frequency calculated as: %f Hz", m_frequency);
                    ESP_LOGE(TAG, "Cycle Count: %d, First Crossing: %f, Last Crossing: %f", v_cycle_count, v_first_crossing_index, v_last_crossing_index);

                    //printf("--- START CHUNK DUMP ---\n");
                    //for (int i = 0; i < CHUNK_SIZE; i++) {
                    //    printf("%f\n", aligned_v_array[i]);
                    //}
                    //printf("--- END CHUNK DUMP ---\n");
                }
            }
            // ------------------------------------

            // --- DATA IS NOW READY FOR ESP-DSP ---
            float v_sq_sum = 0;
            float i_sq_sum = 0;

            dsps_dotprod_f32(aligned_v_array, aligned_v_array, &v_sq_sum, CHUNK_SIZE);
            dsps_dotprod_f32(i_array, i_array, &i_sq_sum, CHUNK_SIZE);

            m_v_rms = sqrt(v_sq_sum / CHUNK_SIZE) * V_COEF;
            m_i_rms = sqrt(i_sq_sum / CHUNK_SIZE) * I_COEF;

            if (m_i_rms < m_i_noise_floor) {
                m_i_rms = 0.0f;
                m_apparent_power = 0.0f;
                m_real_power = 0.0f;
                //m_cos_phi = NAN;
            }
            else {
                m_apparent_power = m_v_rms * m_i_rms;       // VA

                float real_pwr = 0;
                dsps_dotprod_f32(aligned_v_array, i_array, &real_pwr, CHUNK_SIZE);
                real_pwr /= CHUNK_SIZE;
                real_pwr *= V_COEF * I_COEF;                // Watts
                m_real_power = real_pwr;

                // 800 samples at 4000Hz is exactly 0.2 seconds (0.00005556 hours)
                const double CHUNK_TIME = (static_cast<double>(CHUNK_SIZE) / CALIBRATED_FREQ_HZ);
                m_acc_energy_ws += m_real_power * CHUNK_TIME;
            }

            InterfaceTaskMessage qmsg {
                .type = MessageType::ResultMessage,
                .result = {
                    .v_rms = m_v_rms,
                    .i_rms = m_i_rms,
                    .apparent_power = m_apparent_power,
                    .real_power = m_real_power,
                    .energy = m_acc_energy_ws / 3600.0,
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
