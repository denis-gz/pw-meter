#include "PowerMeterApp.h"

#include <esp_log.h>
#include <esp_dsp.h>
#include <esp_timer.h>
#include <algorithm>
#include <array>
#include <cmath>

#include "common.h"

// V/I alignment: V is delayed by whole samples only (values stay intact),
// the remaining fraction is applied to I using linear interpolation
constexpr int V_SHIFT_SAMPLES = static_cast<int>(LM_SAMPLES_SHIFT) + 1;
constexpr float I_SHIFT_FRAC = V_SHIFT_SAMPLES - LM_SAMPLES_SHIFT;

// Grid frequency = cycles / span of rising zero crossings over this window: error falls as 1/T
constexpr double FREQ_WINDOW_SAMPLES = CHANNEL_FREQ_HZ * 1.0;

// Rising zero crossings on the absolute sample timeline, oldest dropped past the window span
class CrossingWindow
{
private:
    std::array<double, 128> times {};
    size_t head = 0;
    size_t count = 0;

    void drop_oldest() {
        head = (head + 1) % times.size();
        count--;
    }

public:
    size_t size() const { return count; }
    double oldest() const { return times[head]; }
    double newest() const { return times[(head + count - 1) % times.size()]; }
    double mean_period() const { return (newest() - oldest()) / (count - 1); }

    void clear() {
        count = 0;
    }

    void push(double t) {
        if (count == times.size())
            drop_oldest();
        times[(head + count) % times.size()] = t;
        count++;
        while (count > 2 && t - oldest() > FREQ_WINDOW_SAMPLES)
            drop_oldest();
    }
};

void PowerMeterApp::compute_task()
{
    ESP_LOGI(TAG, "compute_task: started");

    // These will contain the centered AC_V and AC_I values
    auto v_array = new float[CHUNK_SIZE];
    auto i_array = new float[CHUNK_SIZE];
    auto aligned_v_array = new float[CHUNK_SIZE];
    auto aligned_i_array = new float[CHUNK_SIZE];

    // Tails of the previous chunk, since the current goes ahead of voltage
    // due to LM358 signal processing delay
    float prev_v_history[V_SHIFT_SAMPLES] {};
    float prev_i_last = 0;

    // Track exact wall-clock time for zero-drift energy accumulation
    int64_t last_energy_update_us = 0;

    // Rising zero-crossing detector, kept across chunks on an absolute sample timeline
    int64_t chunk_base = 0;
    bool zc_positive_half = false;
    float zc_prev_v = 0.0f;
    double zc_candidate = NAN;
    int64_t zc_last_rising = INT64_MIN / 2;
    CrossingWindow zc_window;

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

                // Save last centered samples for the next chunk
                for (int i = 0; i < V_SHIFT_SAMPLES; i++) {
                    prev_v_history[i] = m_v_dc_blocker.process(v_array[CHUNK_SIZE - V_SHIFT_SAMPLES + i]);
                }
                prev_i_last = m_i_dc_blocker.process(i_array[CHUNK_SIZE - 1]);
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
                    v_array[i] = (i > 0) ? v_array[i - 1] : prev_v_history[V_SHIFT_SAMPLES - 1];
                }
                else {
                    v_array[i] = m_v_dc_blocker.process(v_array[i]);
                }
                i_array[i] = m_i_dc_blocker.process(i_array[i]);
            }
            // -----------------------------------------------------------

            // Get first samples from the previous chunk
            memcpy(&aligned_v_array[0], &prev_v_history[0], V_SHIFT_SAMPLES * sizeof(float));
            // Fill the rest samples from the new chunk
            memcpy(&aligned_v_array[V_SHIFT_SAMPLES], &v_array[0], (CHUNK_SIZE - V_SHIFT_SAMPLES) * sizeof(float));
            // Save the last samples of the new chunk for the next reading
            memcpy(&prev_v_history[0], &v_array[CHUNK_SIZE - V_SHIFT_SAMPLES], V_SHIFT_SAMPLES * sizeof(float));

            // Delay I by the fractional remainder: i(n - frac) between i[n] and i[n - 1]
            aligned_i_array[0] = (1.0f - I_SHIFT_FRAC) * i_array[0] + I_SHIFT_FRAC * prev_i_last;
            for (int n = 1; n < CHUNK_SIZE; n++) {
                aligned_i_array[n] = (1.0f - I_SHIFT_FRAC) * i_array[n] + I_SHIFT_FRAC * i_array[n - 1];
            }
            prev_i_last = i_array[CHUNK_SIZE - 1];

            // --- CALCULATE THE GRID FREQUENCY (15 Hz .. 65 Hz) ---

            const float HYSTERESIS_THRESHOLD = DeviceSettings::V_HYSTERESIS / m_settings.v_coef;

            // Broad hardware bounds: 15.0 Hz (~266.7 samples) to 65.0 Hz (~61.5 samples)
            const float MIN_VALID_PERIOD = CHANNEL_FREQ_HZ / DeviceSettings::MAX_FREQ_HZ; // ~61.5 samples
            const float MAX_VALID_PERIOD = CHANNEL_FREQ_HZ / DeviceSettings::MIN_FREQ_HZ; // ~266.7 samples

            // Deadtime lockout: ~75% of the minimum period at 65 Hz (~45 samples / 11.2 ms)
            const int DEADTIME_SAMPLES = static_cast<int>(MIN_VALID_PERIOD * 0.75f);

            int first_crossing_sample = -1;
            int last_crossing_sample = -1;
            int chunk_crossings = 0;

            // A missed or spurious crossing breaks the cycle count: restart the window
            auto register_crossing = [&](double t) {
                if (zc_window.size() > 0) {
                    const double period = t - zc_window.newest();
                    bool plausible = period >= MIN_VALID_PERIOD && period <= MAX_VALID_PERIOD;
                    if (plausible && zc_window.size() >= 2) {
                        const double expected = zc_window.mean_period();
                        plausible = std::abs(period - expected) <= expected * 0.015;
                    }
                    if (!plausible)
                        zc_window.clear();
                }
                zc_window.push(t);
            };

            for (int i = 0; i < CHUNK_SIZE; i++) {
                const float v = aligned_v_array[i];
                const int64_t n = chunk_base + i;

                if (!zc_positive_half) {
                    // Latest rising crossing, interpolated between the straddling samples
                    if (zc_prev_v <= 0.0f && v > 0.0f) {
                        const float denom = v - zc_prev_v;
                        zc_candidate = static_cast<double>(n - 1) + ((denom > 1e-3f) ? -zc_prev_v / denom : 0.0f);
                    }
                    // Signal must securely exceed the positive threshold to register a cycle
                    if (v > HYSTERESIS_THRESHOLD) {
                        zc_positive_half = true;
                        if (n - zc_last_rising > DEADTIME_SAMPLES && !std::isnan(zc_candidate)) {
                            zc_last_rising = n;
                            register_crossing(zc_candidate);

                            const int z = static_cast<int>(std::floor(zc_candidate - chunk_base));
                            if (z >= 0) {
                                if (first_crossing_sample < 0)
                                    first_crossing_sample = z;
                                last_crossing_sample = z;
                                chunk_crossings++;
                            }
                        }
                        zc_candidate = NAN;
                    }
                }
                // Signal must securely fall below the negative threshold to arm for the next cycle
                else if (v < -HYSTERESIS_THRESHOLD) {
                    zc_positive_half = false;
                }
                zc_prev_v = v;
            }
            chunk_base += CHUNK_SIZE;

            if (chunk_crossings == 0)
                zc_window.clear();

            m_frequency = (zc_window.size() >= 2)
                ? static_cast<float>(CHANNEL_FREQ_HZ / zc_window.mean_period())
                : NAN;
            // -----------------------------------------------------

            // --- DATA IS NOW READY FOR ESP-DSP ---
            // Use cycle-synchronized window [first_crossing_sample .. last_crossing_sample)
            // if we have at least 1 valid full cycle, so V_rms, I_rms, and Power don't wobble off-50Hz.
            int dsp_start_idx = 0;
            int dsp_sample_count = CHUNK_SIZE;

            if (chunk_crossings >= 2 &&
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
                zc_window.clear();
            }
            if (m_i_rms < m_settings.i_noise_floor) {
                m_i_rms = 0.0f;
            }

            // Calculate exact wall-clock elapsed time since the previous chunk
            double dt_seconds = static_cast<double>(CHUNK_SIZE) / CHANNEL_FREQ_HZ;
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
                dsps_dotprod_f32(&aligned_v_array[dsp_start_idx], &aligned_i_array[dsp_start_idx], &real_pwr, dsp_sample_count);
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

    delete[] aligned_i_array;
    delete[] aligned_v_array;
    delete[] i_array;
    delete[] v_array;

    ESP_LOGI(TAG, "compute_task: finished");
    // Reader notifies this task, so it must be gone before the handle becomes invalid
    xEventGroupWaitBits(m_task_exit_group, READER_EXIT_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    m_compute_task = nullptr;
    xEventGroupSetBits(m_task_exit_group, COMPUTE_EXIT_BIT);
    vTaskDelete(nullptr);
}
