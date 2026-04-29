#include "PowerMeterApp.h"

void PowerMeterApp::setup_reader(bool disposing)
{
    if (disposing) {
        adc_continuous_stop(m_adc);
        adc_continuous_deinit(m_adc), m_adc = nullptr;
        adc_cali_delete_scheme_curve_fitting(m_adc_v_cali), m_adc_v_cali = nullptr;
        adc_cali_delete_scheme_curve_fitting(m_adc_i_cali), m_adc_i_cali = nullptr;
    }
    else {
        ESP_LOGI(TAG, "reader setup");
        adc_unit_t adc_unit_id = {};
        ESP_ERROR_CHECK(adc_continuous_io_to_channel(CONFIG_PIN_ADC_V, &adc_unit_id, &m_adc_v_channel_id));
        if (adc_unit_id != ADC_UNIT_1)
            ESP_ERROR_CHECK(ESP_ERR_NOT_ALLOWED);
        ESP_LOGI(TAG, "ADC_V: working with ADC_UNIT_%d, ADC_CHANNEL_%0d", adc_unit_id + 1, m_adc_v_channel_id);

        adc_unit_t adc_i_unit_id = {};
        ESP_ERROR_CHECK(adc_continuous_io_to_channel(CONFIG_PIN_ADC_I, &adc_i_unit_id, &m_adc_i_channel_id));
        if (adc_i_unit_id != ADC_UNIT_1)
            ESP_ERROR_CHECK(ESP_ERR_NOT_ALLOWED);
        ESP_LOGI(TAG, "ADC_I: working with ADC_UNIT_%d, ADC_CHANNEL_%0d", adc_i_unit_id + 1, m_adc_i_channel_id);

        adc_continuous_handle_cfg_t alloc_cfg = {
            .max_store_buf_size = FRAME_BUF_SIZE * 2,
            .conv_frame_size = FRAME_BUF_SIZE,
        };
        ESP_ERROR_CHECK(adc_continuous_new_handle(&alloc_cfg, &m_adc));

        // Configure the pattern array for two channels
        adc_digi_pattern_config_t adc_pattern[] = {
            {   .atten = ADC_ATTEN_DB_12,       // 0 to ~3.1V range
                .channel = (uint8_t) m_adc_v_channel_id,
                .unit = ADC_UNIT_1,
                .bit_width = ADC_BITWIDTH_12,
            },
            {   .atten = ADC_ATTEN_DB_12,       // 0 to ~3.1V range
                .channel = (uint8_t) m_adc_i_channel_id,
                .unit = ADC_UNIT_1,
                .bit_width = ADC_BITWIDTH_12,
            }
        };
        adc_continuous_config_t config = {
            .pattern_num = countof(adc_pattern),
            .adc_pattern = adc_pattern,
            .sample_freq_hz = SAMPLE_FREQ_HZ,
            .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        };
        ESP_ERROR_CHECK(adc_continuous_config(m_adc, &config));

        adc_cali_scheme_ver_t scheme_mask = {};
        ESP_ERROR_CHECK(adc_cali_check_scheme(&scheme_mask));
        if (!(scheme_mask & ADC_CALI_SCHEME_VER_CURVE_FITTING))
            ESP_ERROR_CHECK(ESP_ERR_NOT_SUPPORTED);

        adc_cali_curve_fitting_config_t adc_v_cali_config = {
            .unit_id = ADC_UNIT_1,
            .chan = m_adc_v_channel_id,
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_12,
        };
        ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&adc_v_cali_config, &m_adc_v_cali));

        adc_cali_curve_fitting_config_t adc_i_cali_config = {
            .unit_id = ADC_UNIT_1,
            .chan = m_adc_i_channel_id,
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_12,
        };
        ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&adc_i_cali_config, &m_adc_i_cali));

        ESP_ERROR_CHECK(adc_continuous_start(m_adc));
        ESP_LOGI(TAG, "ADC DMA Initialized");
    }
}

void PowerMeterApp::reader_task()
{
    ESP_LOGI(TAG, "reader_task: started");
    auto parsed_buf = new adc_continuous_data_t[SAMPLES_TO_READ];
    uint32_t samples_count = 0;

    setup_reader();

    while (!m_stop_tasks) {
        // Block and wait for a chunk of data from DMA
        esp_err_t ret = adc_continuous_read_parse(m_adc, parsed_buf, SAMPLES_TO_READ, &samples_count, portMAX_DELAY);
        if (ret == ESP_OK) {
            // Parse the raw ESP32-S3 data into clean values
            for (int i = 0; i < samples_count; ++i) {
                if (!parsed_buf[i].valid) {
                    m_v_ring_buffer.reset();
                    m_i_ring_buffer.reset();
                    //m_error_count++;
                    break;
                }
                if (parsed_buf[i].channel == m_adc_v_channel_id) {
                    int calibrated_mv = 0;
                    adc_cali_raw_to_voltage(m_adc_v_cali, parsed_buf[i].raw_data, &calibrated_mv);
                    m_v_ring_buffer.push(calibrated_mv);
                }
                else if (parsed_buf[i].channel == m_adc_i_channel_id) {
                    int calibrated_mv = 0;
                    adc_cali_raw_to_voltage(m_adc_i_cali, parsed_buf[i].raw_data, &calibrated_mv);
                    m_i_ring_buffer.push(calibrated_mv);
                }
            }
            if (m_compute_task)
                xTaskNotifyGive(m_compute_task);
        }
        else if (ret == ESP_ERR_TIMEOUT) {
            //m_error_count++;
            continue;
        }
        else {
            //m_error_count++;
        }
    }

    setup_reader(kDisposing);

    delete[] parsed_buf;
    ESP_LOGI(TAG, "reader_task: finished");
    vTaskDelete(nullptr);
}
