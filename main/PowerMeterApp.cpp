#include "PowerMeterApp.h"

#include <cmath>

#include <i2cdev.h>
#include <esp_log.h>
#include <esp_adc/adc_cali_scheme.h>

#include "common.h"

PowerMeterApp::PowerMeterApp()
    : m_button(CONFIG_PIN_ENCODER_KEY, [this] { m_stop_tasks = true; })
{
    ESP_LOGI(TAG, "Running on core #%d", xPortGetCoreID());
    m_display_queue = xQueueCreate(10, sizeof(ComputeResultMessage));

    ESP_ERROR_CHECK(i2cdev_init());
    setup_adc();
    setup_tasks();
}

PowerMeterApp::~PowerMeterApp()
{
    m_stop_tasks = true;
    vTaskDelay(pdMS_TO_TICKS(100));

    adc_continuous_deinit(m_adc_handle);
    adc_cali_delete_scheme_curve_fitting(m_adc_v_cali_handle);
    adc_cali_delete_scheme_curve_fitting(m_adc_i_cali_handle);
    i2cdev_done();
    vQueueDelete(m_display_queue);

    ESP_LOGI(TAG, "Running on core #%d", xPortGetCoreID());
}

void PowerMeterApp::setup_adc()
{
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
        .max_store_buf_size = FRAME_BUF_SIZE * 4,
        .conv_frame_size = FRAME_BUF_SIZE,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&alloc_cfg, &m_adc_handle));

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

    ESP_ERROR_CHECK(adc_continuous_config(m_adc_handle, &config));

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
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&adc_v_cali_config, &m_adc_v_cali_handle));

    adc_cali_curve_fitting_config_t adc_i_cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = m_adc_i_channel_id,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&adc_i_cali_config, &m_adc_i_cali_handle));

    ESP_LOGI(TAG, "ADC DMA Initialized");
}

void PowerMeterApp::setup_tasks()
{
    xTaskCreatePinnedToCore(
        member_cast<TaskFunction_t>(&PowerMeterApp::display_task),
        "display_task",         // A descriptive name for debugging
        4096,                   // Stack size
        this,                   // Parameter passed to the task (pointer to object)
        5,                      // Task priority (0 is lowest, configMAX_PRIORITIES-1 is highest)
        nullptr,                // No task handle needed
        xPortGetCoreID()        // Pin to the same core
    );
    xTaskCreatePinnedToCore(
        member_cast<TaskFunction_t>(&PowerMeterApp::compute_task),
        "compute_task",         // A descriptive name for debugging
        16384,                  // Stack size
        this,                   // Parameter passed to the task (pointer to object)
        9,                      // Task priority (0 is lowest, configMAX_PRIORITIES-1 is highest)
        nullptr,                // No task handle needed
        xPortGetCoreID()        // Pin to the same core
    );
    xTaskCreatePinnedToCore(
        member_cast<TaskFunction_t>(&PowerMeterApp::reader_task),
        "reader_task",          // A descriptive name for debugging
        8192,                   // Stack size (4K for parsed_buf, 4K for the code)
        this,                   // Parameter passed to the task (pointer to object)
        10,                     // Task priority (0 is lowest, configMAX_PRIORITIES-1 is highest)
        nullptr,                // No task handle needed
        xPortGetCoreID()        // Pin to the same core
    );
}

void PowerMeterApp::reader_task()
{
    ESP_LOGI(TAG, "reader_task: started");
    adc_continuous_data_t parsed_buf[SAMPLES_TO_READ] = {};
    uint32_t samples_count = 0;

    ESP_ERROR_CHECK(adc_continuous_start(m_adc_handle));

    while (!m_stop_tasks) {
        // Block and wait for a chunk of data from DMA
        esp_err_t ret = adc_continuous_read_parse(m_adc_handle, parsed_buf, SAMPLES_TO_READ, &samples_count, portMAX_DELAY);
        if (ret == ESP_OK) {
            // Parse the raw ESP32-S3 data into clean values
            for (int i = 0; i < samples_count; ++i) {
                if (!parsed_buf[i].valid)
                    continue;
                if (parsed_buf[i].channel == m_adc_v_channel_id) {
                    int calibrated_mv = 0;
                    adc_cali_raw_to_voltage(m_adc_v_cali_handle, parsed_buf[i].raw_data, &calibrated_mv);
                    m_v_ring_buffer.push(calibrated_mv);
                }
                else if (parsed_buf[i].channel == m_adc_i_channel_id) {
                    int calibrated_mv = 0;
                    adc_cali_raw_to_voltage(m_adc_i_cali_handle, parsed_buf[i].raw_data, &calibrated_mv);
                    m_i_ring_buffer.push(calibrated_mv);
                }
            }
        }
        else {
            //++m_err_count;
        }
    }

    adc_continuous_stop(m_adc_handle);
    ESP_LOGI(TAG, "reader_task: finished");
    vTaskDelete(nullptr);
}
