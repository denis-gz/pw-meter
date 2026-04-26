#include "PowerMeterApp.h"

const bool kDisposing = true;

PowerMeterApp::PowerMeterApp()
    : m_encoder(CONFIG_PIN_ENCODER_S1, CONFIG_PIN_ENCODER_S2, [this](bool is_ccw) { on_rotate(is_ccw); })
    , m_encoder_key(CONFIG_PIN_ENCODER_KEY, [this] (bool is_long) { on_click(is_long); })
{
    ESP_LOGI(TAG, "Running on core #%d", xPortGetCoreID());

    gpio_config_t gpio_conf = {
        .pin_bit_mask = 1ULL << 10,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&gpio_conf));

    setup_reader();
    setup_input();
    setup_display();
    setup_telemetry();

    setup_tasks();
}

PowerMeterApp::~PowerMeterApp()
{
    m_stop_tasks = true;
    vTaskDelay(pdMS_TO_TICKS(500));

    setup_telemetry(kDisposing);
    setup_input(kDisposing);
    setup_reader(kDisposing);
    setup_display(kDisposing);

    ESP_LOGI(TAG, "App destroyed");
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
        15,                     // Task priority (0 is lowest, configMAX_PRIORITIES-1 is highest)
        nullptr,                // No task handle needed
        xPortGetCoreID()        // Pin to the same core
    );
    xTaskCreatePinnedToCore(
        member_cast<TaskFunction_t>(&PowerMeterApp::reader_task),
        "reader_task",          // A descriptive name for debugging
        8192,                   // Stack size (4K for parsed_buf, 4K for the code)
        this,                   // Parameter passed to the task (pointer to object)
        20,                     // Task priority (0 is lowest, configMAX_PRIORITIES-1 is highest)
        nullptr,                // No task handle needed
        xPortGetCoreID()        // Pin to the same core
    );
    xTaskCreatePinnedToCore(
        member_cast<TaskFunction_t>(&PowerMeterApp::telemetry_task),
        "telemetry_task",       // A descriptive name for debugging
        4096,                   // Stack size
        this,                   // Parameter passed to the task (pointer to object)
        5,                      // Task priority (0 is lowest, configMAX_PRIORITIES-1 is highest)
        nullptr,                // No task handle needed
        xPortGetCoreID()        // Pin to the same core
    );
}

void PowerMeterApp::post_log(display_line_t message)
{
    DisplayTaskMessage qmsg {
        .type = MessageType::LogMessage,
        .log { .message = message }
    };
    if (xQueueSend(m_display_queue, &qmsg, pdMS_TO_TICKS(100)) != pdPASS) {
        ESP_LOGI(TAG, "Warning: Display queue is full, dropping LogMessage\n");
    }
}
