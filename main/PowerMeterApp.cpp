#include "PowerMeterApp.h"

const bool kDisposing = true;

PowerMeterApp::PowerMeterApp()
    : m_encoder(CONFIG_PIN_ENCODER_S1, CONFIG_PIN_ENCODER_S2, [this](bool is_ccw) { on_rotate(is_ccw); })
    , m_encoder_key(CONFIG_PIN_ENCODER_KEY, [this] (bool is_long) { on_click(is_long); })
{
    ESP_LOGI(TAG, "Running on core #%d", xPortGetCoreID());

    setup_reader();
    setup_input();
    setup_display();

    setup_tasks();
}

PowerMeterApp::~PowerMeterApp()
{
    m_stop_tasks = true;
    vTaskDelay(pdMS_TO_TICKS(500));

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
