#include "PowerMeterApp.h"

const bool kDisposing = true;

PowerMeterApp::PowerMeterApp()
    : m_encoder(CONFIG_PIN_ENCODER_S1, CONFIG_PIN_ENCODER_S2, [this](bool is_ccw) { on_encoder_rotate(is_ccw); })
    , m_encoder_key(CONFIG_PIN_ENCODER_KEY, [this] (bool is_long) { on_encoder_click(is_long); })
{
    ESP_LOGI(TAG, "Running on core #%d", xPortGetCoreID());

    setup_reader();
    setup_interface();
    setup_telemetry();
    setup_console_input();

    setup_tasks();
}

PowerMeterApp::~PowerMeterApp()
{
    stop_tasks();
    vTaskDelay(pdMS_TO_TICKS(500));

    setup_console_input(kDisposing);
    setup_telemetry(kDisposing);
    setup_reader(kDisposing);
    setup_interface(kDisposing);

    ESP_LOGI(TAG, "App destroyed");
}

void PowerMeterApp::setup_tasks()
{
    xTaskCreatePinnedToCore(
        member_cast<TaskFunction_t>(&PowerMeterApp::interface_task),
        "interface_task",       // A descriptive name for debugging
        4096,                   // Stack size
        this,                   // Parameter passed to the task (pointer to object)
        5,                      // Task priority (0 is lowest, configMAX_PRIORITIES-1 is highest)
        nullptr,                // No task handle needed
        xPortGetCoreID()        // Pin to the same core
    );
    xTaskCreatePinnedToCore(
        member_cast<TaskFunction_t>(&PowerMeterApp::compute_task),
        "compute_task",         // A descriptive name for debugging
        4096,                   // Stack size
        this,                   // Parameter passed to the task (pointer to object)
        15,                     // Task priority (0 is lowest, configMAX_PRIORITIES-1 is highest)
        &m_compute_task,        // Task handle for notifications
        xPortGetCoreID()        // Pin to the same core
    );
    xTaskCreatePinnedToCore(
        member_cast<TaskFunction_t>(&PowerMeterApp::reader_task),
        "reader_task",          // A descriptive name for debugging
        4096,                   // Stack size
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

void PowerMeterApp::stop_tasks()
{
    m_stop_tasks = true;
}
