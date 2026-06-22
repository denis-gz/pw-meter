#include "PowerMeterApp.h"

timeval PowerMeterApp::s_start_time {};

PowerMeterApp::PowerMeterApp()
    : m_encoder(CONFIG_PIN_ENCODER_A, CONFIG_PIN_ENCODER_B, [this](bool is_ccw) { on_encoder_rotate(is_ccw); })
    , m_encoder_key(CONFIG_PIN_ENCODER_KEY, [this] (bool is_long) { on_encoder_click(is_long); })
    , m_led(CONFIG_PIN_LED, true)
{
    SettingsManager::setup();
    SettingsManager::load(m_settings);

    setup_console_input();
    start_tasks();
}

PowerMeterApp::~PowerMeterApp()
{
    stop_tasks();
    setup_console_input(kDisposing);

    SettingsManager::setup(kDisposing);

    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "App destroyed");
}

void PowerMeterApp::start_tasks()
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
        member_cast<TaskFunction_t>(&PowerMeterApp::telemetry_task),
        "telemetry_task",       // A descriptive name for debugging
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
        member_cast<TaskFunction_t>(&PowerMeterApp::indicator_task),
        "indicator_task",       // A descriptive name for debugging
        2048,                   // Stack size
        this,                   // Parameter passed to the task (pointer to object)
        1,                      // Task priority (0 is lowest, configMAX_PRIORITIES-1 is highest)
        nullptr,                // No task handle needed
        xPortGetCoreID()        // Pin to the same core
    );
}

void PowerMeterApp::stop_tasks()
{
    m_stop_tasks = true;
}
