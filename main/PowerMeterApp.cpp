#include "PowerMeterApp.h"

#include <esp_system.h>

PowerMeterApp::PowerMeterApp()
    : m_encoder(CONFIG_PIN_ENCODER_A, CONFIG_PIN_ENCODER_B, [this](bool is_ccw) { on_encoder_rotate(is_ccw); })
    , m_encoder_key(CONFIG_PIN_ENCODER_KEY, [this] (bool is_long) { on_encoder_click(is_long); })
    , m_led(CONFIG_PIN_LED, true)
{
    SettingsManager::setup();
    SettingsManager::load(m_settings);

    m_acc_energy = m_settings.acc_energy;

    setup_ipc();
    setup_console_input();
    start_tasks();
}

PowerMeterApp::~PowerMeterApp()
{
    stop_tasks();

    // Shared queues and the console loop are disposed only after all tasks have exited
    const EventBits_t bits = xEventGroupWaitBits(m_task_exit_group, ALL_TASKS_EXIT_BITS, pdFALSE, pdTRUE, pdMS_TO_TICKS(5000));
    if ((bits & ALL_TASKS_EXIT_BITS) != ALL_TASKS_EXIT_BITS) {
        ESP_LOGE(TAG, "Tasks did not stop in time (0x%02x), rebooting", (unsigned) bits);
        SettingsManager::save(KEY_ACC_ENERGY, m_acc_energy);
        esp_restart();
    }

    setup_console_input(kDisposing);
    setup_ipc(kDisposing);

    SettingsManager::save(KEY_ACC_ENERGY, m_acc_energy);
    SettingsManager::setup(kDisposing);

    ESP_LOGI(TAG, "App destroyed");
}

void PowerMeterApp::setup_ipc(bool disposing)
{
    if (disposing) {
        vQueueDelete(m_interface_queue), m_interface_queue = nullptr;
        vQueueDelete(m_telemetry_queue), m_telemetry_queue = nullptr;
        vQueueDelete(m_indicator_queue), m_indicator_queue = nullptr;
        vEventGroupDelete(m_task_exit_group), m_task_exit_group = nullptr;
    }
    else {
        m_interface_queue = xQueueCreate(10, sizeof(InterfaceTaskMessage));
        m_telemetry_queue = xQueueCreate(10, sizeof(MqttMessage));
        m_indicator_queue = xQueueCreate(10, sizeof(led_mode_t));
        m_task_exit_group = xEventGroupCreate();
    }
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
        4096,                   // Stack size
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
