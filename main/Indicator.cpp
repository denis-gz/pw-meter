#include "PowerMeterApp.h"

void PowerMeterApp::indicator_task()
{
    ESP_LOGI(TAG, "indicator_task: started");
    setup_indicator();

    led_mode_t current_mode = LED_STATE_OFF;
    led_mode_t background_mode = LED_STATE_OFF; // Remembers state during transient flashes

    uint32_t tick_count = 0;
    int transient_ticks_remaining = 0;

    while (!m_stop_tasks) {
        tick_count++;

        // Non-blocking check for incoming status updates
        if (led_mode_t new_mode; xQueueReceive(m_indicator_queue, &new_mode, 0)) {
            if (new_mode == LED_STATE_MQTT_TX) {
                current_mode = LED_STATE_MQTT_TX;
                transient_ticks_remaining = 4;
            }
            else if (new_mode == LED_STATE_MQTT_ERROR) {
                current_mode = LED_STATE_MQTT_ERROR;
                transient_ticks_remaining = 12; // 3 full cycles of 100ms ON / 100ms OFF (12 ticks total)
            }
            else {
                // Persistent states update both the current view and the background target
                current_mode = new_mode;
                background_mode = new_mode;
                transient_ticks_remaining = 0;
            }
        }

        // --- State Machine Logic ---

        // Handle Transient Animations
        if (transient_ticks_remaining > 0) {
            transient_ticks_remaining--;

            // Toggle every 2 ticks (100ms)
            int phase = transient_ticks_remaining % 4;
            m_led.set_state(phase < 2);

            // If animation just finished, seamlessly restore the background state
            if (transient_ticks_remaining == 0) {
                current_mode = background_mode;
            }
        }

        // Handle Persistent Background States
        if (transient_ticks_remaining == 0) {
            switch (current_mode) {
                case LED_STATE_OFF:
                    m_led.set_state(false);
                    break;
                case LED_STATE_WIFI_SEARCHING:
                    // Slow blink: Toggle every 500ms (10 ticks * 50ms)
                    if (tick_count % 10 == 0) {
                        static bool toggle = false;
                        toggle = !toggle;
                        m_led.set_state(toggle);
                    }
                    break;
                case LED_STATE_CONNECTED:
                    m_led.set_state(true);
                    break;
                default:
                    m_led.set_state(false);
                    break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // 50ms precise time slice
    }

    setup_indicator(kDisposing);
    ESP_LOGI(TAG, "indicator_task: finished");
    vTaskDelete(nullptr);
}

void PowerMeterApp::setup_indicator(bool disposing)
{
    if (disposing) {
        vQueueDelete(m_indicator_queue), m_indicator_queue = nullptr;
    }
    else {
        ESP_LOGI(TAG, "indicator setup");
        m_indicator_queue = xQueueCreate(10, sizeof(led_mode_t));
    }
}

void PowerMeterApp::set_indicator(led_mode_t mode)
{
    if (m_indicator_queue) {
        xQueueSend(m_indicator_queue, &mode, 0);
    }
}
