#include "PowerMeterApp.h"

void PowerMeterApp::setup_input(bool disposing)
{
    if (disposing) {
        m_encoder.Stop();
    }
    else {
        m_encoder.Start();
    }
}

void PowerMeterApp::on_rotate(bool is_ccw)
{
    DisplayTaskMessage qmsg {
        .type = MessageType::InputMessage,
        .input = { .action = is_ccw ? InputAction::Prev : InputAction::Next },
    };
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xQueueSendFromISR(m_display_queue, &qmsg, &xHigherPriorityTaskWoken) != pdPASS) {
        ESP_LOGI(TAG, "Warning: Display queue is full, dropping message\n");
    }
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

void PowerMeterApp::on_click(bool is_long)
{
    DisplayTaskMessage qmsg {
        .type = MessageType::InputMessage,
        .input = { .action = is_long ? InputAction::Back : InputAction::Confirm },
    };
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xQueueSendFromISR(m_display_queue, &qmsg, &xHigherPriorityTaskWoken) != pdPASS) {
        ESP_LOGI(TAG, "Warning: Display queue is full, dropping message\n");
    }
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}
