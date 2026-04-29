#include "PowerMeterApp.h"

ESP_EVENT_DEFINE_BASE(EVENT_CONSOLE_INPUT);

void PowerMeterApp::setup_console_input(bool disposing)
{
    if (disposing) {
        esp_event_handler_unregister_with(m_console_event_loop,
            EVENT_CONSOLE_INPUT, ESP_EVENT_ANY_ID,
            member_cast<esp_event_handler_t>(&PowerMeterApp::on_console_input_event));
        esp_event_loop_delete(m_console_event_loop);
    }
    else {
        ESP_LOGI(TAG, "console input setup");
        esp_event_loop_args_t args {
            .queue_size = 5,
            .task_name = "console_input_task",
            .task_priority = 1,
            .task_stack_size = 4096,
            .task_core_id = xPortGetCoreID(),
        };
        ESP_ERROR_CHECK(esp_event_loop_create(&args, &m_console_event_loop));
        ESP_ERROR_CHECK(esp_event_handler_register_with(m_console_event_loop,
            EVENT_CONSOLE_INPUT, ESP_EVENT_ANY_ID,
            member_cast<esp_event_handler_t>(&PowerMeterApp::on_console_input_event), this
        ));
    }
}

void PowerMeterApp::on_console_input_event(esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    string_t prompt = *(string_t*) event_data;
    printf("%s", prompt.data());

    InterfaceTaskMessage qmsg {
        .type = MessageType::ConsoleInputMessage,
        .console_input { .action = ConsoleInputAction::None },
    };
    switch (event_id) {
        case DECIMAL_INPUT_ID: {
            int value;
            if (input_value(value)) {
                qmsg.console_input = {
                    .action = ConsoleInputAction::Decimal,
                    .decimal_value = value,
                };
            }
            break;
        }
        case FLOAT_INPUT_ID: {
            float value;
            if (input_value(value)) {
                qmsg.console_input = {
                    .action = ConsoleInputAction::Float,
                    .float_value = value,
                };
            }
            break;
        }
        case STRING_INPUT_ID: {
            string_t value;
            if (input_value(value)) {
                qmsg.console_input = {
                    .action = ConsoleInputAction::String,
                    .string_value = value,
                };
            }
            break;
        }
        default:
            break;
    }
    if (xQueueSend(m_interface_queue, &qmsg, pdMS_TO_TICKS(200)) != pdPASS) {
        ESP_LOGW(TAG, "Interface queue is full, dropping ConsoleInputMessage");
    }
}

bool PowerMeterApp::input_value(int& value)
{
    string_t input {};
    if (get_console_input(input.data(), sizeof(input))) {
        char* endptr;
        int val = strtod(input.data(), &endptr);
        if (endptr != input.data()) { // Check if we actually parsed a number
            value = val;
            return true;
        }
    }
    return false;
}

bool PowerMeterApp::input_value(float& value)
{
    string_t input {};
    if (get_console_input(input.data(), sizeof(input))) {
        char* endptr;
        float val = strtof(input.data(), &endptr);
        if (endptr != input.data()) { // Check if we actually parsed a number
            value = val;
            return true;
        }
    }
    return false;
}

bool PowerMeterApp::input_value(string_t& value)
{
    string_t input {};
    if (get_console_input(input.data(), sizeof(input))) {
        value = input;
        return true;
    }
    return false;
}

bool PowerMeterApp::get_console_input(char* buffer, size_t max_len)
{
    int c;
    while ((c = getchar()) != EOF) {
        // Skip any chars which possibly existed in stdin before the prompt
    }

    size_t index = 0;
    printf("> "); // Prompt
    fflush(stdout);

    while (index < max_len - 1) {
        c = getchar();
        // If the call is non-blocking, getchar() returns immediately
        if (c == 0xFFFF || c == EOF) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        // Handle Escape (Cancel)
        if (c == 27) {
            printf("\n[Canceled]\n");
            buffer[0] = '\0';
            return false;
        }

        // Handle Enter (Submit)
        if (c == '\n' || c == '\r') {
            buffer[index] = '\0';
            printf("\n");
            return true;
        }

        // Handle Backspace (Delete)
        if (c == 0x08 || c == 0x7f) {
            if (index > 0) {
                index--;
                printf("\b \b");
                fflush(stdout);
            }
        }
        // Handle Printable Characters
        else if (c >= 32 && c <= 126) {
            buffer[index++] = (char)c;
            putchar(c);
            fflush(stdout);
        }
    }
    buffer[index] = '\0';
    return true;
}
