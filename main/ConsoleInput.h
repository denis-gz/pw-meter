#pragma once

#include <esp_event.h>

ESP_EVENT_DECLARE_BASE(EVENT_CONSOLE_INPUT);

enum console_input_event_id_t
{
    DECIMAL_INPUT_ID,
    FLOAT_INPUT_ID,
    STRING_INPUT_ID,
};
