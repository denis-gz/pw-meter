#include "RotaryEncoder.h"

#include <freertos/FreeRTOS.h>
#include <driver/gpio.h>
#include <esp_log.h>

#include "common.h"

RotaryEncoder::RotaryEncoder(int pin_S1, int pin_S2, std::function<void(bool)> callback)
{
    m_callback = callback;

    pcnt_unit_config_t pcnt_unit_config = {
        .low_limit = -1,
        .high_limit = 1,
        .flags { .accum_count = 1 },
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&pcnt_unit_config, &m_unit));

    pcnt_event_callbacks_t cbs = { .on_reach = pcnt_watch_callback };
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(m_unit, &cbs, this));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(m_unit, -1));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(m_unit, 1));

    pcnt_glitch_filter_config_t pcnt_glitch_config = { .max_glitch_ns = 7500 };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(m_unit, &pcnt_glitch_config));

    ESP_ERROR_CHECK(gpio_pullup_en(static_cast<gpio_num_t>(pin_S1)));
    ESP_ERROR_CHECK(gpio_pullup_en(static_cast<gpio_num_t>(pin_S2)));

    pcnt_chan_config_t pcnt_chan_config = {
        .edge_gpio_num = pin_S1,
        .level_gpio_num = pin_S2,
    };
    ESP_ERROR_CHECK(pcnt_new_channel(m_unit, &pcnt_chan_config, &m_chan));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(m_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(m_chan, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_enable(m_unit));
}

RotaryEncoder::~RotaryEncoder()
{
    pcnt_unit_disable(m_unit);
    pcnt_del_channel(m_chan);
    pcnt_del_unit(m_unit);
}

void RotaryEncoder::Start()
{
    pcnt_unit_start(m_unit);
}

void RotaryEncoder::Stop()
{
    pcnt_unit_stop(m_unit);
}

void RotaryEncoder::ResetCounter()
{
    pcnt_unit_clear_count(m_unit);
}

int RotaryEncoder::Counter() const
{
    int counter = 0;
    pcnt_unit_get_count(m_unit, &counter);
    return counter;
}


bool IRAM_ATTR RotaryEncoder::pcnt_watch_callback(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t* edata, void* user_ctx)
{
    auto& _ = *static_cast<RotaryEncoder*>(user_ctx);

    int counter = 0;
    pcnt_unit_get_count(unit, &counter);
    _.m_callback(counter > _.m_counter);
    _.m_counter = counter;

    return false;
}
