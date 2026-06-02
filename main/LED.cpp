#include "LED.h"

LED::LED(int pin, bool active_level)
    : m_pin(static_cast<gpio_num_t>(pin))
    , m_active_level(active_level)
{ }

LED::~LED()
{
    if (m_pin != GPIO_NUM_NC) {
        gpio_reset_pin(m_pin);
    }
}

esp_err_t LED::init()
{
    gpio_config_t gpio_conf = {
        .pin_bit_mask = 1ULL << m_pin,
        .mode         = GPIO_MODE_OUTPUT,
    };
    esp_err_t ret;
    ESP_ERROR_CHECK(ret = gpio_config(&gpio_conf));
    ESP_ERROR_CHECK(ret = gpio_set_level(m_pin, !m_active_level));
    return ret;
}

void LED::set_state(bool state)
{
    gpio_set_level(m_pin, state ? m_active_level : !m_active_level);
    m_state = state;
}

bool LED::get_state() const
{
    return m_state;
}

