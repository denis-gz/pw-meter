#pragma once

#include <driver/gpio.h>

class LED
{
public:
    LED(int pin, bool active_level);
    ~LED();

    esp_err_t init();

    void set_state(bool state);
    bool get_state() const;

private:
    gpio_num_t m_pin = GPIO_NUM_NC;
    bool m_state = false;
    bool m_active_level = false;
};

