#pragma once

#include <driver/gpio.h>
#include <driver/gpio_filter.h>

#include <atomic>
#include <functional>
#include <mutex>

class Button
{
public:
    typedef std::function<void()> callback_t;

    Button(int pin, callback_t callback);
    ~Button();

private:
    static std::once_flag s_isr_service_flag;

    std::atomic_int m_counter;
    std::atomic_int64_t m_sample_time;
    std::atomic_int64_t m_depress_time;
    std::atomic_int64_t m_release_time;
    std::atomic_int m_level;

    gpio_num_t m_pin = GPIO_NUM_NC;
    gpio_glitch_filter_handle_t gl_handle = {};

    callback_t m_callback;

    void handler();
};
