#pragma once

#include <driver/pulse_cnt.h>
#include <functional>

class RotaryEncoder
{
public:
    RotaryEncoder(int pin_S1, int pin_S2, std::function<void(bool)> callback);
    ~RotaryEncoder();

    void Start();
    void Stop();
    void ResetCounter();
    int Counter() const;

private:
    static bool pcnt_watch_callback(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t* edata, void* user_ctx);

    int m_counter = 0;
    pcnt_unit_handle_t m_unit = {};
    pcnt_channel_handle_t m_chan = {};
    std::function<void(bool)> m_callback;
};

