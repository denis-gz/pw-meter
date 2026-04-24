#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class CPULoadMeter
{
public:
    // Call this roughly once per second in your display update task
    void get_load(float& core0_load, float& core1_load);

private:
    TaskStatus_t* task_array = nullptr;

    uint64_t prev_total_time = 0;
    uint64_t prev_idle0_time = 0;
    uint64_t prev_idle1_time = 0;
};
