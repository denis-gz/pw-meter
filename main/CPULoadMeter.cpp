#include "CPULoadMeter.h"

#include <cstring>

void CPULoadMeter::get_load(float& core0_load, float& core1_load)
{
    UBaseType_t num_tasks = uxTaskGetNumberOfTasks();

    // Allocate memory to hold the snapshot of all running tasks
    auto task_array = (TaskStatus_t*) alloca(num_tasks * sizeof(TaskStatus_t));
    uint64_t current_total_time = 0;

    if (task_array != NULL) {
        // Take the snapshot!
        num_tasks = uxTaskGetSystemState(task_array, num_tasks, &current_total_time);

        uint64_t current_idle0_time = 0;
        uint64_t current_idle1_time = 0;

        // Search the snapshot for the two FreeRTOS IDLE tasks
        for (UBaseType_t i = 0; i < num_tasks; i++) {
            if (std::strncmp(task_array[i].pcTaskName, "IDLE0", 5) == 0) {
                current_idle0_time = task_array[i].ulRunTimeCounter;
            }
            if (std::strncmp(task_array[i].pcTaskName, "IDLE1", 5) == 0) {
                current_idle1_time = task_array[i].ulRunTimeCounter;
            }
        }

        // Calculate the delta (time spent in the last cycle)
        uint64_t total_delta = current_total_time - prev_total_time;
        uint64_t idle0_delta = current_idle0_time - prev_idle0_time;
        uint64_t idle1_delta = current_idle1_time - prev_idle1_time;

        // Calculate percentage
        if (total_delta > 0 && prev_total_time > 0) {
            core0_load = 100.0f - ((idle0_delta / (float)total_delta) * 100.0f);
            core1_load = 100.0f - ((idle1_delta / (float)total_delta) * 100.0f);
        }
        else {
            core0_load = 0.0f;
            core1_load = 0.0f;
        }

        // Clean up float rounding quirks
        if (core0_load < 0.0f)
            core0_load = 0.0f;
        if (core1_load < 0.0f)
            core1_load = 0.0f;

        // Save the current state for the next time this function is called
        prev_total_time = current_total_time;
        prev_idle0_time = current_idle0_time;
        prev_idle1_time = current_idle1_time;
    }
}
