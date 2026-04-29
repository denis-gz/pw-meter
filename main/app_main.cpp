#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>
#include <fcntl.h>
#include <memory>

#include "common.h"
#include "PowerMeterApp.h"

constexpr const char* TAG = "pw-meter";

extern "C" void app_main() {
    setenv("TZ", "EET-2EEST,M3.5.0/3,M10.5.0/4", 1);
    tzset();

    // Set stdin to non-blocking or simple raw mode
    fcntl(fileno(stdin), F_SETFL, 0);

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    do {
        auto app = std::make_unique<PowerMeterApp>();

        while (!app->is_stop_tasks()) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(TAG, "Restarting...");
    }
    while (1);
}
