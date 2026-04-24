#include "PowerMeterApp.h"

#include <esp_log.h>

#include "common.h"
#include "Filters.h"

void PowerMeterApp::display_task()
{
    const TickType_t xTimeout = pdMS_TO_TICKS(100);

    using Filter = AdaptiveDisplayFilter;

    ESP_LOGI(TAG, "display_task: started");
    static Filter filter_v(20);
    static Filter filter_i(0.5);
    static Filter filter_w(10);
    static Filter filter_pf(0.1);
    static Filter filter_hz(0.1);

    char buf[1024];
    const char* fmt = "\n"
                      "\tv_rms           = %.0fV\n"
                      "\ti_rms           = %.2fA\n"
                      "\tapparent_power  = %.1fVA\n"
                      "\treal_power      = %.1fW\n"
                      "\tpower_factor    = %.2f\n"
                      "\tcos_phi         = %.2f\n"
                      "\tfrequency       = %.2fHz\n"
                      "\tcpu0_load       = %.1f%%\n"
                      "\tcpu1_load       = %.1f%%\n"
                      "";

    float cpu0 = 0;
    float cpu1 = 0;
    uint32_t counter = 0;

    ComputeResultMessage qmsg;
    while (!m_stop_tasks) {
        if (xQueueReceive(m_display_queue, &qmsg, xTimeout)) {
            float f_v = filter_v.process(qmsg.v_rms);
            float f_i = filter_i.process(qmsg.i_rms);
            float f_av = qmsg.i_rms ? f_v * f_i : 0.0f;
            float f_w = filter_w.process(qmsg.real_power);
            float f_hz = filter_hz.process(qmsg.frequency);
            float f_cos = filter_pf.process(qmsg.cos_phi);

            float f_pf = NAN;
            if (f_av > 0.0f)
                f_pf = f_w / f_av;
            if (f_pf > 1.0f)
                f_pf = 1.0f;
            if (f_pf < -1.0f)
                f_pf = -1.0f;

            if (++counter % 5 == 0) {
                m_cpu_meter.get_load(cpu0, cpu1);
            }

            snprintf(buf, sizeof(buf), fmt, f_v, f_i, f_av, f_w, f_pf, f_cos, f_hz, cpu0, cpu1);
            ESP_LOGI(TAG, "%s", buf);
        }
    };

    ESP_LOGI(TAG, "display_task: finished");
    vTaskDelete(nullptr);
}
