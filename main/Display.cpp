#include "PowerMeterApp.h"

#include "Filters.h"

void PowerMeterApp::setup_display(bool disposing)
{
    if (disposing) {
        vQueueDelete(m_display_queue), m_display_queue = nullptr;
        if (m_oled._i2c_dev_handle) {
            i2c_master_bus_rm_device(m_oled._i2c_dev_handle);
            m_oled._i2c_dev_handle = nullptr;
        }
        if (m_oled._i2c_bus_handle) {
            i2c_del_master_bus(m_oled._i2c_bus_handle);
            m_oled._i2c_bus_handle = nullptr;
        }
    }
    else {
        m_display_queue = xQueueCreate(10, sizeof(DisplayTaskMessage));
        i2c_master_init(&m_oled, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);

        #ifdef CONFIG_SSD1306_128x64
        ssd1306_init(&m_oled, 128, 64);
        #elif CONFIG_SSD1306_128x32
        ssd1306_init(&m_oled, 128, 32);
        #endif

        ssd1306_clear_screen(&m_oled, false);
        ssd1306_contrast(&m_oled, 0xFF);
    }
}

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
    static Filter filter_sh(0.3);

    char buf[1024];
    const char* fmt = "\n"
                      "\tv_rms           = %.0fV\n"
                      "\ti_rms           = %.2fA\n"
                      "\tapparent_power  = %.1fVA\n"
                      "\treal_power      = %.1fW\n"
                      "\tenergy          = %.2fWh\n"
                      "\tpower_factor    = %.2f\n"
                      "\tcos_phi         = %.2f\n"
                      "\tfrequency       = %.2fHz\n"
                      "\tcpu0_load       = %.1f%%\n"
                      "\tcpu1_load       = %.1f%%\n"
                      "\tsample_shift    = %.1f\n"
                      "";

    float cpu0 = 0;
    float cpu1 = 0;
    uint32_t counter = 0;
    bool pause = false;

    DisplayTaskMessage qmsg;
    while (!m_stop_tasks) {
        if (xQueueReceive(m_display_queue, &qmsg, xTimeout)) {
            switch (qmsg.type) {
                case MessageType::ComputeResultMessage: {
                    const auto& result = qmsg.compute_result;

                    float f_v = filter_v.process(result.v_rms);
                    float f_i = (result.i_rms == 0.0f) ? (filter_i.reset(), 0.0f) : filter_i.process(result.i_rms);
                    float f_av = f_v * f_i;
                    float f_w = (result.real_power == 0.0f) ? (filter_w.reset(), 0.0f) : filter_w.process(result.real_power);
                    float f_hz = filter_hz.process(result.frequency);
                    float f_cos = filter_pf.process(result.cos_phi);
                    float f_sh = filter_sh.process(result.m_vi_shift);
                    float e = result.energy;

                    float f_pf = NAN;
                    if (f_av > 0.0f)
                        f_pf = f_w / f_av;
                    if (f_pf > 1.0f)
                        f_pf = 1.0f;
                    if (f_pf < -1.0f)
                        f_pf = -1.0f;

                    if (++counter % 5 == 0)
                        m_cpu_meter.get_load(cpu0, cpu1);

                    if (!pause) {
                        snprintf(buf, sizeof(buf), fmt, f_v, f_i, f_av, f_w, e, f_pf, f_cos, f_hz, cpu0, cpu1, f_sh);
                        ESP_LOGI(TAG, "%s", buf);
                    }
                    break;
                }
                case MessageType::InputMessage: {
                    const auto& input = qmsg.input;
                    switch (input.action) {
                        case InputAction::Next:
                        case InputAction::Prev:
                        case InputAction::Confirm:
                            pause = !pause;
                            ESP_LOGI(TAG, "Short press!");
                            break;
                        case InputAction::Back:
                            pause = !pause;
                            ESP_LOGI(TAG, "Long press!");
                            break;
                    }

                }
            }


        }
    };

    ESP_LOGI(TAG, "display_task: finished");
    vTaskDelete(nullptr);
}
