#include "PowerMeterApp.h"

#include "Filters.h"
#include "../resources/SplashScreen_PowerMeter_128x64.h"
#include <list>

#include <esp_wifi.h>
#include <esp_ota_ops.h>

enum ScreenPage: uint8_t {
    MainsPage,
    DevicePage,
    NetworkPage,
    SettingsPage,
    LogPage,
    AboutPage,

    PageCount
};

struct DisplayState
{
    using Filter = AdaptiveDisplayFilter;

    Filter filter_v { 20 };
    Filter filter_i { 0.5 };
    Filter filter_w { 10 };
    // Filter filter_pf{ 0.1 };
    Filter filter_hz{ 0.1 };
    // Filter filter_sh{ 0.3 };

    float cpu0 = 0;
    float cpu1 = 0;
    uint32_t counter = 0;
    bool pause = false;

    // Display matrix is 16x8 chars
    display_line_t lines[8] {};
    int inverse[8] {};

    std::list<display_line_t> log;

    ScreenPage page = MainsPage;
};

static DisplayState ds;

void PowerMeterApp::setup_display(bool disposing)
{
    if (disposing) {
        ssd1306_clear_screen(&m_oled, false);
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
        ssd1306_bitmaps(&m_oled, 0, 0, SPLASH_SCREEN, 128, 64, false);

        // 5. Hold the splash screen for 3 seconds so the user can see it.
        vTaskDelay(pdMS_TO_TICKS(1500));
    }
}

void PowerMeterApp::display_task()
{
    const TickType_t xTimeout = pdMS_TO_TICKS(100);
    ESP_LOGI(TAG, "display_task: started");

    DisplayTaskMessage qmsg;
    while (!m_stop_tasks) {
        if (xQueueReceive(m_display_queue, &qmsg, xTimeout)) {
            switch (qmsg.type) {
                case MessageType::ResultMessage:
                    process_result(qmsg.result);
                    break;
                case MessageType::InputMessage:
                    process_input(qmsg.input);
                    break;
                case MessageType::LogMessage:
                    process_log(qmsg.log);
                    break;
            }
        }
    };

    ESP_LOGI(TAG, "display_task: finished");
    vTaskDelete(nullptr);
}

void PowerMeterApp::process_result(const ResultMessage& result)
{
    MqttMessage m {};
    m.f_v = ds.filter_v.process(result.v_rms, &ds.inverse[0]);
    m.f_i = (result.i_rms == 0.0f)
        ? (ds.filter_i.reset(), 0.0f)
        : ds.filter_i.process(result.i_rms, &ds.inverse[1]);
    m.f_va = m.f_v * m.f_i;
    m.f_w = (result.real_power == 0.0f)
        ? (ds.filter_w.reset(), 0.0f)
        : ds.filter_w.process(result.real_power, &ds.inverse[2]);
    m.f_hz = ds.filter_hz.process(result.frequency, &ds.inverse[6]);
    //float f_cos = filter_pf.process(result.cos_phi);
    //float f_sh = filter_sh.process(result.vi_shift);
    m.f_wh = result.energy;

    m.f_pf = NAN;
    if (m.f_va > 0.0f)
        m.f_pf = m.f_w / m.f_va;
    if (m.f_pf > 1.0f)
        m.f_pf = 1.0f;
    if (m.f_pf < -1.0f)
        m.f_pf = -1.0f;

    ++ds.counter;
    if (ds.counter % 5 == 0)    // Roughly 1 sec
        m_cpu_meter.get_load(ds.cpu0, ds.cpu1);

    if (ds.counter % 15 == 0) { // Roughly 3 sec
        if (xQueueSend(m_telemetry_queue, &m, 0) != pdPASS) {
            ESP_LOGI(TAG, "Warning: Telemetry queue is full, dropping message\n");
        }
    }

    static const char* fmt = "\n"
                      "\tv_rms           = %.0fV\n"
                      "\ti_rms           = %.2fA\n"
                      "\tapparent_power  = %.1fVA\n"
                      "\treal_power      = %.1fW\n"
                      "\tenergy          = %.2fWh\n"
                      "\tpower_factor    = %.2f\n"
                      "\tfrequency       = %.2fHz\n"
                      "\tcpu0_load       = %.1f%%\n"
                      "\tcpu1_load       = %.1f%%\n"
    //                "\tsample_shift    = %.1f\n"
                      "";

    if (!ds.pause) {
        char buf[1024];
        snprintf(buf, sizeof(buf), fmt, m.f_v, m.f_i, m.f_va, m.f_w, m.f_wh, m.f_pf, m.f_hz, ds.cpu0, ds.cpu1/*, f_sh */);
        ESP_LOGI(TAG, "%s", buf);
    }

    switch (ds.page) {
        case MainsPage: {
            if (!ds.pause) {
                memset(ds.lines, 0x20, sizeof(ds.lines));
                snprintf(ds.lines[0].data(), sizeof(display_line_t), "V_rms : %.0fV", m.f_v);
                snprintf(ds.lines[1].data(), sizeof(display_line_t), "I_rms : %.2fA", m.f_i);
                snprintf(ds.lines[2].data(), sizeof(display_line_t), "F_pwr : %.1fVA", m.f_va);
                snprintf(ds.lines[3].data(), sizeof(display_line_t), "R_pwr : %.1fW", m.f_w);
                snprintf(ds.lines[4].data(), sizeof(display_line_t), "Enrgy : %.2fWh", m.f_wh);
                if (std::isnan(m.f_pf))
                    snprintf(ds.lines[5].data(), sizeof(display_line_t), "PF    : (n/a)");
                else
                    snprintf(ds.lines[5].data(), sizeof(display_line_t), "PF    : %.2f", m.f_pf);
                snprintf(ds.lines[6].data(), sizeof(display_line_t), "Freq  : %.2fHz", m.f_hz);
                ds.inverse[3] = ds.inverse[0] || ds.inverse[1];
                ds.inverse[5] = ds.inverse[0] || ds.inverse[1];
            }
            break;
        }
        case DevicePage: {
            if (!ds.pause) {
                memset(ds.lines, 0x20, sizeof(ds.lines));
                time_t utc_now = std::time(nullptr);
                if (tm* local = std::localtime(&utc_now)) {
                    strftime(ds.lines[0].data(), sizeof(display_line_t), "%H:%M:%S", local);
                    strftime(ds.lines[1].data(), sizeof(display_line_t), "%a %d.%m.%Y", local);
                }
                uint32_t secs = pdTICKS_TO_MS(xTaskGetTickCount()) / 1000;
                uint16_t hours = secs / 3600;
                uint16_t minutes = (secs % 3600) / 60;
                uint16_t seconds = secs % 60;
                size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_8BIT);
                size_t largest_block = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
                snprintf(ds.lines[3].data(), sizeof(display_line_t), "Uptm : %03hu:%02hu:%02hu", hours, minutes, seconds);
                snprintf(ds.lines[4].data(), sizeof(display_line_t), "Cpu0 : %4.1f%%", ds.cpu0);
                snprintf(ds.lines[5].data(), sizeof(display_line_t), "Cpu1 : %4.1f%%", ds.cpu1);
                snprintf(ds.lines[6].data(), sizeof(display_line_t), "Heap : %4uK", free_heap / 1024);
                snprintf(ds.lines[7].data(), sizeof(display_line_t), "LBlk : %4uK", largest_block / 1024);
            }
            break;
        }
        case NetworkPage: {
            if (ds.lines[0][0] == 0x20) {
                wifi_config_t wifi {};
                esp_wifi_get_config(WIFI_IF_STA, &wifi);

                esp_netif_ip_info_t ip_info {};
                esp_netif_get_ip_info(m_netif, &ip_info);

                snprintf(ds.lines[0].data(), sizeof(display_line_t), "---- Wi-Fi -----");
                snprintf(ds.lines[1].data(), sizeof(display_line_t), "SSID:%s", wifi.sta.ssid);
                snprintf(ds.lines[2].data(), sizeof(display_line_t), "Channel:%d", wifi.sta.channel);

                uint8_t a, b, c, d;
                a = ip_info.ip.addr >> 0;
                b = ip_info.ip.addr >> 8;
                c = ip_info.ip.addr >> 16;
                d = ip_info.ip.addr >> 24;
                snprintf(ds.lines[4].data(), sizeof(display_line_t), "%d.%d.%d.%d", a, b, c, d);

                a = ip_info.netmask.addr >> 0;
                b = ip_info.netmask.addr >> 8;
                c = ip_info.netmask.addr >> 16;
                d = ip_info.netmask.addr >> 24;
                snprintf(ds.lines[5].data(), sizeof(display_line_t), "%d.%d.%d.%d", a, b, c, d);

                a = ip_info.gw.addr >> 0;
                b = ip_info.gw.addr >> 8;
                c = ip_info.gw.addr >> 16;
                d = ip_info.gw.addr >> 24;
                snprintf(ds.lines[6].data(), sizeof(display_line_t), "%d.%d.%d.%d", a, b, c, d);
            }
            break;
        }
        case SettingsPage: {
            if (ds.lines[0][0] == 0x20) {
                snprintf(ds.lines[0].data(), sizeof(display_line_t), "--- Settings ---");
                snprintf(ds.lines[1].data(), sizeof(display_line_t), "1. TODO");
                snprintf(ds.lines[2].data(), sizeof(display_line_t), "2. TODO");
                snprintf(ds.lines[3].data(), sizeof(display_line_t), "3. TODO");
            }
            break;
        }
        case LogPage: {
            if (ds.lines[0][0] == 0x20) {
                snprintf(ds.lines[0].data(), sizeof(display_line_t), "----- Log ------");
                auto j = ds.log.begin();
                for (int i = 1; i < countof(ds.lines); i++) {
                    if (ds.log.size() >= i) {
                        auto log_line = *j++;
                        ds.lines[i] = log_line;
                    }
                }
            }
            break;
        }
        case AboutPage: {
            if (ds.lines[0][0] == 0x20) {
                auto app = esp_app_get_description();
                snprintf(ds.lines[0].data(), sizeof(display_line_t), "---- About -----");
                snprintf(ds.lines[1].data(), sizeof(display_line_t), "Power Meter");
                snprintf(ds.lines[3].data(), sizeof(display_line_t), "Ver: %s", app->version);
                snprintf(ds.lines[4].data(), sizeof(display_line_t), "IDF: %s", app->idf_ver);
                snprintf(ds.lines[6].data(), sizeof(display_line_t), "(c) 2026");
                snprintf(ds.lines[7].data(), sizeof(display_line_t), "Denys Zavorotnyi");
            }
            break;
        }
        default:
            break;
    }
    for (size_t i = 0; i < countof(ds.lines); ++i) {
        bool inverse = false;
        if (ds.page == MainsPage) {
            inverse = ds.inverse[i];
            if (ds.inverse[i]) {
                ds.inverse[i]--;
            }
        }
        ssd1306_display_text(&m_oled, i, ds.lines[i].data(), sizeof(display_line_t) - 1, inverse);
    }
}

void PowerMeterApp::process_input(const InputMessage& input)
{
    switch (input.action) {
        case InputAction::Next:
            ds.page = (ScreenPage) ((ds.page + 1) % PageCount);
            memset(ds.lines, 0x20, sizeof(ds.lines));
            memset(ds.inverse, 0, sizeof(ds.inverse));
            break;
        case InputAction::Prev:
            ds.page = (ScreenPage) ((ds.page + PageCount - 1) % PageCount);
            memset(ds.lines, 0x20, sizeof(ds.lines));
            memset(ds.inverse, 0, sizeof(ds.inverse));
            break;
        case InputAction::Confirm:
            if (ds.pause)
                ESP_LOGI(TAG, "Resumed");
            else
                ESP_LOGI(TAG, "Paused");
            ds.pause = !ds.pause;
            break;
        case InputAction::Back:
            ESP_LOGI(TAG, "Restarting...");
            m_stop_tasks = true;
            break;
    }
}

void PowerMeterApp::process_log(const LogMessage& log)
{
    while (ds.log.size() > countof(ds.lines) - 1) {
        ds.log.pop_front();
    }
    ds.log.push_back(log.message);
}
