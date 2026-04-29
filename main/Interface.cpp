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

enum InputState {
    PageSelection,
    MenuSelection,
    ItemDisplay,
    ValueInput,
};

enum MenuItem {
    ItemNone,
    ItemWifiSsid,
    ItemWifiPassword,
    ItemINoiseFloor,
    ItemICalib,
    ItemVCalib,

    ItemCount,
    ItemDefault = ItemWifiSsid,
};

struct DisplayContext
{
    using Filter = AdaptiveDisplayFilter;

    Filter filter_v { 20 };
    Filter filter_i { 0.5 };
    Filter filter_w { 10 };
    // Filter filter_pf{ 0.1 };
    Filter filter_hz{ 0.1 };
    // Filter filter_sh{ 0.3 };

    bool with_display = false;

    float cpu0 = 0;
    float cpu1 = 0;
    uint32_t counter = 0;
    uint32_t pause = 0;

    // Display matrix is 16x8 chars
    display_line_t lines[8] {};
    int inverse[8] {};

    void clear_lines(bool with_inverse = true) {
        memset(lines, 0x20, sizeof(lines));
        if (with_inverse)
            memset(inverse, 0, sizeof(inverse));
    }

    std::list<display_line_t> log;

    ScreenPage page {};
    InputState input_state {};
    MenuItem item_selected {};
};

static DisplayContext ds;

void PowerMeterApp::setup_interface(bool disposing)
{
    if (disposing) {
        m_encoder.Stop();

        if (ds.with_display) {
            ssd1306_clear_screen(&m_oled, false);
        }
        if (m_oled._i2c_dev_handle) {
            i2c_master_bus_rm_device(m_oled._i2c_dev_handle);
            m_oled._i2c_dev_handle = nullptr;
        }
        if (m_oled._i2c_bus_handle) {
            i2c_del_master_bus(m_oled._i2c_bus_handle);
            m_oled._i2c_bus_handle = nullptr;
        }
        vQueueDelete(m_interface_queue), m_interface_queue = nullptr;
    }
    else {
        ESP_LOGI(TAG, "interface setup");
        m_interface_queue = xQueueCreate(10, sizeof(InterfaceTaskMessage));

        i2c_master_init(&m_oled, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);

        int retries = 5;
        esp_err_t res;

        while (retries--) {
            res = ssd1306_init(&m_oled, 128, 64);
            if (res == ESP_OK)
                break;
            ESP_LOGI(TAG, "Retrying in 50 msec...");
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        if (res == ESP_OK) {
            ssd1306_clear_screen(&m_oled, false);
            ssd1306_contrast(&m_oled, 0xFF);
            ssd1306_bitmaps(&m_oled, 0, 0, SPLASH_SCREEN, 128, 64, false);

            // Delay before displaying results to let splash screen linger for some time
            ds.pause = 7;
            ds.with_display = true;
        }
        else {
            ESP_LOGI(TAG, "Working without display output");
        }

        m_encoder.Start();
    }
}

void PowerMeterApp::interface_task()
{
    ESP_LOGI(TAG, "interface_task: started");
    setup_interface();

    InterfaceTaskMessage qmsg;
    while (!m_stop_tasks) {
        if (xQueueReceive(m_interface_queue, &qmsg, pdMS_TO_TICKS(100))) {
            switch (qmsg.type) {
                case MessageType::ResultMessage:
                    process_result(qmsg.result);
                    break;
                case MessageType::EncoderInputMessage:
                    process_encoder_input(qmsg.encoder_input);
                    break;
                case MessageType::ConsoleInputMessage:
                    process_console_input(qmsg.console_input);
                    break;
                case MessageType::LogMessage:
                    process_log(qmsg.log);
                    break;
            }
        }
    };

    setup_interface(kDisposing);
    ESP_LOGI(TAG, "interface_task: finished");
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
            ESP_LOGW(TAG, "Telemetry queue is full, dropping MqttMessage");
        }
    }
    if (ds.pause)
        ds.pause--;
    if (!ds.pause) {
        static constexpr const char* fmt = "\n"
            "\tv_rms           = %.0fV\n"
            "\ti_rms           = %.2fA\n"
            "\tapparent_power  = %.1fVA\n"
            "\treal_power      = %.1fW\n"
            "\tenergy          = %.2fWh\n"
            "\tpower_factor    = %.2f\n"
            "\tfrequency       = %.2fHz\n"
            "\tcpu0_load       = %.1f%%\n"
            "\tcpu1_load       = %.1f%%\n"
        //  "\tsample_shift    = %.1f\n"
            "";
        if (0 /*ds.counter % 5 == 0*/) {
            char buf[1024];
            snprintf(buf, sizeof(buf), fmt, m.f_v, m.f_i, m.f_va, m.f_w, m.f_wh, m.f_pf, m.f_hz, ds.cpu0, ds.cpu1/*, f_sh */);
            ESP_LOGI(TAG, "%s", buf);
        }
    }

    switch (ds.page) {
        case MainsPage: {
            if (!ds.pause) {
                ds.clear_lines(false);
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
                ds.clear_lines();
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
                switch (ds.input_state) {
                    case PageSelection:
                    case MenuSelection:
                        snprintf(ds.lines[1].data(), sizeof(display_line_t), "1. Wi-Fi SSID");
                        snprintf(ds.lines[2].data(), sizeof(display_line_t), "2. Wi-Fi passw");
                        snprintf(ds.lines[3].data(), sizeof(display_line_t), "3. I noise flr");
                        snprintf(ds.lines[4].data(), sizeof(display_line_t), "4. I calib");
                        snprintf(ds.lines[5].data(), sizeof(display_line_t), "5. V calib");
                        break;
                    case ItemDisplay:
                        switch (ds.item_selected) {
                            case ItemNone:
                                snprintf(ds.lines[1].data(), sizeof(display_line_t), "- none -");
                                break;
                            case ItemWifiSsid: {
                                wifi_config_t wifi {};
                                esp_wifi_get_config(WIFI_IF_STA, &wifi);
                                snprintf(ds.lines[1].data(), sizeof(display_line_t), "1. Wi-Fi SSID:");
                                snprintf(ds.lines[2].data(), sizeof(display_line_t), "%s", wifi.sta.ssid);
                                break;
                            }
                            case ItemWifiPassword: {
                                wifi_config_t wifi {};
                                esp_wifi_get_config(WIFI_IF_STA, &wifi);
                                string_t pass {};
                                std::fill_n(pass.begin(), std::min(sizeof(string_t), strlen((char*) wifi.sta.password)), '*');
                                snprintf(ds.lines[1].data(), sizeof(display_line_t), "2. Wi-Fi passw:");
                                snprintf(ds.lines[2].data(), sizeof(display_line_t), "%s", pass.data());
                                break;
                            }
                            case ItemINoiseFloor: {
                                snprintf(ds.lines[1].data(), sizeof(display_line_t), "3. I noise flr:");
                                snprintf(ds.lines[2].data(), sizeof(display_line_t), "%f", m_i_noise_floor);
                                break;
                            }
                            case ItemICalib: {
                                snprintf(ds.lines[1].data(), sizeof(display_line_t), "4. I calibr:");
                                snprintf(ds.lines[2].data(), sizeof(display_line_t), "%f", I_COEF);
                                break;
                            }
                            case ItemVCalib: {
                                snprintf(ds.lines[1].data(), sizeof(display_line_t), "3. V calibr:");
                                snprintf(ds.lines[2].data(), sizeof(display_line_t), "%f", V_COEF);
                                break;
                            }
                            default:
                                break;
                        }
                        break;
                    case ValueInput:
                        snprintf(ds.lines[2].data(), sizeof(display_line_t), "Awaiting input");
                        snprintf(ds.lines[3].data(), sizeof(display_line_t), "in console...");
                        break;
                }
            }
            break;
        }
        case LogPage: {
            if (ds.lines[0][0] == 0x20) {
                snprintf(ds.lines[0].data(), sizeof(display_line_t), "----- Log ------");
                auto j = ds.log.begin();
                for (int i = 1; i < countof(ds.lines); i++) {
                    if (j != ds.log.end()) {
                        ds.lines[i] = *j++;
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
        if (ds.page == MainsPage && !ds.pause) {
            inverse = ds.inverse[i];
            if (ds.inverse[i]) {
                ds.inverse[i]--;
            }
        }
        else if (ds.page == SettingsPage) {
            if (ds.input_state == MenuSelection) {
                inverse = (i > 0) && (i == ds.item_selected);
            }
        }
        if (ds.with_display) {
            ssd1306_display_text(&m_oled, i, ds.lines[i].data(), sizeof(display_line_t) - 1, inverse);
        }
    }
}

void PowerMeterApp::process_encoder_input(const EncoderInputMessage& input)
{
    switch (input.action) {
        case EncoderInputAction::Next:
            switch (ds.input_state) {
                case PageSelection:
                    ds.pause = 0;
                    ds.page = (ScreenPage) ((ds.page + 1) % PageCount);
                    ds.clear_lines();
                    break;
                case MenuSelection:
                    ds.item_selected = (MenuItem) ((ds.item_selected + 1) % ItemCount);
                    if (ds.item_selected == ItemNone)
                        ds.item_selected = ItemDefault;
                    break;
                default:
                    break;
            }
            break;
        case EncoderInputAction::Prev:
            switch (ds.input_state) {
                case PageSelection:
                    ds.pause = 0;
                    ds.page = (ScreenPage) ((ds.page + PageCount - 1) % PageCount);
                    ds.clear_lines();
                    break;
                case MenuSelection:
                    ds.item_selected = (MenuItem) ((ds.item_selected + ItemCount - 1) % ItemCount);
                    if (ds.item_selected == ItemNone)
                        ds.item_selected = (MenuItem) (ItemCount - 1);
                    break;
                default:
                    break;
            }
            break;
        case EncoderInputAction::Confirm:
            switch (ds.input_state) {
                case PageSelection:
                    if (ds.page == MainsPage) {
                        if (ds.pause) {
                            ds.pause = 0;
                            ESP_LOGI(TAG, "Resumed");
                        }
                        else {
                            ds.pause = UINT32_MAX;
                            ESP_LOGI(TAG, "Paused");
                        }
                    }
                    if (ds.page == SettingsPage) {
                        ds.input_state = MenuSelection;
                        ds.item_selected = ItemDefault;
                    }
                    break;
                case MenuSelection:
                    ds.clear_lines();
                    ds.input_state = ItemDisplay;
                    break;
                case ItemDisplay:
                    ds.clear_lines();
                    ds.input_state = ValueInput;
                    ds.pause = UINT32_MAX;
                    switch (ds.item_selected) {
                        case ItemWifiSsid: {
                            string_t prompt { "Input new SSID:" };
                            esp_event_post_to(m_console_event_loop, EVENT_CONSOLE_INPUT, STRING_INPUT_ID, &prompt, sizeof(prompt), pdMS_TO_TICKS(100));
                            break;
                        }
                        case ItemWifiPassword: {
                            string_t prompt { "Input new password:" };
                            esp_event_post_to(m_console_event_loop, EVENT_CONSOLE_INPUT, STRING_INPUT_ID, &prompt, sizeof(prompt), pdMS_TO_TICKS(100));
                            break;
                        }
                        case ItemINoiseFloor: {
                            string_t prompt { "Input new I noise floor:" };
                            esp_event_post_to(m_console_event_loop, EVENT_CONSOLE_INPUT, FLOAT_INPUT_ID, &prompt, sizeof(prompt), pdMS_TO_TICKS(100));
                            break;
                        }
                        case ItemICalib: {
                            string_t prompt { "Input new I calibration:" };
                            esp_event_post_to(m_console_event_loop, EVENT_CONSOLE_INPUT, FLOAT_INPUT_ID, &prompt, sizeof(prompt), pdMS_TO_TICKS(100));
                            break;
                        }
                        case ItemVCalib: {
                            string_t prompt { "Input new V calibration:" };
                            esp_event_post_to(m_console_event_loop, EVENT_CONSOLE_INPUT, FLOAT_INPUT_ID, &prompt, sizeof(prompt), pdMS_TO_TICKS(100));
                            break;
                        }
                        default:
                            break;
                    }
                    break;
                default:
                    break;
            }
            break;
        case EncoderInputAction::Back:
            switch (ds.input_state) {
                case PageSelection:
                    ESP_LOGI(TAG, "Restarting...");
                    stop_tasks();
                    break;
                case MenuSelection:
                    ds.clear_lines();
                    ds.input_state = PageSelection;
                    ds.item_selected = ItemNone;
                    break;
                case ItemDisplay:
                    ds.clear_lines();
                    ds.input_state = MenuSelection;
                    ds.pause = 0;
                    break;
                case ValueInput:
                    ds.clear_lines();
                    ds.input_state = ItemDisplay;
                    break;
            }
            break;
    }
}

void PowerMeterApp::process_console_input(const ConsoleInputMessage& input)
{
    if (ds.input_state == ValueInput) {
        switch (input.action) {
            case ConsoleInputAction::Decimal:
                ESP_LOGI(TAG, "Console input: %d", input.decimal_value);
                ds.input_state = ItemDisplay;
                break;
            case ConsoleInputAction::Float:
                switch (ds.item_selected) {
                    case ItemINoiseFloor:
                        if (input.float_value > 0) {
                            ESP_LOGI(TAG, "Setting new I noise floor: %f", input.float_value);
                            m_i_noise_floor = input.float_value;
                        }
                        else {
                            ESP_LOGE(TAG, "Setting new I noise floor: out of range (%f)", input.float_value);
                        }
                        break;
                    case ItemICalib:
                        ESP_LOGW(TAG, "Setting new I calibration: not implemented");
                        break;
                    case ItemVCalib:
                        ESP_LOGW(TAG, "Setting new V calibration: not implemented");
                        break;
                    default:
                        break;
                }
                ds.input_state = ItemDisplay;
                break;
            case ConsoleInputAction::String:
                switch (ds.item_selected) {
                    case ItemWifiSsid:
                        set_wifi_ssid(input.string_value);
                        ds.input_state = MenuSelection;
                        break;
                    case ItemWifiPassword:
                        set_wifi_pass(input.string_value);
                        ds.input_state = MenuSelection;
                        break;
                    default:
                        ds.input_state = ItemDisplay;
                        break;
                }
                break;
            case ConsoleInputAction::None:
                ESP_LOGI(TAG, "Console input cancelled");
                ds.input_state = ItemDisplay;
                break;
        }
        ds.clear_lines();
    }
}

void PowerMeterApp::on_encoder_rotate(bool is_ccw)
{
    InterfaceTaskMessage qmsg {
        .type = MessageType::EncoderInputMessage,
        .encoder_input = { .action = is_ccw ? EncoderInputAction::Prev : EncoderInputAction::Next },
    };
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xQueueSendFromISR(m_interface_queue, &qmsg, &xHigherPriorityTaskWoken) != pdPASS) {
        ESP_LOGW(TAG, "Interface queue is full, dropping EncoderInputMessage");
    }
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

void PowerMeterApp::on_encoder_click(bool is_long)
{
    InterfaceTaskMessage qmsg {
        .type = MessageType::EncoderInputMessage,
        .encoder_input = { .action = is_long ? EncoderInputAction::Back : EncoderInputAction::Confirm },
    };
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xQueueSendFromISR(m_interface_queue, &qmsg, &xHigherPriorityTaskWoken) != pdPASS) {
        ESP_LOGW(TAG, "Interface queue is full, dropping EncoderInputMessage");
    }
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

void PowerMeterApp::process_log(const LogMessage& log)
{
    ds.log.push_back(log.message);
    while (ds.log.size() > countof(ds.lines) - 1) {
        ds.log.pop_front();
    }
}

void PowerMeterApp::post_log(display_line_t message)
{
    ESP_LOGI(TAG, "log event: %s", message.data());
    InterfaceTaskMessage qmsg {
        .type = MessageType::LogMessage,
        .log { .message = message }
    };
    if (xQueueSend(m_interface_queue, &qmsg, pdMS_TO_TICKS(100)) != pdPASS) {
        ESP_LOGW(TAG, "Interface queue is full, dropping LogMessage");
    }
}
