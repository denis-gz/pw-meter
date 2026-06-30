#include "PowerMeterApp.h"

#include "Filters.h"
#include "../resources/SplashScreen_PowerMeter_128x64.h"
#include <list>

#include <esp_wifi.h>
#include <esp_ota_ops.h>
#include "pin_creds.h"

static const std::array<uint8_t, 4> SETTINGS_PIN = DEVICE_PIN;

#define READINGS_PER_SECOND 5

enum ScreenPage: uint8_t {
    SplashScreen,
    MainsPage,
    DevicePage,
    SettingsPage,
    LogPage,
    AboutPage,

    PageCount
};

enum InputState {
    PageSelection,
    PinInput,
    MenuSelection,
    ItemDisplay,
    ValueInput,
};

enum MenuItem {
    ItemNone = -1,
    ItemWifiSsid,
    ItemWifiPassword,
    ItemWifiInfo,
    ItemMqttUri,
    ItemMqttCreds,
    ItemMqttTopic,
    ItemMqttPeriod,
    ItemINoiseFloor,
    ItemICalib,
    ItemVCalib,
    ItemVEnergySave,
    ItemResetEnrgy,

    ItemCount,
    ItemDefault = ItemWifiSsid,
};

static const int N_DISPLAYED_ITEMS = 7;

static constexpr const char MENU_ITEMS[ItemCount][16] {
    { "1. Wi-Fi SSID" },
    { "2. Wi-Fi passw" },
    { "3. Wi-Fi info" },
    { "4. MQTT URI" },
    { "5. MQTT creds" },
    { "6. MQTT topic" },
    { "7. MQTT period" },
    { "8. I noise flr" },
    { "9. I calib" },
    { "10. V calib" },
    { "11. V nrgy save" },
    { "12. Reset enrgy" }
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
    bool is_display_awake = true;
    TickType_t last_interaction_tick = 0;

    uint32_t counter = 0;
    uint32_t pause = 0;

    display_line_t lines[8] {};     // Display matrix is 16x8 chars
    int inverse[8] {};              // Inversion flags for each line

    std::list<display_line_t> log;

    ScreenPage page {};
    InputState input_state {};
    MenuItem item_selected {};
    uint8_t first_displayed_menu_item = 0;
    PowerMeterApp::MqttMessage::DeviceMetrics dev {};

    bool settings_unlocked = false;
    std::array<uint8_t, std::size(SETTINGS_PIN)> pin_digits {};
    uint8_t pin_index = 0;

    void clear_screen(bool clear_inverse = true) {
        memset(lines, 0x20, sizeof(lines));
        if (clear_inverse)
            memset(inverse, 0, sizeof(inverse));
    }

    void set_next_page() {
        page = (ScreenPage) ((page + 1) % PageCount);
        if (page == SplashScreen)
            page = (ScreenPage) (SplashScreen + 1);
    }

    void set_prev_page() {
        page = (ScreenPage) ((page + PageCount - 1) % PageCount);
        if (page == SplashScreen)
            page = (ScreenPage) (PageCount - 1);
    }

    void set_next_item() {
        item_selected = (MenuItem) ((item_selected + 1) % ItemCount);
        update_viewport();
    }

    void set_prev_item() {
        item_selected = (MenuItem) ((item_selected + ItemCount - 1) % ItemCount);
        update_viewport();
    }

    void update_viewport() {
        if (first_displayed_menu_item > item_selected)
            first_displayed_menu_item = item_selected;
        else if (first_displayed_menu_item < item_selected - N_DISPLAYED_ITEMS + 1)
            first_displayed_menu_item = item_selected - N_DISPLAYED_ITEMS + 1;
        lines[0].fill(0x20);    // Invalidate the page
    }
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
        m_led.init();

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
            ssd1306_contrast(&m_oled, 0x7F);
            ssd1306_bitmaps(&m_oled, 0, 0, SPLASH_SCREEN, 128, 64, false);

            // Delay before displaying results to let splash screen linger for some time
            ds.pause = 8;   // 8 * 0.2 = 1.6 sec
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

    ds.is_display_awake = true;
    ds.last_interaction_tick = xTaskGetTickCount();
    const TickType_t DISPLAY_TIMEOUT_TICKS = pdMS_TO_TICKS(60000);

    InterfaceTaskMessage qmsg;
    while (!m_stop_tasks) {
        if (xQueueReceive(m_interface_queue, &qmsg, pdMS_TO_TICKS(100))) {
            switch (qmsg.type) {
                case MessageType::ResultMessage:
                    process_result(qmsg.result);
                    break;
                case MessageType::EncoderInputMessage:
                    if (!ds.is_display_awake) {
                        // Asleep: Wake ONLY on clicks (Confirm or Back)
                        if (qmsg.encoder_input.action == EncoderInputAction::Confirm ||
                            qmsg.encoder_input.action == EncoderInputAction::Back) {

                            ESP_LOGI(TAG, "Wake click detected. Powering up OLED.");
                            ssd1306_sleep(&m_oled, false);
                            ds.is_display_awake = true;
                            ds.last_interaction_tick = xTaskGetTickCount();
                        }
                        // We intentionally do NOT call process_encoder_input here.
                        // This consumes the click and prevents phantom scrolling while asleep.
                    }
                    else {
                        // Awake: Reset the sleep timer and process the action normally
                        ds.last_interaction_tick = xTaskGetTickCount();
                        process_encoder_input(qmsg.encoder_input);
                    }
                    break;
                case MessageType::ConsoleInputMessage:
                    ds.last_interaction_tick = xTaskGetTickCount();
                    process_console_input(qmsg.console_input);
                    break;
                case MessageType::LogMessage:
                    process_log(qmsg.log);
                    break;
            }
        }

        // --- IDLE TIMEOUT EVALUATION ---
        if (ds.with_display && ds.is_display_awake) {
            if ((xTaskGetTickCount() - ds.last_interaction_tick) > DISPLAY_TIMEOUT_TICKS) {
                ESP_LOGI(TAG, "Idle timeout reached. Sleeping display.");
                ssd1306_sleep(&m_oled, true);
                ds.is_display_awake = false;
                ds.settings_unlocked = false;
                if (ds.page == SettingsPage) {
                    ds.input_state = PageSelection;
                    ds.item_selected = ItemNone;
                    ds.first_displayed_menu_item = 0;
                    ds.pin_index = 0;
                    ds.pin_digits.fill(0);
                    ds.clear_screen();
                }
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
    m.time = std::time(0);
    m.power.f_v = ds.filter_v.process(result.v_rms, &ds.inverse[0]);
    m.power.f_i = (result.i_rms == 0.0f)
        ? (ds.filter_i.reset(), ds.inverse[1] = false, 0.0f)
        : ds.filter_i.process(result.i_rms, &ds.inverse[1]);
    m.power.f_va = m.power.f_v * m.power.f_i;
    m.power.f_w = (result.real_power == 0.0f)
        ? (ds.filter_w.reset(), ds.inverse[2] = false, 0.0f)
        : ds.filter_w.process(result.real_power, &ds.inverse[2]);
    m.power.f_hz = (m.power.f_v < 10)
        ? (ds.filter_hz.reset(), ds.inverse[6] = false, NAN)
        : ds.filter_hz.process(result.frequency, &ds.inverse[6]);
    //float f_cos = filter_pf.process(result.cos_phi);
    //float f_sh = filter_sh.process(result.vi_shift);
    if (m.power.f_v < m_settings.v_acc_energy_save) {
        if (!m_acc_energy_saved) {
            SettingsManager::save(KEY_ACC_ENERGY, m_acc_energy);
            m_acc_energy_saved = true;
        }
    }
    else {
        m_acc_energy_saved = false;
    }
    m.power.f_wh = result.energy;
    m.power.f_pf = NAN;
    if (m.power.f_va > 0.0f)
        m.power.f_pf = m.power.f_w / m.power.f_va;
    if (m.power.f_pf > 1.0f)
        m.power.f_pf = 1.0f;
    if (m.power.f_pf < -1.0f)
        m.power.f_pf = -1.0f;

    ++ds.counter;

    // Period is 1 sec on DevicePage; 5 sec in any other condition
    if (ds.counter % (uint32_t) (READINGS_PER_SECOND * (ds.page == DevicePage ? 1 : 5)) == 0) {
        get_device_metrics(m);
        ds.dev = m.device;
    }

    if (ds.pause)
        ds.pause--;
    if (!ds.pause) {
        if (ds.counter % (uint32_t) (READINGS_PER_SECOND * m_settings.mqtt_period) == 0) {
            if (xQueueSend(m_telemetry_queue, &m, 0) != pdPASS) {
                ESP_LOGW(TAG, "Telemetry queue is full, dropping MqttMessage");
            }
        }
        //static constexpr const char* fmt = "\n"
        //    "\tv_rms           = %.1fV\n"
        //    "\ti_rms           = %.2fA\n"
        //    "\tapparent_power  = %.1fVA\n"
        //    "\treal_power      = %.1fW\n"
        //    "\tenergy          = %.2fWh\n"
        //    "\tpower_factor    = %.2f\n"
        //    "\tfrequency       = %.2fHz\n"
        //    "\tcpu0_load       = %.1f%%\n"
        //    "\tcpu1_load       = %.1f%%\n"
        //    "\tsample_shift    = %.1f\n"
        //    "";
        //if (0 /*ds.counter % 5 == 0*/) {
        //    char buf[1024];
        //    snprintf(buf, sizeof(buf), fmt, m.f_v, m.f_i, m.f_va, m.f_w, m.f_wh, m.f_pf, m.f_hz, ds.cpu0, ds.cpu1/*, f_sh */);
        //    ESP_LOGI(TAG, "%s", buf);
        //}
    }

    switch (ds.page) {
        case SplashScreen:
            if (ds.pause)
                return;
            ds.page = MainsPage;
            break;
        case MainsPage: {
            if (!ds.pause) {
                ds.clear_screen(false);
                snprintf(ds.lines[0].data(), sizeof(display_line_t), "V_rms : %.1fV", m.power.f_v);
                snprintf(ds.lines[1].data(), sizeof(display_line_t), "I_rms : %.2fA", m.power.f_i);
                snprintf(ds.lines[2].data(), sizeof(display_line_t), "F_pwr : %.1fVA", m.power.f_va);
                snprintf(ds.lines[3].data(), sizeof(display_line_t), "R_pwr : %.1fW", m.power.f_w);
                snprintf(ds.lines[4].data(), sizeof(display_line_t), "Enrgy : %.1fWh", m.power.f_wh);
                if (std::isnan(m.power.f_pf))
                    snprintf(ds.lines[5].data(), sizeof(display_line_t), "PF    : (n/a)");
                else
                    snprintf(ds.lines[5].data(), sizeof(display_line_t), "PF    : %.2f", m.power.f_pf);
                if (std::isnan(m.power.f_hz))
                    snprintf(ds.lines[6].data(), sizeof(display_line_t), "Freq  : (n/a)");
                else
                    snprintf(ds.lines[6].data(), sizeof(display_line_t), "Freq  : %.2fHz", m.power.f_hz);
                ds.inverse[3] = ds.inverse[0] || ds.inverse[1];
                ds.inverse[5] = ds.inverse[0] || ds.inverse[1];
            }
            break;
        }
        case DevicePage: {
            ds.clear_screen();
            std::tm local {};
            if (localtime_r(&m.time, &local)) {
                strftime(ds.lines[0].data(), sizeof(display_line_t), "%H:%M:%S", &local);
                strftime(ds.lines[1].data(), sizeof(display_line_t), "%a %d.%m.%Y", &local);
            }
            uint32_t secs = ds.dev.uptime;
            uint16_t days = secs / 86400;
            uint16_t hours = secs / 3600;
            uint16_t minutes = (secs % 3600) / 60;
            if (days < 1000) {
                snprintf(ds.lines[3].data(), sizeof(display_line_t), "Uptm : %03hu.%02hu:%02hu", days, hours, minutes);
            }
            else {
                snprintf(ds.lines[3].data(), sizeof(display_line_t), "Uptm : ---.--:--");
            }
            snprintf(ds.lines[4].data(), sizeof(display_line_t), "Cpu0 : %4.1f%%", ds.dev.cpu0);
            snprintf(ds.lines[5].data(), sizeof(display_line_t), "Cpu1 : %4.1f%%", ds.dev.cpu1);
            snprintf(ds.lines[6].data(), sizeof(display_line_t), "Heap : %4uK", ds.dev.heap / 1024);
            snprintf(ds.lines[7].data(), sizeof(display_line_t), "LBlk : %4uK", ds.dev.lblk / 1024);
            break;
        }
        case SettingsPage: {
            if (ds.lines[0][0] == 0x20) {
                snprintf(ds.lines[0].data(), sizeof(display_line_t), "--- Settings ---");
                if (ds.settings_unlocked) {
                    switch (ds.input_state) {
                        case PageSelection:
                        case MenuSelection:
                            for (uint8_t i = 0; i < N_DISPLAYED_ITEMS; ++i)
                                snprintf(ds.lines[i + 1].data(), sizeof(display_line_t), "%-16.16s", MENU_ITEMS[ds.first_displayed_menu_item + i]);
                            break;
                        case ItemDisplay:
                            assert(ds.item_selected != ItemNone);
                            snprintf(ds.lines[1].data(), sizeof(display_line_t), "%-16.16s", MENU_ITEMS[ds.item_selected]);
                            switch (ds.item_selected) {
                                case ItemWifiSsid: {
                                    snprintf(ds.lines[2].data(), sizeof(display_line_t), "%-16.16s", m_settings.wifi_ssid.data());
                                    break;
                                }
                                case ItemWifiPassword: {
                                    string_t pass {};
                                    std::fill_n(pass.begin(), std::min(sizeof(string_t), strlen((char*) m_settings.wifi_pass.data())), '*');
                                    snprintf(ds.lines[2].data(), sizeof(display_line_t), "%-16.16s", pass.data());
                                    break;
                                }
                                case ItemWifiInfo: {
                                    wifi_config_t wifi {};
                                    esp_wifi_get_config(WIFI_IF_STA, &wifi);

                                    esp_netif_ip_info_t ip_info {};
                                    esp_netif_get_ip_info(m_netif, &ip_info);

                                    snprintf(ds.lines[2].data(), sizeof(display_line_t), "SSID:%.11s", wifi.sta.ssid);
                                    snprintf(ds.lines[3].data(), sizeof(display_line_t), "Channel:%d", wifi.sta.channel);

                                    uint8_t a, b, c, d;
                                    a = ip_info.ip.addr >> 0;
                                    b = ip_info.ip.addr >> 8;
                                    c = ip_info.ip.addr >> 16;
                                    d = ip_info.ip.addr >> 24;
                                    snprintf(ds.lines[5].data(), sizeof(display_line_t), "%d.%d.%d.%d", a, b, c, d);

                                    a = ip_info.netmask.addr >> 0;
                                    b = ip_info.netmask.addr >> 8;
                                    c = ip_info.netmask.addr >> 16;
                                    d = ip_info.netmask.addr >> 24;
                                    snprintf(ds.lines[6].data(), sizeof(display_line_t), "%d.%d.%d.%d", a, b, c, d);

                                    a = ip_info.gw.addr >> 0;
                                    b = ip_info.gw.addr >> 8;
                                    c = ip_info.gw.addr >> 16;
                                    d = ip_info.gw.addr >> 24;
                                    snprintf(ds.lines[7].data(), sizeof(display_line_t), "%d.%d.%d.%d", a, b, c, d);
                                    break;
                                }
                                case ItemMqttUri: {
                                    if (strlen(m_settings.mqtt_uri.data()) == 0) {
                                        snprintf(ds.lines[2].data(), sizeof(display_line_t), "%s", "(empty)");
                                    }
                                    else for (int i = 0; i < 6; ++i) {
                                        const char* part = m_settings.mqtt_uri.data() + (i * 16);
                                        snprintf(ds.lines[i + 2].data(), sizeof(display_line_t), "%-16.16s", part);
                                    }
                                    break;
                                }
                                case ItemMqttCreds: {
                                    if (strlen(m_settings.mqtt_creds.data()) == 0) {
                                        snprintf(ds.lines[2].data(), sizeof(display_line_t), "%s", "(empty)");
                                    }
                                    else for (int i = 0; i < 6; ++i) {
                                        const char* part = m_settings.mqtt_creds.data() + (i * 16);
                                        snprintf(ds.lines[i + 2].data(), sizeof(display_line_t), "%-16.16s", part);
                                    }
                                    break;
                                }
                                case ItemMqttTopic: {
                                    if (strlen(m_settings.mqtt_topic.data()) == 0) {
                                        snprintf(ds.lines[2].data(), sizeof(display_line_t), "%s", "(empty)");
                                    }
                                    else for (int i = 0; i < 6; ++i) {
                                        const char* part = m_settings.mqtt_topic.data() + (i * 16);
                                        snprintf(ds.lines[i + 2].data(), sizeof(display_line_t), "%-16.16s", part);
                                    }
                                    break;
                                }
                                case ItemMqttPeriod: {
                                    snprintf(ds.lines[2].data(), sizeof(display_line_t), "%f sec", m_settings.mqtt_period);
                                    break;
                                }
                                case ItemINoiseFloor: {
                                    snprintf(ds.lines[2].data(), sizeof(display_line_t), "%f A", m_settings.i_noise_floor);
                                    break;
                                }
                                case ItemICalib: {
                                    snprintf(ds.lines[2].data(), sizeof(display_line_t), "%f", m_settings.i_coef);
                                    break;
                                }
                                case ItemVCalib: {
                                    snprintf(ds.lines[2].data(), sizeof(display_line_t), "%f", m_settings.v_coef);
                                    break;
                                }
                                case ItemVEnergySave: {
                                    snprintf(ds.lines[2].data(), sizeof(display_line_t), "%f V", m_settings.v_acc_energy_save);
                                    break;
                                }
                                case ItemResetEnrgy: {
                                    snprintf(ds.lines[2].data(), sizeof(display_line_t), "Push to reset...");
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
                        default:
                            break;
                    }
                }
                else {
                    if (ds.input_state == PageSelection) {
                        snprintf(ds.lines[1].data(), sizeof(display_line_t), "Enter PIN code:");
                        snprintf(ds.lines[3].data(), sizeof(display_line_t), "[Push to start]");
                    }
                    else if (ds.input_state == PinInput) {
                        snprintf(ds.lines[1].data(), sizeof(display_line_t), "Enter PIN code:");

                        // Dynamically build the input string: e.g., "[ * * 3 _ ]"
                        char pin_str[16] = "[ _ _ _ _ ]";
                        for (int i = 0; i < 4; i++) {
                            int char_idx = 2 + (i * 2); // Map array to string spaces
                            if (i < ds.pin_index) {
                                pin_str[char_idx] = '*'; // Past digits
                            }
                            else if (i == ds.pin_index) {
                                pin_str[char_idx] = '0' + ds.pin_digits[i]; // Current digit
                            }
                        }
                        snprintf(ds.lines[3].data(), sizeof(display_line_t), "%s", pin_str);
                    }
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
                snprintf(ds.lines[1].data(), sizeof(display_line_t), "PW-Meter");
                snprintf(ds.lines[3].data(), sizeof(display_line_t), "Ver: %.11s", app->version);
                snprintf(ds.lines[4].data(), sizeof(display_line_t), "IDF: %.11s", app->idf_ver);
                snprintf(ds.lines[6].data(), sizeof(display_line_t), "Copyrt (c) 2026");
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
                inverse = (i > 0) && (i == ds.item_selected - ds.first_displayed_menu_item + 1);
            }
        }
        if (ds.with_display && ds.is_display_awake) {
            ssd1306_display_text(&m_oled, i, ds.lines[i].data(), sizeof(display_line_t) - 1, inverse);
        }
    }
}

void PowerMeterApp::process_encoder_input(const EncoderInputMessage& encoder)
{
    switch (encoder.action) {
        case EncoderInputAction::Next:
        case EncoderInputAction::Prev:
            switch (ds.input_state) {
                case PageSelection:
                    ds.pause = 0;
                    if (encoder.action == EncoderInputAction::Next) {
                        ds.set_next_page();
                    }
                    else {
                        ds.set_prev_page();
                    }
                    ds.clear_screen();
                    break;
                case MenuSelection:
                    if (encoder.action == EncoderInputAction::Next) {
                        ds.set_next_item();
                    }
                    else {
                        ds.set_prev_item();
                    }
                    break;
                case PinInput:
                    // Rotate to change the current digit (0-9 with wrap-around)
                    if (encoder.action == EncoderInputAction::Next) {
                        ds.pin_digits[ds.pin_index] = (ds.pin_digits[ds.pin_index] + 1) % 10;
                    }
                    else {
                        ds.pin_digits[ds.pin_index] = (ds.pin_digits[ds.pin_index] + 9) % 10;
                    }
                    ds.clear_screen();
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
                            ds.pause = READINGS_PER_SECOND * 10;
                            ESP_LOGI(TAG, "Paused for 10 sec");
                        }
                    }
                    else if (ds.page == SettingsPage) {
                        if (ds.settings_unlocked) {
                            ds.input_state = MenuSelection;
                            ds.item_selected = ItemDefault;
                        }
                        else {
                            ds.input_state = PinInput;
                            ds.pin_index = 0;
                            ds.pin_digits.fill(0);
                        }
                        ds.clear_screen();
                    }
                    break;
                case PinInput:
                    // Confirm the current digit and move to the next
                    ds.pin_index++;
                    if (ds.pin_index >= 4) {
                        // Validate PIN
                        if (ds.pin_digits == SETTINGS_PIN) {
                            ds.settings_unlocked = true;
                            ds.input_state = MenuSelection;
                            ds.item_selected = ItemDefault;
                            ESP_LOGI(TAG, "Settings unlocked.");
                        }
                        else {
                            // Failed: Kick back to prompt
                            ds.input_state = PageSelection;
                            ESP_LOGW(TAG, "Invalid PIN entered.");
                        }
                    }
                    ds.clear_screen();
                    break;
                case MenuSelection:
                    ds.clear_screen();
                    ds.input_state = ItemDisplay;
                    break;
                case ItemDisplay:
                    ds.clear_screen();
                    ds.pause = UINT32_MAX;
                    switch (ds.item_selected) {
                        case ItemWifiSsid: {
                            string_t prompt { "Input new Wi-Fi network name:" };
                            esp_event_post_to(m_console_event_loop, EVENT_CONSOLE_INPUT, STRING_INPUT_ID, &prompt, sizeof(prompt), pdMS_TO_TICKS(100));
                            ds.input_state = ValueInput;
                            break;
                        }
                        case ItemWifiPassword: {
                            string_t prompt { "Input new Wi-Fi password:" };
                            esp_event_post_to(m_console_event_loop, EVENT_CONSOLE_INPUT, STRING_INPUT_ID, &prompt, sizeof(prompt), pdMS_TO_TICKS(100));
                            ds.input_state = ValueInput;
                            break;
                        }
                        case ItemWifiInfo: {
                            ds.input_state = MenuSelection;
                            ds.pause = 0;
                            break;
                        }
                        case ItemMqttUri: {
                            string_t prompt { "Input new MQTT broker URI:" };
                            esp_event_post_to(m_console_event_loop, EVENT_CONSOLE_INPUT, STRING_INPUT_ID, &prompt, sizeof(prompt), pdMS_TO_TICKS(100));
                            ds.input_state = ValueInput;
                            break;
                        }
                        case ItemMqttCreds: {
                            string_t prompt { "Input new MQTT creds (use ':' as a separator, like \"username:password\")" };
                            esp_event_post_to(m_console_event_loop, EVENT_CONSOLE_INPUT, STRING_INPUT_ID, &prompt, sizeof(prompt), pdMS_TO_TICKS(100));
                            ds.input_state = ValueInput;
                            break;
                        }
                        case ItemMqttTopic: {
                            string_t prompt { "Input new MQTT topic name:" };
                            esp_event_post_to(m_console_event_loop, EVENT_CONSOLE_INPUT, STRING_INPUT_ID, &prompt, sizeof(prompt), pdMS_TO_TICKS(100));
                            ds.input_state = ValueInput;
                            break;
                        }
                        case ItemMqttPeriod: {
                            string_t prompt { "Input new MQTT period in seconds:" };
                            esp_event_post_to(m_console_event_loop, EVENT_CONSOLE_INPUT, FLOAT_INPUT_ID, &prompt, sizeof(prompt), pdMS_TO_TICKS(100));
                            ds.input_state = ValueInput;
                            break;
                        }
                        case ItemINoiseFloor: {
                            string_t prompt { "Input new I noise floor:" };
                            esp_event_post_to(m_console_event_loop, EVENT_CONSOLE_INPUT, FLOAT_INPUT_ID, &prompt, sizeof(prompt), pdMS_TO_TICKS(100));
                            ds.input_state = ValueInput;
                            break;
                        }
                        case ItemICalib: {
                            string_t prompt { "Input new I calibration:" };
                            esp_event_post_to(m_console_event_loop, EVENT_CONSOLE_INPUT, FLOAT_INPUT_ID, &prompt, sizeof(prompt), pdMS_TO_TICKS(100));
                            ds.input_state = ValueInput;
                            break;
                        }
                        case ItemVCalib: {
                            string_t prompt { "Input new V calibration:" };
                            esp_event_post_to(m_console_event_loop, EVENT_CONSOLE_INPUT, FLOAT_INPUT_ID, &prompt, sizeof(prompt), pdMS_TO_TICKS(100));
                            ds.input_state = ValueInput;
                            break;
                        }
                        case ItemVEnergySave: {
                            string_t prompt { "Input new V threshold to save acc energy counter in NVS:" };
                            esp_event_post_to(m_console_event_loop, EVENT_CONSOLE_INPUT, FLOAT_INPUT_ID, &prompt, sizeof(prompt), pdMS_TO_TICKS(100));
                            ds.input_state = ValueInput;
                            break;
                        }
                        case ItemResetEnrgy: {
                            ESP_LOGI(TAG, "Accumulated energy counter was reset");
                            SettingsManager::save(KEY_ACC_ENERGY, m_acc_energy = 0.0f);
                            ds.input_state = MenuSelection;
                            ds.pause = 0;
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
                    //ESP_LOGI(TAG, "Restarting...");
                    //stop_tasks();
                    ds.page = MainsPage;
                    break;
                case PinInput:
                    // Long press cancels PIN entry
                    ds.clear_screen();
                    ds.input_state = PageSelection;
                    break;
                case MenuSelection:
                    ds.clear_screen();
                    ds.input_state = PageSelection;
                    ds.item_selected = ItemNone;
                    ds.first_displayed_menu_item = 0;
                    break;
                case ItemDisplay:
                    ds.clear_screen();
                    ds.input_state = MenuSelection;
                    break;
                case ValueInput:
                    ds.clear_screen();
                    ds.input_state = ItemDisplay;
                    ds.pause = 0;
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
                break;
            case ConsoleInputAction::Float:
                switch (ds.item_selected) {
                    case ItemMqttPeriod:
                        if (input.float_value < 1 || input.float_value > 255) {
                            ESP_LOGW(TAG, "New MQTT period value is invalid ([1..255]): %f", input.float_value);
                        }
                        else {
                            ESP_LOGI(TAG, "Setting new MQTT period: %f", input.float_value);
                            SettingsManager::save(KEY_MQTT_PERIOD, m_settings.mqtt_period = input.float_value);
                        }
                        break;
                    case ItemINoiseFloor:
                        ESP_LOGI(TAG, "Setting new I noise floor: %f", input.float_value);
                        SettingsManager::save(KEY_I_NOISE_FLOOR, m_settings.i_noise_floor = input.float_value);
                        break;
                    case ItemICalib:
                        ESP_LOGW(TAG, "Setting new I calibration: %f", input.float_value);
                        SettingsManager::save(KEY_I_COEF, m_settings.i_coef = input.float_value);
                        break;
                    case ItemVCalib:
                        ESP_LOGW(TAG, "Setting new V calibration: %f", input.float_value);
                        SettingsManager::save(KEY_V_COEF, m_settings.v_coef = input.float_value);
                        break;
                    case ItemVEnergySave:
                        ESP_LOGW(TAG, "Setting new V threshold to save acc energy counter in NVS: %f", input.float_value);
                        SettingsManager::save(KEY_V_ACC_ENERGY_SAVE, m_settings.v_acc_energy_save = input.float_value);
                        break;
                    default:
                        break;
                }
                break;
            case ConsoleInputAction::String:
                switch (ds.item_selected) {
                    case ItemWifiSsid:
                        set_wifi_ssid(input.string_value);
                        break;
                    case ItemWifiPassword:
                        set_wifi_pass(input.string_value);
                        break;
                    case ItemMqttUri:
                        set_mqtt_uri(input.string_value);
                        break;
                    case ItemMqttCreds:
                        set_mqtt_creds(input.string_value);
                        break;
                    case ItemMqttTopic:
                        set_mqtt_topic(input.string_value);
                        break;
                    default:
                        break;
                }
                break;
            case ConsoleInputAction::None:
                ESP_LOGI(TAG, "Console input cancelled");
                break;
        }
        ds.input_state = ItemDisplay;
        ds.pause = 0;
        ds.clear_screen();
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

void PowerMeterApp::get_device_metrics(MqttMessage& m)
{
    m_cpu_meter.get_load(m.device.cpu0, m.device.cpu1);
    m.device.uptime = m.time - static_cast<timeval>(s_start_time).tv_sec;
    m.device.heap = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    m.device.lblk = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
}
