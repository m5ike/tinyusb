// SPDX-License-Identifier: MIT
// Target: ESP32-S2 / ESP32-S3 (native USB OTG)
// Toolchain: ESP-IDF v5.x

#include <cstring>
#include <string>
#include <vector>
#include <cstdio>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_event.h"   // <- you were missing the closing quote

// byteswap shim (IDF has no <byteswap.h>)
#ifndef __bswap_16
  #define __bswap_16 __builtin_bswap16
#endif
#ifndef __bswap_32
  #define __bswap_32 __builtin_bswap32
#endif
#ifndef __bswap_64
  #define __bswap_64 __builtin_bswap64
#endif

#define UNUSED(x) (void)(x)
// ...
#include "usb/usb_host.h"

static bool finished [[maybe_unused]] = false;
static bool is_ready [[maybe_unused]] = false;
static uint32_t last_seqnum [[maybe_unused]] = 0;
static usb_transfer_t* _transfer [[maybe_unused]] = nullptr;

// Delete the separate UNUSED(...) lines entirely.

#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_netif_types.h"
#include <inttypes.h>
#include <cinttypes>
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "usb/usb_host.h"      // for usb_host_config_t, usb_host_* APIs

#include "driver/gpio.h"

// TinyUSB CDC (ESP-IDF component: tinyusb)
//#include "tinyusb.h"
#include "esp_timer.h"

#if __has_include("tinyusb.h")
  #include "tinyusb.h"
#elif __has_include("tinyusb/tinyusb.h")
  #include "tinyusb/tinyusb.h"
#else
  // Leave this as a warning so the compiler can still find tusb_cdc_acm.h below if present
  #warning "esp_tinyusb.h not found"
#endif

#include "esp_check.h"        // ESP_RETURN_ON_ERROR
#include "usb/usb_host.h"     // usb_transfer_t

extern "C" {
#include "tusb_cdc_acm.h"
}

#include "class/cdc/cdc_device.h"

static void client_event_callback(const usb_host_client_event_msg_t *msg, void *arg)
{
    (void)msg; (void)arg;
}
static usb_host_client_handle_t client_hdl = nullptr;

// handle pro CDC (typ je z tusb_cdc_acm.h)
static tusb_cdcacm_handle_t g_cdc;

static const char* TAG = "APP";

// -------- Pins (adjust for your board) --------
#define BUTTON_PIN  GPIO_NUM_0        // BOOT button
#define LED_PIN     GPIO_NUM_2        // Onboard LED (or choose valid output)

// -------- Mode handling --------
enum class OpMode : uint8_t {
    SETUP_CONSOLE = 1,  // USB-C as CDC console
    USB_HID_OTG   = 2,  // USB-C as HID/OTG normal operation
    HTTP_CONSOLE  = 3   // Normal + HTTP server console
};

static volatile OpMode g_mode = OpMode::SETUP_CONSOLE;
static QueueHandle_t g_btn_evt_q;

// LED blink parameters
static constexpr TickType_t LED_ON_MS  = 700 / portTICK_PERIOD_MS;
static constexpr TickType_t LED_OFF_MS = 700 / portTICK_PERIOD_MS;

// -------- Config persisted to NVS --------
struct NetConfig {
    bool dhcp = true;                        // true=DHCP, false=manual
    char ssid[33] = {0};
    char pass[65] = {0};
    char ipv4_addr[16] = {0};
    char ipv4_gw[16]   = {0};
    char ipv4_dns[16]  = {0};
};

static NetConfig g_cfg;

static esp_err_t cfg_load(NetConfig& out) {
    nvs_handle_t h;
    esp_err_t err = nvs_open("netcfg", NVS_READONLY, &h);
    if (err != ESP_OK) return err;

    size_t len;
    uint8_t dhcp = 1;
    if (nvs_get_u8(h, "dhcp", &dhcp) == ESP_OK) out.dhcp = (dhcp != 0);

    len = sizeof(out.ssid); nvs_get_str(h, "ssid", out.ssid, &len);
    len = sizeof(out.pass); nvs_get_str(h, "pass", out.pass, &len);
    len = sizeof(out.ipv4_addr); nvs_get_str(h, "ip", out.ipv4_addr, &len);
    len = sizeof(out.ipv4_gw);  nvs_get_str(h, "gw", out.ipv4_gw, &len);
    len = sizeof(out.ipv4_dns); nvs_get_str(h, "dns", out.ipv4_dns, &len);

    nvs_close(h);
    return ESP_OK;
}

static esp_err_t cfg_save(const NetConfig& in) {
    nvs_handle_t h;
    ESP_RETURN_ON_ERROR(nvs_open("netcfg", NVS_READWRITE, &h), TAG, "nvs_open");
    ESP_RETURN_ON_ERROR(nvs_set_u8(h, "dhcp", in.dhcp ? 1 : 0), TAG, "set dhcp");
    ESP_RETURN_ON_ERROR(nvs_set_str(h, "ssid", in.ssid), TAG, "set ssid");
    ESP_RETURN_ON_ERROR(nvs_set_str(h, "pass", in.pass), TAG, "set pass");
    ESP_RETURN_ON_ERROR(nvs_set_str(h, "ip",   in.ipv4_addr), TAG, "set ip");
    ESP_RETURN_ON_ERROR(nvs_set_str(h, "gw",   in.ipv4_gw),   TAG, "set gw");
    ESP_RETURN_ON_ERROR(nvs_set_str(h, "dns",  in.ipv4_dns),  TAG, "set dns");
    ESP_RETURN_ON_ERROR(nvs_commit(h), TAG, "commit");
    nvs_close(h);
    return ESP_OK;
}

static void cfg_print(const NetConfig& c, FILE* out = stdout) {
    fprintf(out, "mode: %s\n", c.dhcp ? "dhcp" : "manual");
    fprintf(out, "ssid: %s\n", c.ssid[0] ? c.ssid : "(empty)");
    fprintf(out, "pass: %s\n", c.pass[0] ? "******" : "(empty)");
    fprintf(out, "ipv4_addr: %s\n", c.ipv4_addr[0] ? c.ipv4_addr : "(unset)");
    fprintf(out, "ipv4_gw:   %s\n", c.ipv4_gw[0]   ? c.ipv4_gw   : "(unset)");
    fprintf(out, "ipv4_dns:  %s\n", c.ipv4_dns[0]  ? c.ipv4_dns  : "(unset)");
}

// -------- Button ISR / debounce --------
static void IRAM_ATTR btn_isr(void* arg) {
    BaseType_t hpw = pdFALSE;
    uint32_t evt = 1;
    if (g_btn_evt_q) xQueueSendFromISR(g_btn_evt_q, &evt, &hpw);
    if (hpw == pdTRUE) portYIELD_FROM_ISR();
}

static void button_init() {
    gpio_config_t io = {};
    io.pin_bit_mask = 1ULL << BUTTON_PIN;
    io.mode = GPIO_MODE_INPUT;
    io.pull_up_en = GPIO_PULLUP_ENABLE;    // BOOT is usually active-low
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_NEGEDGE;      // falling edge when pressed
    ESP_ERROR_CHECK(gpio_config(&io));

    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(BUTTON_PIN, btn_isr, nullptr));
}

// -------- LED annunciator task --------
static void led_init() {
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN, 0);
}

static void led_flash_count(uint8_t count) {
    for (uint8_t i = 0; i < count; ++i) {
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(LED_ON_MS);
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(LED_OFF_MS);
    }
}

// -------- TinyUSB CDC console (Setup mode) --------
#if CONFIG_TINYUSB_CDC_ENABLED  // or CONFIG_TINYUSB_DEVICE_CDC
// tusb_cdcacm_handle_t g_cdc;  // etc…
    static tusb_cdcacm_handle_t g_cdc;

#endif

// minimal line buffer
static std::string read_line_cdc() {
    std::string line;
    uint8_t ch;
    size_t got = 0;
    while (true) {
        if (tusb_cdc_acm_available(g_cdc)) {
            if (tusb_cdc_acm_read(g_cdc, &ch, 1, &got) == ESP_OK && got == 1) {
                if (ch == '\r') continue;
                if (ch == '\n') break;
                line.push_back((char)ch);
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    return line;
}

static void cdc_write(const char* s) {
    size_t len = strlen(s);
    size_t written = 0;
    tusb_cdc_acm_write(g_cdc, (uint8_t*)s, len, &written);
    tinyusb_cdcacm_write_flush(g_cdc, 0);  // 0 = default timeout
}

static void cdc_writeln(const char* s) {
    cdc_write(s);
    cdc_write("\r\n");
}

static void cmd_show() {
    // print to CDC
    char buf[256];
    snprintf(buf, sizeof(buf),
        "mode: %s\r\nssid: %s\r\npass: %s\r\nipv4_addr: %s\r\nipv4_gw: %s\r\nipv4_dns: %s\r\n",
        g_cfg.dhcp ? "dhcp" : "manual",
        g_cfg.ssid[0] ? g_cfg.ssid : "(empty)",
        g_cfg.pass[0] ? "******" : "(empty)",
        g_cfg.ipv4_addr[0] ? g_cfg.ipv4_addr : "(unset)",
        g_cfg.ipv4_gw[0]   ? g_cfg.ipv4_gw   : "(unset)",
        g_cfg.ipv4_dns[0]  ? g_cfg.ipv4_dns  : "(unset)");
    cdc_write(buf);
}

static void parse_kv(const std::string& ln, std::string& k, std::string& v) {
    auto sp = ln.find(' ');
    if (sp == std::string::npos) { k = ln; v.clear(); return; }
    k = ln.substr(0, sp);
    // skip spaces
    size_t i = sp;
    while (i < ln.size() && ln[i] == ' ') i++;
    v = ln.substr(i);
}

static void console_setup_mode() {
    cdc_writeln("\r\n-- SETUP CONSOLE --");
    cdc_writeln("Commands: ssid [val], pass [val], mode [manual|dhcp],");
    cdc_writeln("ipv4_addr [x.x.x.x], ipv4_gw [x.x.x.x], ipv4_dns [x.x.x.x],");
    cdc_writeln("save, show, exit");

    while (g_mode == OpMode::SETUP_CONSOLE) {
        if (!tud_cdc_n_connected(0)) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }

        cdc_write("\r\n> ");
        std::string ln = read_line_cdc();
        std::string cmd, val;
        parse_kv(ln, cmd, val);

        if (cmd == "ssid") {
            if (val.empty()) { cdc_writeln(g_cfg.ssid[0] ? g_cfg.ssid : "(empty)"); }
            else { strncpy(g_cfg.ssid, val.c_str(), sizeof(g_cfg.ssid)-1); cdc_writeln("OK"); }
        } else if (cmd == "pass") {
            if (val.empty()) { cdc_writeln(g_cfg.pass[0] ? "******" : "(empty)"); }
            else { strncpy(g_cfg.pass, val.c_str(), sizeof(g_cfg.pass)-1); cdc_writeln("OK"); }
        } else if (cmd == "mode") {
            if (val == "dhcp") { g_cfg.dhcp = true; cdc_writeln("OK (dhcp)"); }
            else if (val == "manual") { g_cfg.dhcp = false; cdc_writeln("OK (manual)"); }
            else { cdc_writeln("ERR: use manual|dhcp"); }
        } else if (cmd == "ipv4_addr") {
            if (val.empty()) cdc_writeln(g_cfg.ipv4_addr[0] ? g_cfg.ipv4_addr : "(unset)");
            else { strncpy(g_cfg.ipv4_addr, val.c_str(), sizeof(g_cfg.ipv4_addr)-1); cdc_writeln("OK"); }
        } else if (cmd == "ipv4_gw") {
            if (val.empty()) cdc_writeln(g_cfg.ipv4_gw[0] ? g_cfg.ipv4_gw : "(unset)");
            else { strncpy(g_cfg.ipv4_gw, val.c_str(), sizeof(g_cfg.ipv4_gw)-1); cdc_writeln("OK"); }
        } else if (cmd == "ipv4_dns") {
            if (val.empty()) cdc_writeln(g_cfg.ipv4_dns[0] ? g_cfg.ipv4_dns : "(unset)");
            else { strncpy(g_cfg.ipv4_dns, val.c_str(), sizeof(g_cfg.ipv4_dns)-1); cdc_writeln("OK"); }
        } else if (cmd == "save") {
            if (cfg_save(g_cfg) == ESP_OK) cdc_writeln("SAVED");
            else cdc_writeln("ERR: NVS save failed");
        } else if (cmd == "show") {
            cmd_show();
        } else if (cmd == "exit") {
            cdc_writeln("Exiting setup mode...");
            break;
        } else if (!cmd.empty()) {
            cdc_writeln("ERR: unknown command");
        }

        // allow cooperative switch by button while in loop
        taskYIELD();
    }
}

// -------- TinyUSB init (CDC-ACM) --------
static void tinyusb_init_cdc() {
    // Install TinyUSB driver via esp_tinyusb
    tinyusb_config_t tusb_cfg = {};
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    // Bring up a CDC-ACM instance (uses interface 0 by default)
    tusb_cdc_acm_config_t cdc_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = 256,
        .callback_rx = nullptr,
        .callback_rx_wanted_char = nullptr,
        .callback_line_state_changed = nullptr,
        .callback_line_coding_changed = nullptr,
    };
    ESP_ERROR_CHECK(tusb_cdc_acm_init(&cdc_cfg, &g_cdc));
    ESP_LOGI(TAG, "CDC-ACM ready");
}

// -------- Mode services (stubs to fill) --------
static void start_usb_hid_mode() {
    // TODO: Initialize your HID/USBIP stack here
    ESP_LOGI(TAG, "USB HID/OTG mode START");
}
static void stop_usb_hid_mode() {
    // TODO: Deinit HID stack
    ESP_LOGI(TAG, "USB HID/OTG mode STOP");
}
static void start_http_console() {
    // TODO: Start HTTP server, expose logs/commands/control of usbip
    ESP_LOGI(TAG, "HTTP console START");
}
static void stop_http_console() {
    // TODO: Stop HTTP server
    ESP_LOGI(TAG, "HTTP console STOP");
}

// -------- Tasks --------
static void task_led_annunciator(void*) {
    OpMode last = g_mode;
    // Announce initial mode
    led_flash_count(static_cast<uint8_t>(g_mode));
    while (true) {
        if (last != g_mode) {
            last = g_mode;
            led_flash_count(static_cast<uint8_t>(g_mode));
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void task_button_monitor(void*) {
    // simple debounce and mode cycle on press
    uint64_t last = 0;
    uint32_t evt;
    while (true) {
        if (xQueueReceive(g_btn_evt_q, &evt, portMAX_DELAY) == pdTRUE) {
            uint64_t now = esp_timer_get_time();
            if (now - last > 200000) { // 200 ms
                last = now;
                uint8_t m = static_cast<uint8_t>(g_mode);
                m++;
                if (m > 3) m = 1;
                g_mode = static_cast<OpMode>(m);
                ESP_LOGI(TAG, "Mode -> %u", (unsigned)m);
            }
        }
    }
}

static void task_mode_runner(void*) {
    OpMode current = OpMode::SETUP_CONSOLE;

    // initial: Setup console on USB
    tinyusb_init_cdc();

    while (true) {
        current = g_mode;

        switch (current) {
            case OpMode::SETUP_CONSOLE:
                // ensure other services are off
                stop_usb_hid_mode();
                stop_http_console();
                console_setup_mode();   // blocking loop until exit or mode change
                // after exit, drop into whatever g_mode is
                break;

            case OpMode::USB_HID_OTG:
                // ensure HTTP off, CDC can still be enumerated but ignored
                stop_http_console();
                start_usb_hid_mode();
                // idle loop until mode changes
                while (g_mode == OpMode::USB_HID_OTG) {
                    vTaskDelay(pdMS_TO_TICKS(50));
                }
                stop_usb_hid_mode();
                break;

            case OpMode::HTTP_CONSOLE:
                // ensure HID off
                stop_usb_hid_mode();
                start_http_console();
                while (g_mode == OpMode::HTTP_CONSOLE) {
                    vTaskDelay(pdMS_TO_TICKS(50));
                }
                stop_http_console();
                break;
        }

        // loop will re-evaluate g_mode and launch appropriate service
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void usb_host_start(void) {
    usb_host_config_t host_cfg = {};
    host_cfg.intr_flags = ESP_INTR_FLAG_LEVEL1;
    host_cfg.skip_phy_setup = false;
    host_cfg.root_port_unpowered = false;
    host_cfg.enum_filter_cb = NULL;
    ESP_ERROR_CHECK(usb_host_install(&host_cfg));

    usb_host_client_config_t cli_cfg = {};
    cli_cfg.is_synchronous = false;
    cli_cfg.max_num_event_msg = 8;
    cli_cfg.async.client_event_callback = client_event_callback; // your cb
    cli_cfg.async.callback_arg = NULL;
    ESP_ERROR_CHECK(usb_host_client_register(&cli_cfg, &client_hdl));
}
void wifi_start_station(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t sta = { 0 };
    // fill from your NVS config
    strlcpy((char*)sta.sta.ssid, g_cfg.ssid, sizeof(sta.sta.ssid));
    strlcpy((char*)sta.sta.password, g_cfg.pass, sizeof(sta.sta.password));
    sta.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta));
    ESP_ERROR_CHECK(esp_wifi_start());
}

// -------- app_main --------
extern "C" void app_main(void) {
    esp_log_level_set("*", ESP_LOG_INFO);

    ESP_ERROR_CHECK(nvs_flash_init());
    // Load saved config if present
    cfg_load(g_cfg);

    // GPIO
    led_init();
    button_init();

    // Button event queue
    g_btn_evt_q = xQueueCreate(4, sizeof(uint32_t));

    // Tasks
    xTaskCreatePinnedToCore(task_led_annunciator, "led_ann", 2048, nullptr, 4, nullptr, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(task_button_monitor,   "btn_mon", 2048, nullptr, 5, nullptr, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(task_mode_runner,      "mode_run", 4096, nullptr, 5, nullptr, tskNO_AFFINITY);

    ESP_LOGI(TAG, "Started. Press BOOT to cycle modes 1/2/3. LED blinks count each switch.");
}
