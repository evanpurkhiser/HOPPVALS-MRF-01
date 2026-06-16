#include "hv-mrf-01/http_debug.hpp"

#include <algorithm>
#include <cstring>
#include <string>

#include "esp_check.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "hv-mrf-01/config.hpp"
#include "hv-mrf-01/console.hpp"
#include "hv-mrf-01/motion.hpp"
#include "hv-mrf-01/motor.hpp"

namespace hvmrf01::http_debug {

namespace {

constexpr auto* TAG = "hv-mrf-01.debug";

// Signalled by the WiFi event handler once the station has an IP.
EventGroupHandle_t   wifi_events = nullptr;
constexpr EventBits_t GOT_IP_BIT = BIT0;

// Dotted-quad address acquired in debug mode, captured for the ready banner.
char acquired_ip[16] = {};

void on_wifi_event(void*, esp_event_base_t base, std::int32_t id, void* data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        // Keep retrying; the overall connect deadline is enforced by run().
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const auto* event = static_cast<ip_event_got_ip_t*>(data);
        std::snprintf(acquired_ip, sizeof(acquired_ip), IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "got IP %s", acquired_ip);
        xEventGroupSetBits(wifi_events, GOT_IP_BIT);
    }
}

// Initialize the WiFi station and begin association. Returns an esp_err_t so
// failures propagate to run() and become a reboot-to-normal rather than an
// abort() that could crash-loop an installed, USB-less device.
esp_err_t wifi_init_sta(const config::Network& net)
{
    wifi_events = xEventGroupCreate();
    ESP_RETURN_ON_FALSE(wifi_events != nullptr, ESP_ERR_NO_MEM, TAG, "event group");

    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "netif init");
    esp_netif_t* sta_netif = esp_netif_create_default_wifi_sta();
    // DHCP hostname so the device shows up by name in the router/DNS lease
    // table instead of the default "espressif". Set before DHCP runs.
    static_cast<void>(esp_netif_set_hostname(sta_netif, "hv-mrf-01"));

    const wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init), TAG, "wifi init");

    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &on_wifi_event, nullptr, nullptr), TAG, "wifi evt");
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &on_wifi_event, nullptr, nullptr), TAG, "ip evt");

    // cfg is zero-initialized, so a memcpy of at most the field size leaves the
    // remaining bytes as NUL terminators (the 802.11 SSID field isn't required
    // to be terminated, but esp_wifi treats a shorter value as a C string).
    wifi_config_t cfg{};
    std::memcpy(cfg.sta.ssid, net.ssid,
                std::min(std::strlen(net.ssid), sizeof(cfg.sta.ssid)));
    std::memcpy(cfg.sta.password, net.pass,
                std::min(std::strlen(net.pass), sizeof(cfg.sta.password)));

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "set mode");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &cfg), TAG, "set config");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "wifi start");
    return ESP_OK;
}

// Bring up WiFi STA and block until associated-with-IP or the timeout expires.
bool wifi_connect(const config::Network& net)
{
    if (wifi_init_sta(net) != ESP_OK) {
        return false;
    }

    ESP_LOGI(TAG, "joining '%s' (timeout %ds)", net.ssid, net.connect_timeout_s);
    const auto bits = xEventGroupWaitBits(wifi_events, GOT_IP_BIT, pdFALSE, pdFALSE,
                                          pdMS_TO_TICKS(net.connect_timeout_s * 1000));
    return (bits & GOT_IP_BIT) != 0;
}

// Websocket handler: one text frame in (a command line), one text frame out
// (the command's captured output).
esp_err_t ws_handler(httpd_req_t* req)
{
    if (req->method == HTTP_GET) {
        // Opening handshake — completed by the server, nothing to do.
        return ESP_OK;
    }

    httpd_ws_frame_t frame{};
    frame.type = HTTPD_WS_TYPE_TEXT;

    // First call with len 0 just fills in frame.len.
    esp_err_t ret = httpd_ws_recv_frame(req, &frame, 0);
    if (ret != ESP_OK || frame.len == 0) {
        return ret;
    }

    std::string line(frame.len, '\0');
    frame.payload = reinterpret_cast<std::uint8_t*>(line.data());
    ret = httpd_ws_recv_frame(req, &frame, frame.len);
    if (ret != ESP_OK) {
        return ret;
    }

    std::string out = console::run_line(line);

    httpd_ws_frame_t reply{};
    reply.type    = HTTPD_WS_TYPE_TEXT;
    reply.payload = reinterpret_cast<std::uint8_t*>(out.data());
    reply.len     = out.size();
    return httpd_ws_send_frame(req, &reply);
}

// True if the request's query string contains `key` (value ignored).
bool has_query_key(httpd_req_t* req, const char* key)
{
    const size_t qlen = httpd_req_get_url_query_len(req) + 1;
    if (qlen <= 1) {
        return false;
    }
    std::string query(qlen, '\0');
    if (httpd_req_get_url_query_str(req, query.data(), qlen) != ESP_OK) {
        return false;
    }
    char value[8];
    return httpd_query_key_value(query.c_str(), key, value, sizeof(value)) == ESP_OK;
}

// Firmware push: stream the POST body straight into the inactive OTA slot via
// the standard esp_ota_ops flow, set it as the boot partition, and reboot.
//   curl --data-binary @build/hv-mrf-01.bin http://<ip>/ota
// Re-arms debug mode by default so the device comes back on WiFi for the next
// push; POST /ota?normal=1 boots into normal Zigbee mode instead.
// Reboot from a detached task so the /ota handler can return and httpd can
// finish flushing the response first — a synchronous reset in the handler can
// surface to the client as a connection reset even on a successful flash.
void deferred_reboot(void*)
{
    vTaskDelay(pdMS_TO_TICKS(750));
    esp_restart();
}

esp_err_t ota_handler(httpd_req_t* req)
{
    if (req->content_len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "empty body / missing Content-Length");
        return ESP_FAIL;
    }

    // De-energize the motors for the duration of the write + reboot: halt the
    // control loop so it stops issuing drives, then pull nSLEEP low so the
    // drivers can't run regardless of what the incoming image does on boot.
    motion::stop();
    motor::disable();

    // Confirm the running image before begin: esp_ota_begin refuses to run from
    // an unverified app, so without this a device sitting in debug mode on a
    // freshly-pushed (still pending-verify) image couldn't be flashed again.
    // This also means each push validates the image it replaces.
    confirm_running_image();

    const esp_partition_t* target = esp_ota_get_next_update_partition(nullptr);
    if (target == nullptr) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no OTA partition");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "OTA: receiving %d bytes into '%s'", req->content_len, target->label);

    esp_ota_handle_t handle = 0;
    if (esp_ota_begin(target, OTA_SIZE_UNKNOWN, &handle) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "esp_ota_begin failed");
        return ESP_FAIL;
    }

    char buf[1024];
    int  remaining = req->content_len;
    while (remaining > 0) {
        const int r = httpd_req_recv(req, buf, std::min(static_cast<int>(sizeof(buf)), remaining));
        if (r == HTTPD_SOCK_ERR_TIMEOUT) {
            continue;
        }
        if (r <= 0 || esp_ota_write(handle, buf, r) != ESP_OK) {
            esp_ota_abort(handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "receive/write failed");
            return ESP_FAIL;
        }
        remaining -= r;
    }

    if (esp_ota_end(handle) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "image invalid (esp_ota_end)");
        return ESP_FAIL;
    }
    if (esp_ota_set_boot_partition(target) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "set_boot_partition failed");
        return ESP_FAIL;
    }

    const bool to_normal = has_query_key(req, "normal");
    if (!to_normal) {
        static_cast<void>(config::request_debug_boot());  // come back on WiFi for the next push
    }
    ESP_LOGW(TAG, "OTA complete → '%s'; rebooting into %s mode",
             target->label, to_normal ? "normal" : "debug");
    httpd_resp_sendstr(req, to_normal ? "OTA OK; rebooting to normal mode\n"
                                      : "OTA OK; rebooting to debug mode\n");

    xTaskCreate(deferred_reboot, "ota_reboot", 2048, nullptr, 5, nullptr);
    return ESP_OK;
}

esp_err_t start_http_server()
{
    httpd_handle_t server = nullptr;
    httpd_config_t cfg    = HTTPD_DEFAULT_CONFIG();
    // Console commands like `ramp`/`trace` block for seconds while streaming;
    // give the handler task generous headroom and a long recv timeout.
    cfg.stack_size      = 8192;
    cfg.recv_wait_timeout = 60;
    cfg.send_wait_timeout = 60;

    ESP_RETURN_ON_ERROR(httpd_start(&server, &cfg), TAG, "httpd start");

    const httpd_uri_t ws{
        .uri                      = "/ws",
        .method                   = HTTP_GET,
        .handler                  = &ws_handler,
        .user_ctx                 = nullptr,
        .is_websocket             = true,
        .handle_ws_control_frames = false,
        .supported_subprotocol    = nullptr,
    };
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &ws), TAG, "ws register");

    const httpd_uri_t ota{
        .uri                      = "/ota",
        .method                   = HTTP_POST,
        .handler                  = &ota_handler,
        .user_ctx                 = nullptr,
        .is_websocket             = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol    = nullptr,
    };
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &ota), TAG, "ota register");
    return ESP_OK;
}

}  // namespace

void run()
{
    const config::Network net = config::get().network;
    if (net.ssid[0] == '\0') {
        ESP_LOGE(TAG, "debug mode requested but no WiFi SSID configured; "
                      "rebooting to normal mode");
        esp_restart();
    }

    // Register the command set for remote dispatch (no USB REPL in this mode).
    console::init_for_remote();

    if (!wifi_connect(net)) {
        ESP_LOGW(TAG, "WiFi join failed; rebooting to normal mode");
        esp_restart();
    }

    if (start_http_server() != ESP_OK) {
        ESP_LOGW(TAG, "HTTP server failed to start; rebooting to normal mode");
        esp_restart();
    }

    // Note: we deliberately do NOT confirm the running image here. The debug
    // server being up only proves WiFi+httpd work — not the normal-mode code
    // (motion, Zigbee, the recovery switch). Confirming on that shallow signal
    // would defeat rollback for a normal-mode-broken image. Instead /ota
    // confirms the image it replaces, and main confirms on Zigbee rejoin — the
    // real normal-mode health gate.

    ESP_LOGI(TAG, "=================================================");
    ESP_LOGI(TAG, " WiFi debug mode ready");
    ESP_LOGI(TAG, " websocket console: ws://%s/ws", acquired_ip);
    ESP_LOGI(TAG, " firmware push:     curl --data-binary @fw.bin http://%s/ota", acquired_ip);
    ESP_LOGI(TAG, "=================================================");
}

void confirm_running_image()
{
    const esp_partition_t* running = esp_ota_get_running_partition();
    esp_ota_img_states_t   state;
    if (esp_ota_get_state_partition(running, &state) == ESP_OK &&
        state == ESP_OTA_IMG_PENDING_VERIFY) {
        if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK) {
            ESP_LOGI(TAG, "running image confirmed valid (rollback cancelled)");
        } else {
            ESP_LOGW(TAG, "failed to confirm running image; it may roll back on reboot");
        }
    }
}

}  // namespace hvmrf01::http_debug
