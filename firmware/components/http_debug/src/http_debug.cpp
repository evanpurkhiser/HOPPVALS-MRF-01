#include "hv-mrf-01/http_debug.hpp"

#include <algorithm>
#include <cstring>
#include <string>

#include "esp_check.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "hv-mrf-01/config.hpp"
#include "hv-mrf-01/console.hpp"

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
    esp_netif_create_default_wifi_sta();

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

    ESP_LOGI(TAG, "=================================================");
    ESP_LOGI(TAG, " WiFi debug mode ready");
    ESP_LOGI(TAG, " websocket console: ws://%s/ws", acquired_ip);
    ESP_LOGI(TAG, "=================================================");
}

}  // namespace hvmrf01::http_debug
