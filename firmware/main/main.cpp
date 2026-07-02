// Top-level wiring. app_main brings up the default event loop, loads persisted
// config, and starts the hardware stack (motor, encoder, current sense, motion,
// LED). It then selects this boot's radio personality — normal Zigbee operation
// or the one-shot WiFi debug console — and, in normal mode, arms a recovery
// fallback that reboots into debug mode if the Zigbee network is never joined.
//
// The components own their own subsystems and talk over the esp_event bus; main
// just sequences their bring-up and registers the app-level Zigbee listener.

#include <cinttypes>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "hv-mrf-01/config.hpp"
#include "hv-mrf-01/console.hpp"
#include "hv-mrf-01/current_sense.hpp"
#include "hv-mrf-01/encoder.hpp"
#include "hv-mrf-01/http_debug.hpp"
#include "hv-mrf-01/led.hpp"
#include "hv-mrf-01/motion.hpp"
#include "hv-mrf-01/motor.hpp"
#include "hv-mrf-01/position_report.hpp"
#include "hv-mrf-01/zigbee.hpp"

namespace {

constexpr auto *TAG = "hv-mrf-01.app";

// If a normal boot never joins the Zigbee network within this window, fall back
// to WiFi debug mode — so a Zigbee-side fault (or a regression that breaks the
// join path) can't strand a mounted device that has no USB access. Generous
// enough not to false-fire on a slow commission; the JoinedNetwork handler
// cancels it on success.
constexpr std::int64_t RECOVERY_TIMEOUT_US = 5LL * 60 * 1000 * 1000;  // 5 minutes
esp_timer_handle_t recovery_timer = nullptr;

// Arm the one-shot debug flag and reboot. Halt the control loop and sleep the
// drivers first so the motors are never mid-drive across the reset.
void reboot_into_debug(const char *why)
{
    ESP_LOGW(TAG, "%s; arming debug boot and rebooting", why);
    static_cast<void>(hvmrf01::config::request_debug_boot());
    hvmrf01::motion::stop();
    hvmrf01::motor::disable();
    esp_restart();
}

void enter_debug_recovery(void *)
{
    reboot_into_debug("no Zigbee join within recovery window");
}

// Bridge the transport-agnostic position reporter to the hub. The reporter
// posts PositionChanged on the bus; we mirror it onto the Window Covering
// cluster. Runs in the default event loop task (not a Zigbee callback), so
// taking the stack lock in report_position is safe here.
void on_position_changed(void *, esp_event_base_t, std::int32_t, void *data)
{
    if (data == nullptr) {
        return;
    }
    const auto pct = *static_cast<std::uint8_t *>(data);
    hvmrf01::zigbee::report_position(pct);
}

void on_zigbee_event(void *, esp_event_base_t, std::int32_t id, void *data)
{
    using hvmrf01::zigbee::Event;
    switch (static_cast<Event>(id)) {
    case Event::JoinedNetwork:
        ESP_LOGI(TAG, "→ joined zigbee network");
        // We're reachable over Zigbee, so the recovery fallback isn't needed.
        if (recovery_timer != nullptr) {
            esp_timer_stop(recovery_timer);
        }
        // Rejoining proves a freshly-OTA'd image is healthy in normal mode;
        // confirm it so the bootloader won't roll back on the next reboot.
        hvmrf01::http_debug::confirm_running_image();
        break;
    case Event::LeftNetwork:
        ESP_LOGI(TAG, "→ left zigbee network");
        break;
    case Event::Identify: {
        const auto effect = *static_cast<std::uint8_t *>(data);
        ESP_LOGI(TAG, "→ identify effect 0x%02x", effect);
    } break;
    case Event::EnterDebug:
        reboot_into_debug("reboot-to-debug command received");
        break;
    case Event::Calibrate:
        ESP_LOGI(TAG, "→ calibrate: homing to top");
        if (!hvmrf01::motion::begin_home()) {
            ESP_LOGW(TAG, "calibrate ignored — homing already in progress");
        }
        break;
    }
}

}  // namespace

extern "C" void app_main()
{
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_event_handler_register(hvmrf01::zigbee::EVENTS,
                                               ESP_EVENT_ANY_ID, &on_zigbee_event,
                                               nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register(hvmrf01::position_report::EVENTS,
                                               ESP_EVENT_ANY_ID, &on_position_changed,
                                               nullptr));
    ESP_LOGI(TAG, "Starting hv-mrf-01 firmware");
    // Bring up NVS and load persisted config before any consumer reads it.
    if (auto r = hvmrf01::config::init(); !r) {
        ESP_LOGW(TAG, "config init failed (err %d); running on defaults",
                 static_cast<int>(r.error()));
    }
    // Hardware stack comes up in both modes so the CLI and benchmarks work
    // whether we're talking Zigbee or the WiFi debug console.
    hvmrf01::motor::start();          // configure pins + drivers (land in brake)
    hvmrf01::encoder::start();        // PCNT quadrature readers for both motors
    hvmrf01::current_sense::start();  // ADC1 IPROPI sampling task (100 Hz)
    hvmrf01::motion::start();         // 100 Hz speed loop; registers cover handlers
    hvmrf01::led::start();

    // One-shot debug flag (set over Zigbee/CLI) selects the radio personality
    // for this boot. The flag is cleared on read, so any later reboot returns
    // to normal Zigbee operation.
    if (hvmrf01::config::take_debug_boot()) {
        ESP_LOGW(TAG, "→ entering WiFi debug mode");
        hvmrf01::http_debug::run();   // WiFi STA + websocket console (no Zigbee)
        return;
    }

    hvmrf01::console::start();        // serial REPL — needs motor + encoder up first
    hvmrf01::zigbee::start();
    hvmrf01::position_report::start();  // push cover position to the hub as it moves

    // Arm the recovery fallback; the JoinedNetwork handler cancels it on join.
    const esp_timer_create_args_t recovery_args{
        .callback              = &enter_debug_recovery,
        .arg                   = nullptr,
        .dispatch_method       = ESP_TIMER_TASK,
        .name                  = "zb_recovery",
        .skip_unhandled_events = false,
    };
    ESP_ERROR_CHECK(esp_timer_create(&recovery_args, &recovery_timer));
    ESP_ERROR_CHECK(esp_timer_start_once(recovery_timer, RECOVERY_TIMEOUT_US));
}
