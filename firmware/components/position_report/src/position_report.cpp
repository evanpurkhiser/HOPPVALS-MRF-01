#include "hv-mrf-01/position_report.hpp"

#include <atomic>
#include <cstdint>
#include <utility>

#include "esp_event.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "hv-mrf-01/motion.hpp"

namespace hvmrf01::position_report {

// Explicit, component-unique base string — not ESP_EVENT_DEFINE_BASE(EVENTS).
// Two components stringizing the same "EVENTS" name collide: esp_event matches
// bases by pointer and the linker pools identical literals, so the bases would
// share an address and cross-deliver events (a null-data zigbee event landing
// here used to crash on_position_changed). See the matching note in zigbee.cpp.
const esp_event_base_t EVENTS = "hvmrf01.posreport";

namespace {

constexpr auto* TAG = "hv-mrf-01.posreport";

// Poll fast enough to catch every 1% step at cruise — 1% of full travel is a
// fraction of a revolution (~0.3 s at cover speed) — without spamming the bus.
// The control loop runs at 100 Hz; we sit well below it.
constexpr int        POLL_HZ     = 10;
constexpr TickType_t POLL_PERIOD = pdMS_TO_TICKS(1000 / POLL_HZ);

constexpr UBaseType_t   TASK_PRIO = 4;  // below the 100 Hz control task (prio 6)
constexpr std::uint32_t STACK_SZ  = 3072;

// Sentinel for "nothing reported yet this session" so the first valid sample
// always posts.
constexpr int NONE = -1;

std::atomic<int> last_reported{ NONE };

void publish(std::uint8_t pct)
{
    esp_event_post(EVENTS, std::to_underlying(Event::PositionChanged),
                   &pct, sizeof(pct), portMAX_DELAY);
    last_reported.store(pct);
}

void publish_current_position(bool force, bool allow_uncalibrated_top = false)
{
    const auto pos = motion::position_pct();
    if (!pos.valid) {
        if (allow_uncalibrated_top && motion::is_homed()) {
            publish(0);
        }
        return;  // not homed / not calibrated — no meaningful percentage
    }

    if (force || pos.pct != last_reported.load()) {
        publish(pos.pct);
    }
}

void on_motion_event(void*, esp_event_base_t, std::int32_t id, void*)
{
    if (static_cast<motion::Event>(id) != motion::Event::PositionReportRequested) {
        return;
    }

    // Homing establishes the top reference even without mm calibration; other
    // motion events use the current calibrated percentage.
    publish_current_position(true, true);
}

void report_task(void*)
{
    bool was_moving     = false;

    TickType_t wake = xTaskGetTickCount();
    while (true) {
        vTaskDelayUntil(&wake, POLL_PERIOD);

        const bool moving  = motion::is_moving();
        const bool stopped = was_moving && !moving;  // move just ended
        was_moving = moving;

        // Post on a 1% change, and once more on the move→stop edge so consumers
        // land on the true final position even if it fell between steps.
        publish_current_position(stopped);
    }
}

}  // namespace

void start()
{
    ESP_ERROR_CHECK(esp_event_handler_register(motion::EVENTS, ESP_EVENT_ANY_ID,
                                               &on_motion_event, nullptr));

    const BaseType_t ok = xTaskCreate(&report_task, "posreport", STACK_SZ,
                                      nullptr, TASK_PRIO, nullptr);
    configASSERT(ok == pdPASS);
    ESP_LOGI(TAG, "position reporter up @ %d Hz", POLL_HZ);
}

}  // namespace hvmrf01::position_report
