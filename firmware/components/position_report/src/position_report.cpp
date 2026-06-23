#include "hv-mrf-01/position_report.hpp"

#include <cstdint>
#include <utility>

#include "esp_event.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "hv-mrf-01/motion.hpp"

namespace hvmrf01::position_report {

ESP_EVENT_DEFINE_BASE(EVENTS);

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

void publish(std::uint8_t pct)
{
    esp_event_post(EVENTS, std::to_underlying(Event::PositionChanged),
                   &pct, sizeof(pct), portMAX_DELAY);
}

void report_task(void*)
{
    int  last_reported = NONE;
    bool was_moving     = false;

    TickType_t wake = xTaskGetTickCount();
    while (true) {
        vTaskDelayUntil(&wake, POLL_PERIOD);

        const bool moving  = motion::is_moving();
        const bool stopped = was_moving && !moving;  // move just ended
        was_moving = moving;

        const auto pos = motion::position_pct();
        if (!pos.valid) {
            continue;  // not homed / not calibrated — no meaningful percentage
        }

        // Post on a 1% change, and once more on the move→stop edge so consumers
        // land on the true final position even if it fell between steps.
        if (pos.pct != last_reported || stopped) {
            publish(pos.pct);
            last_reported = pos.pct;
        }
    }
}

}  // namespace

void start()
{
    const BaseType_t ok = xTaskCreate(&report_task, "posreport", STACK_SZ,
                                      nullptr, TASK_PRIO, nullptr);
    configASSERT(ok == pdPASS);
    ESP_LOGI(TAG, "position reporter up @ %d Hz", POLL_HZ);
}

}  // namespace hvmrf01::position_report
