#pragma once

#include <cstdint>

#include "esp_event.h"

// Reports cover position onto the esp_event bus. A low-rate task polls the
// motion controller for the current lift percentage and posts a PositionChanged
// event whenever it changes by a percent or a move ends; motion events also
// trigger immediate reports for motion synchronization points without waiting
// for the next poll.
//
// Transport-agnostic by design: the reporter knows nothing about Zigbee. The
// application bridges PositionChanged to the hub (zigbee::report_position), so
// this component never takes the Zigbee stack lock and never runs on the 100 Hz
// control loop.
//
// Position is only meaningful once the blind is homed and mm-calibrated; until
// then motion::position_pct() reports invalid and the task simply idles.

namespace hvmrf01::position_report {

// Published when the reported cover position changes (and once when a move
// ends). Subscribe via esp_event_handler_register on this base.
ESP_EVENT_DECLARE_BASE(EVENTS);

enum class Event : std::int32_t
{
    PositionChanged,  // data: std::uint8_t ZCL lift percentage (0 open, 100 closed)
};

// Start the reporting task. Call once from app_main in normal mode, after
// motion is up. (No need in WiFi debug mode — there's no hub to report to.)
void start();

}  // namespace hvmrf01::position_report
