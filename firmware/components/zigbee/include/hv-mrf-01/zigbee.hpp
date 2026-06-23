#pragma once

#include <cstdint>

#include "esp_event.h"

namespace hvmrf01::zigbee {

// All events published by the Zigbee component flow through this esp_event
// base. The application registers handlers via esp_event_handler_register;
// the Zigbee task posts events from inside the ZCL action handler.
ESP_EVENT_DECLARE_BASE(EVENTS);

// Notifications: event bus
//
// Things downstream code may care about but doesn't need to respond to.
// Multiple subscribers are fine. Subscribe via esp_event_handler_register.

enum class Event : std::int32_t
{
    JoinedNetwork, // no data
    LeftNetwork,   // no data
    Identify,      // data: std::uint8_t ZCL IdentifyEffect ID
    EnterDebug,    // no data: the "Debug Mode" switch was turned on
};

// Commands: synchronous handler interface
//
// Cover commands have request/response semantics — the hub waits for a ZCL
// status code in response. The application (the motion controller) registers a
// set of handlers; the Zigbee component calls them synchronously from inside
// the ZCL command callback and uses the returned status as the response.
//
// Handlers run in the Zigbee task context. Keep them fast and non-blocking —
// kick off motion via a queue/notification to your motion task, then return.

// Mirror of the ZCL status codes we'll actually use. Defined here so the
// application doesn't need to include any SDK headers.
enum class CommandStatus : std::uint8_t
{
    Success      = 0x00, // command accepted
    Failure      = 0x01, // generic failure
    InvalidValue = 0x87, // arg out of range
    NotFound     = 0x8b, // cluster/endpoint not configured
    NotCalibrated = 0xc2 // device hasn't been calibrated yet (ZCL CalibrationError)
};

struct CoverHandlers
{
    CommandStatus (*open)()                          = nullptr;
    CommandStatus (*close)()                         = nullptr;
    CommandStatus (*stop)()                          = nullptr;
    CommandStatus (*go_to_percent)(std::uint8_t pct) = nullptr;
};

// Register cover handlers. Call before start() (or at any point thereafter —
// commands arriving before registration get CommandStatus::Failure). Calling
// again replaces the previous set.
void register_cover_handlers(const CoverHandlers& handlers);

// State updates: motion → Zigbee, free function
//
// Push current cover position back to the hub. Writes the Window Covering
// cluster's CurrentPositionLiftPercentage attribute, which the SDK then
// reports per the hub's configured reporting interval.
//
// Thread-safe: internally acquires the Zigbee stack lock. Do not call from
// within a Zigbee handler (it'd deadlock).

void report_position(std::uint8_t pct);

// ZCL Identify effect IDs (see ZCL spec §3.5.2.3). Exposed here so the LED
// component can map directly without a magic-number switch.
enum class IdentifyEffect : std::uint8_t
{
    Blink         = 0x00, // turn on/off once
    Breathe       = 0x01, // fade in/out cyclically for ~15s
    Okay          = 0x02, // two short blinks
    ChannelChange = 0x0B, // one quick flash
    FinishEffect  = 0xFE, // gracefully end current effect
    StopEffect    = 0xFF, // terminate immediately
};

// Spawn the Zigbee task, init the stack, register endpoints, and start
// commissioning. Returns once everything is queued — actual joining is
// asynchronous and surfaces via Event::JoinedNetwork.
//
// Call exactly once from app_main. The application's esp_event default loop
// must already exist before calling.
void start();

} // namespace hvmrf01::zigbee
