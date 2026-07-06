#pragma once

#include <cstdint>

#include "esp_event.h"

// Closed-loop velocity controller for the two blind motors.
//
// Architecture: a 100 Hz FreeRTOS task samples both encoders, computes
// per-motor measured RPM, runs an independent PI+feedforward speed loop
// per motor, and drives the motor PWM via `hvmrf01::motor::drive`.
//
// Cross-coupled sync: each motor's RPM setpoint is biased by K_SYNC ×
// (count_other - count_me). The lagging motor speeds up and the leading
// motor slows down — symmetric, no master. See DESIGN.md "Two-motor
// synchronization (C)" for the rationale.
//
// Safety: if the count delta between motors exceeds SYNC_FAULT_LIMIT we
// brake both and enter a faulted state. Subsequent motion commands are ignored
// until stop() clears the fault, except home(), which intentionally clears it
// so calibration can recover from an endpoint/jam fault.
//
// Beyond the speed loop, the controller homes against the top hard stop and
// seeks absolute positions (go_to_mm/pct) with a soft-landing approach that
// eases the setpoint down over the final revolution. Still open: full
// trajectory generation (trapezoidal accel + spring easing).

namespace hvmrf01::motion {

enum class Direction : std::uint8_t { Raise, Lower, Stop };

// Motion events. Subscribe through esp_event_handler_register().
ESP_EVENT_DECLARE_BASE(EVENTS);

enum class Event : std::int32_t
{
    PositionReportRequested,  // position changed at a known synchronization point
};

// Debug/manual velocity control: set the target output-shaft RPM and direction.
// Thread-safe; the control task picks this up on the next 10 ms tick. RPM is the
// post-gearbox shaft speed (matches encoder::COUNTS_PER_OUTPUT_REV convention).
//
// Direction::Stop is equivalent to calling stop() — both motors brake.
void set_target(int rpm, Direction d);

// Halt the controller, brake both motors, zero the PI integrators, and
// clear any fault flag.
void stop();

// Per-motor outcome of a homing run.
struct HomeResult
{
    bool left;   // true if the left motor settled at the top stop
    bool right;  // true if the right motor settled
};

// Home both motors against the top hard stop. Drives each upward open-loop at
// config home_duty_pct; a motor is done once its encoder has been stopped for
// home_settle_ms, at which point that motor brakes and its encoder is zeroed
// (top = 0). The sync/stall fault watchdogs are bypassed for the run (the
// encoders intentionally diverge and the stall is the success condition). Any
// latched fault is cleared on entry. Blocks until both settle or
// home_timeout_s elapses; returns which sides settled. Both motors end braked.
HomeResult home();

// Non-blocking home: kick off a homing run on a one-shot task and return
// immediately, for callers that must not block (the Zigbee "calibrate" command
// runs in the stack callback). Returns false if a homing run is already in
// progress. The control task stands down for the duration; is_homed()/is_moving()
// reflect the outcome once it completes.
bool begin_home();

// Outcome of a go_to_mm() move.
enum class GoToStatus : std::uint8_t
{
    Arrived,        // reached the target within tolerance
    NotHomed,       // no valid zero reference — run home() first
    NotCalibrated,  // config mm_per_rev is unset
    Faulted,        // a stall/sync fault tripped mid-move (e.g. hit the bottom)
    Timeout,        // didn't arrive within the time cap
};

struct GoToResult
{
    GoToStatus status;
    float      mm_l;  // final left position, mm below the homed top
    float      mm_r;  // final right position
};

// Move both motors to an absolute position `mm` below the homed top, using the
// synced speed loop, slowing over the final revolution and braking on arrival.
// `rpm` is the cruise speed for the move; 0 uses config cover_rpm. Requires a
// prior successful home() (the zero reference) and a calibrated config
// mm_per_rev. Negative mm is clamped to 0 (the top). Blocks until arrival,
// fault, or the time cap; returns the outcome and final positions.
GoToResult go_to_mm(float mm, int rpm = 0);

// Convenience over go_to_mm: position as a percentage of full travel, where
// 100% = config hard_stop_mm. The resulting mm is still clamped to the soft
// stop inside go_to_mm, so a 100% command on a blind with a soft stop set
// stops at the soft stop. `rpm` is the cruise speed; 0 uses cover_rpm.
GoToResult go_to_pct(float pct, int rpm = 0);

// Non-blocking variants for callers that must not block (the Zigbee
// GoToLiftPercentage handler runs in the stack callback — blocking it for a
// multi-second move would stall the mainloop). Starts the move and returns
// immediately; the control task drives to the target and brakes on arrival.
// `rpm` is the cruise speed; 0 uses cover_rpm. Returns false if the move can't
// start (not homed or mm_per_rev unset).
bool begin_go_to_mm(float mm, int rpm = 0);
bool begin_go_to_pct(float pct, int rpm = 0);

// Whether a valid home reference exists (set by a successful home(), cleared on
// boot). go_to_mm requires it.
bool is_homed();

// True while the controller is actively driving — a manual move, a position
// seek, or a homing run. False when idle/braked/faulted. Lets a caller poll for
// the end of a fire-and-forget move (begin_go_to_*).
bool is_moving();

// Current per-motor position, in mm below the homed top. valid is false when
// there's no usable reference (not homed) or no mm calibration — the mm values
// are then meaningless.
struct PositionMm
{
    bool  valid;
    float mm_l;
    float mm_r;
};
PositionMm position_mm();

// Current cover position as a ZCL lift percentage: 0 = fully open (the homed
// top), 100 = fully closed (hard_stop_mm down) — the inverse mapping go_to_pct
// accepts, and the value reported to the hub. Uses the average of both motors.
// valid is false when there's no usable reference (not homed) or no mm
// calibration, in which case pct is meaningless. Cheap enough to poll.
struct PositionPct
{
    bool         valid;
    std::uint8_t pct;
};
PositionPct position_pct();

// Start the 100 Hz control task and register the ZCL cover command handlers
// (open/close/stop/go-to) with the zigbee component — the controller owns the
// cover semantics. Call once from app_main, after motor and encoder are up and
// before zigbee::start(), so the handlers are in place before any command can
// arrive.
void start();

}  // namespace hvmrf01::motion
