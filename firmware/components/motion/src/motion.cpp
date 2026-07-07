#include "hv-mrf-01/motion.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <utility>

#include "esp_event.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "hv-mrf-01/config.hpp"
#include "hv-mrf-01/encoder.hpp"
#include "hv-mrf-01/event_log.hpp"
#include "hv-mrf-01/motor.hpp"
#include "hv-mrf-01/zigbee.hpp"

namespace hvmrf01::motion {

const esp_event_base_t EVENTS = "hvmrf01.motion";

namespace {

constexpr auto* TAG = "hv-mrf-01.motion";

// Tuning constants (gains, watchdog thresholds, cover speed) live in
// config::Motion and are read per tick from a snapshot — see control_task.
// Only the structural loop rate is fixed at compile time: it sets the task
// period and the DT used by the integrator, so it can't change at runtime.
constexpr int   CONTROL_HZ = 100;
constexpr TickType_t TICK  = pdMS_TO_TICKS(1000 / CONTROL_HZ);
constexpr float DT_S       = 1.0f / static_cast<float>(CONTROL_HZ);
constexpr int   MS_PER_TICK = 1000 / CONTROL_HZ;

// FreeRTOS task config.
constexpr UBaseType_t TASK_PRIO   = 6;
constexpr std::uint32_t STACK_SZ  = 4096;

// Shared command state (writer = anyone, reader = control task)
//
// `target_rpm` is the magnitude requested by set_target(); `direction`
// turns it into a signed-velocity intent. `Stop` is the idle/braked state.

std::atomic<int>       target_rpm{ 0 };
std::atomic<Direction> direction{ Direction::Stop };
std::atomic<bool>      fault{ false };
// While set, home() owns the motors directly and the control task stands down
// so the two never drive at once.
std::atomic<bool>      homing{ false };

// A valid zero reference exists (set by a successful home()). go_to_mm needs it.
std::atomic<bool>      homed{ false };

// Position-seek state. While position_mode is set, the control task drives both
// motors toward target_counts (counts below the homed top) via the synced loop
// and sets arrived when within tolerance.
std::atomic<bool>         position_mode{ false };
std::atomic<bool>         arrived{ false };
std::atomic<std::int32_t> target_counts{ 0 };
// Cruise speed (RPM) for the active position move; resolved at begin time
// (0 from the caller means "use cover_rpm").
std::atomic<int>          move_rpm{ 0 };

// Cross-task request to clear control-loop-local state. The control task owns
// the integrators/watchdog counters; callers bump this instead of mutating them
// directly while the loop may be running.
std::atomic<std::uint32_t> reset_request_gen{ 0 };

// Stall watchdog window. Once startup grace has passed and the controller is
// applying meaningful duty, both motors must show progress across this window.
int stall_ticks = 0;
std::int32_t stall_window_start_l = 0;
std::int32_t stall_window_start_r = 0;
Direction stall_window_dir = Direction::Stop;
// Ticks elapsed since motion became active (since last idle→active edge).
// Stall watchdog ignores readings until this exceeds the grace window.
int active_ticks = 0;

// Per-motor controller state (owned by the control task)

struct ControllerState
{
    motor::Side  side;
    const char*  label;
    std::int32_t prev_count = 0;
    float        i_accum    = 0.0f;
};

std::array<ControllerState, 2> controllers{
    ControllerState{ .side = motor::Side::Left,  .label = "L" },
    ControllerState{ .side = motor::Side::Right, .label = "R" },
};

// Helpers

// Measured RPM over the last tick. Positive = encoder counts rising,
// regardless of commanded direction — the controller compares magnitudes.
float measured_rpm(std::int32_t delta_counts)
{
    const float counts_per_sec = static_cast<float>(delta_counts) / DT_S;
    return (counts_per_sec / static_cast<float>(encoder::COUNTS_PER_OUTPUT_REV)) * 60.0f;
}

motor::Mode to_mode(Direction d)
{
    // As-built PCB wiring: motor::Mode::Forward spins the motors in the
    // physical "wind cord up" direction (raise the blind), so Raise maps to
    // Forward. Forward produces *negative* encoder counts on both motors;
    // dir_sign below flips the PI's idea of "positive motion" to match. (The
    // breadboard prototype had this reversed.)
    switch (d) {
    case Direction::Raise: return motor::Mode::Forward;
    case Direction::Lower: return motor::Mode::Reverse;
    case Direction::Stop:  return motor::Mode::Brake;
    }
    return motor::Mode::Brake;
}

event_log::MotionDirection log_direction(Direction d)
{
    switch (d) {
    case Direction::Raise: return event_log::MotionDirection::Raise;
    case Direction::Lower: return event_log::MotionDirection::Lower;
    case Direction::Stop:  return event_log::MotionDirection::Stop;
    }
    return event_log::MotionDirection::Stop;
}

event_log::Side log_side(motor::Side s)
{
    return s == motor::Side::Left ? event_log::Side::Left : event_log::Side::Right;
}

void brake_both()
{
    motor::drive(motor::Side::Both, motor::Mode::Brake, 0);
}

void reset_integrators()
{
    for (auto& c : controllers) {
        c.i_accum = 0.0f;
    }
}

void reset_motion_state()
{
    reset_integrators();
    stall_ticks  = 0;
    active_ticks = 0;
    stall_window_dir = Direction::Stop;
}

void reset_stall_window(std::int32_t count_l, std::int32_t count_r, Direction dir)
{
    stall_ticks = 0;
    stall_window_start_l = count_l;
    stall_window_start_r = count_r;
    stall_window_dir = dir;
}

void request_motion_state_reset()
{
    reset_request_gen.fetch_add(1, std::memory_order_release);
}

void stop_with_source(event_log::StopSource source)
{
    event_log::motion_stop(source,
                           fault.load(), position_mode.load(), homing.load());
    target_rpm.store(0);
    direction.store(Direction::Stop);
    position_mode.store(false);
    fault.store(false);
    request_motion_state_reset();
    // The control task picks up the new state on the next tick; brake
    // immediately here so the motors don't keep coasting through that
    // (≤10 ms) window.
    motor::drive(motor::Side::Both, motor::Mode::Brake, 0);
}

// Latch a fault and brake: the control task skips ticks until stop() clears it.
// Callers log the specific cause first.
void enter_fault()
{
    fault.store(true);
    direction.store(Direction::Stop);
    position_mode.store(false);
    brake_both();
    reset_motion_state();
}

void post(Event ev)
{
    esp_event_post(EVENTS, std::to_underlying(ev), nullptr, 0, portMAX_DELAY);
}

// Effective downward travel limit in mm: the soft stop if one is set, else the
// hard stop. Both are clamped sane.
float down_limit_mm(const config::Motion& m)
{
    if (m.soft_stop_mm > 0.0f) {
        return std::min(m.soft_stop_mm, m.hard_stop_mm);
    }
    return m.hard_stop_mm;
}

// That limit expressed in encoder counts below the homed top. Returns -1 when
// it can't be computed (no mm calibration), meaning "no down limit enforced".
std::int32_t down_limit_counts(const config::Motion& m)
{
    if (m.mm_per_rev <= 0.0f) {
        return -1;
    }
    return static_cast<std::int32_t>(down_limit_mm(m) *
                                     encoder::COUNTS_PER_OUTPUT_REV / m.mm_per_rev);
}

// Sign convention for the speed loop: under Raise, counts move negative; under
// Lower, counts move positive. dir_sign projects measured RPM and sync bias
// into the commanded reference frame ("positive = moving as requested") so the
// same PI code handles both directions. The H-bridge handles direction via
// to_mode().
void run_tick(const config::Motion& m,
              Direction dir,
              int base_setpoint_rpm,
              bool enforce_travel_limits)
{
    // Read both encoders up front so cross-coupling sees consistent values.
    const std::int32_t count_l = encoder::count(motor::Side::Left);
    const std::int32_t count_r = encoder::count(motor::Side::Right);

    // Stop / zero-setpoint: idle state. Don't continuously re-apply brake
    // (that would fight the motor::debug bench drives) and don't run the
    // sync watchdog (manual drives can legitimately diverge the encoders).
    // Transition into stopped is handled by stop() and the on-fault path.
    if (dir == Direction::Stop || base_setpoint_rpm <= 0) {
        reset_integrators();
        stall_ticks  = 0;
        active_ticks = 0;
        for (auto& c : controllers) {
            c.prev_count = (c.side == motor::Side::Left) ? count_l : count_r;
        }
        return;
    }

    // Position end-stops (graceful, not a fault). Once homed we stop at the top
    // (pos 0) going up and at the down limit going down, rather than running
    // into the mechanism and leaning on the stall watchdog. The top needs only
    // the home reference; the down limit also needs mm calibration.
    if (enforce_travel_limits && homed.load()) {
        const std::int32_t pos = (count_l + count_r) / 2;
        const std::int32_t dl  = down_limit_counts(m);
        const bool at_limit = (dir == Direction::Raise && pos <= 0) ||
                              (dir == Direction::Lower && dl >= 0 && pos >= dl);
        if (at_limit) {
            event_log::limit_stop(log_direction(dir), pos, dl);
            brake_both();
            reset_motion_state();
            direction.store(Direction::Stop);
            target_rpm.store(0);
            position_mode.store(false);
            arrived.store(true);
            post(Event::PositionReportRequested);
            for (auto& c : controllers) {
                c.prev_count = (c.side == motor::Side::Left) ? count_l : count_r;
            }
            return;
        }
    }

    // We're actively driving. Count time since the most recent idle→active
    // edge so the stall watchdog can skip the spin-up window.
    ++active_ticks;
    if (active_ticks == 1) {
        event_log::motion_drive_start(log_direction(dir), base_setpoint_rpm, count_l, count_r);
    }

    // Sync-watchdog: gross divergence while *actively* controlling → fault,
    // brake once, latch. Only meaningful when the controller is driving;
    // if a user is poking the motors via the raw debug API, divergence is
    // expected and not a fault.
    const int sync_err = std::abs(count_l - count_r);
    if (sync_err > m.sync_fault_limit) {
        event_log::fault_sync(count_l, count_r, sync_err);
        ESP_LOGW(TAG, "sync fault: |%ld - %ld| = %d > %d; braking",
                 static_cast<long>(count_l), static_cast<long>(count_r),
                 sync_err, m.sync_fault_limit);
        enter_fault();
        return;
    }

    // Sign of expected count motion per direction. Raise drives Mode::Forward
    // (counts go negative), Lower drives Mode::Reverse (counts go positive).
    // dir_sign projects the measured-RPM and sync-bias terms into the commanded
    // reference frame ("positive = on target") regardless of count direction.
    const float dir_sign = (dir == Direction::Raise) ? -1.0f : 1.0f;

    for (auto& c : controllers) {
        const std::int32_t count_me    = (c.side == motor::Side::Left) ? count_l : count_r;
        const std::int32_t count_other = (c.side == motor::Side::Left) ? count_r : count_l;
        const std::int32_t delta       = count_me - c.prev_count;
        c.prev_count = count_me;

        // Measured RPM, projected onto the commanded direction. If we
        // told the motor to Raise (positive counts) and got negative
        // counts, |measured| in our frame is negative — error grows,
        // which is correct.
        const float measured = dir_sign * measured_rpm(delta);

        // Cross-coupled sync bias: lagging motor (smaller count in the
        // commanded direction) gets a positive bump, leading motor gets
        // throttled. Projecting through dir_sign keeps the bias correct
        // when lowering (counts decreasing).
        const float bias = m.k_sync * dir_sign *
                           static_cast<float>(count_other - count_me);

        const float setpoint = static_cast<float>(base_setpoint_rpm) + bias;
        const float error    = setpoint - measured;

        c.i_accum = std::clamp(c.i_accum + error * DT_S, -m.i_max, m.i_max);

        // Static breakaway offset (direction-dependent) plus the slope term,
        // scaled by this motor's trim so the two sides are commanded the right
        // duty for the target speed despite their few-% hardware mismatch —
        // otherwise the PI + sync loop hunts to make up the constant offset.
        const float ff_offset = (dir == Direction::Raise) ? m.ff_offset_raise_pct
                                                           : m.ff_offset_lower_pct;
        const float ff_trim = (c.side == motor::Side::Left) ? m.ff_trim_l : m.ff_trim_r;
        const float ff_duty = (ff_offset + setpoint * m.duty_per_rpm) * ff_trim;
        float duty          = ff_duty + m.kp * error + m.ki * c.i_accum;
        duty                = std::clamp(duty, 0.0f, 100.0f);

        motor::drive(c.side, to_mode(dir), static_cast<int>(duty));
    }

    // Stall-watchdog: only fault when both motors make essentially no progress
    // across the configured window while the requested RPM is high enough that
    // measurable encoder movement should have happened. This avoids false stalls
    // during very slow moves without disabling jam protection solely because the
    // controller's current duty is low.
    const int startup_grace_ticks = m.startup_grace_ms / MS_PER_TICK;
    const int stall_fault_ticks   = std::max(1, m.stall_fault_ms / MS_PER_TICK);
    const float stall_window_s = static_cast<float>(m.stall_fault_ms) / 1000.0f;
    const float expected_counts = static_cast<float>(base_setpoint_rpm) *
        static_cast<float>(encoder::COUNTS_PER_OUTPUT_REV) * stall_window_s / 60.0f;
    const int noise_counts = std::max(3, m.stall_delta_max * 3);
    const int expected_min_progress = std::max(
        1, static_cast<int>(std::lround(expected_counts * 0.10f)));
    const int min_progress_counts = std::max(noise_counts, expected_min_progress);

    if (active_ticks <= startup_grace_ticks ||
        expected_counts <= static_cast<float>(min_progress_counts)) {
        reset_stall_window(count_l, count_r, dir);
        return;
    }

    if (stall_window_dir != dir) {
        reset_stall_window(count_l, count_r, dir);
        return;
    }

    if (stall_ticks == 0) {
        stall_window_start_l = count_l;
        stall_window_start_r = count_r;
        stall_window_dir = dir;
    }

    if (++stall_ticks < stall_fault_ticks) {
        return;
    }

    const std::int32_t delta_l = count_l - stall_window_start_l;
    const std::int32_t delta_r = count_r - stall_window_start_r;
    if (std::abs(delta_l) <= min_progress_counts &&
        std::abs(delta_r) <= min_progress_counts) {
        event_log::fault_stall(delta_l, delta_r, base_setpoint_rpm);
        ESP_LOGW(TAG, "stall fault: both motors moved <=%d counts in %d ms at target %d RPM",
                 min_progress_counts, m.stall_fault_ms, base_setpoint_rpm);
        enter_fault();
        return;
    }

    reset_stall_window(count_l, count_r, dir);
}

// Convert a distance in mm to encoder counts, falling back to `fallback` when
// there's no mm calibration (position mode requires it, so this shouldn't hit).
std::int32_t mm_to_counts(const config::Motion& m, float mm, std::int32_t fallback)
{
    if (m.mm_per_rev <= 0.0f) {
        return fallback;  // shouldn't happen — position mode requires calibration
    }
    return std::max<std::int32_t>(
        1, static_cast<std::int32_t>(mm * encoder::COUNTS_PER_OUTPUT_REV / m.mm_per_rev));
}

void finish_position_move(std::int32_t target, std::int32_t tolerance_counts)
{
    const std::int32_t pos = (encoder::count(motor::Side::Left) +
                              encoder::count(motor::Side::Right)) / 2;
    event_log::go_to_arrived(pos, target, tolerance_counts);
    motor::drive(motor::Side::Both, motor::Mode::Brake, 0);
    reset_motion_state();
    position_mode.store(false);
    arrived.store(true);
    post(Event::PositionReportRequested);
}

// One tick of a position-seek: drive both motors toward target_counts (counts
// below the homed top) via the synced speed loop. Normal targets only arrive
// once both encoders are independently within tolerance, so a tilted average
// cannot hide skew.

void run_position_tick(const config::Motion& m)
{
    const std::int32_t count_l = encoder::count(motor::Side::Left);
    const std::int32_t count_r = encoder::count(motor::Side::Right);
    const std::int32_t pos = (count_l + count_r) / 2;
    const std::int32_t target = target_counts.load();
    const std::int32_t err = target - pos;  // >0 ⇒ need to go down

    const std::int32_t tol_counts  = mm_to_counts(m, m.goto_tol_mm, 24);
    const std::int32_t slow_counts = mm_to_counts(m, m.goto_slow_mm, 896);
    const std::int32_t err_l = target - count_l;
    const std::int32_t err_r = target - count_r;
    const std::int32_t max_side_err = std::max(std::abs(err_l), std::abs(err_r));

    if (std::abs(err_l) <= tol_counts && std::abs(err_r) <= tol_counts) {
        finish_position_move(target, tol_counts);
        return;
    }

    // Down (lower) increases counts; up (raise) decreases them.
    Direction dir = (err > tol_counts) ? Direction::Lower : Direction::Raise;
    if (std::abs(err) <= tol_counts) {
        dir = (std::abs(err_l) > std::abs(err_r))
            ? (err_l > 0 ? Direction::Lower : Direction::Raise)
            : (err_r > 0 ? Direction::Lower : Direction::Raise);
    }

    // Cruise at the move's chosen speed (set at begin time; fall back to
    // cover_rpm). Ease the setpoint down over the slow-down zone for a soft
    // landing, flooring at goto_min_rpm so it keeps creeping rather than
    // stalling short.
    int        cruise = move_rpm.load();
    if (cruise <= 0) {
        cruise = m.cover_rpm;
    }
    int setpoint = cruise;
    const std::int32_t mag = std::max(std::abs(err), max_side_err);

    if (mag < slow_counts) {
        setpoint = std::max(m.goto_min_rpm, static_cast<int>(cruise * mag / slow_counts));
    }

    run_tick(m, dir, setpoint, false);
}

void control_task(void*)
{
    // Initialize prev_count to the current readings so the first tick's
    // delta isn't a huge spurious value if the encoders have drifted
    // since boot.
    controllers[0].prev_count = encoder::count(motor::Side::Left);
    controllers[1].prev_count = encoder::count(motor::Side::Right);

    // Snapshot the tuning config locally so the 100 Hz loop never reads the
    // shared config concurrently with a writer. Re-snapshot only when the
    // generation counter bumps (a save() published new values) — lets the
    // CLI retune gains live without a reboot.
    config::Motion tune = config::get().motion;
    std::uint32_t  tune_gen = config::generation();
    std::uint32_t  applied_reset_gen = reset_request_gen.load(std::memory_order_acquire);

    TickType_t last_wake = xTaskGetTickCount();
    while (true) {
        vTaskDelayUntil(&last_wake, TICK);

        // home() drives the motors directly while this is set; stand down.
        if (homing.load()) {
            continue;
        }

        if (const auto g = config::generation(); g != tune_gen) {
            tune     = config::get().motion;
            tune_gen = g;
        }

        if (const auto g = reset_request_gen.load(std::memory_order_acquire);
            g != applied_reset_gen) {
            reset_motion_state();
            applied_reset_gen = g;
        }

        // Faulted state is latched — the fault path inside run_tick (or the
        // sync watchdog) already braked once. Skip ticks until stop()
        // clears the latch.
        if (fault.load()) {
            continue;
        }

        if (position_mode.load()) {
            run_position_tick(tune);
        } else {
            run_tick(tune, direction.load(), target_rpm.load(), true);
        }
    }
}

// Cover command handlers (registered with the zigbee component)
//
// The ZCL Window Covering cluster routes here. The controller owns motor state,
// so the cover semantics (open = go to top, close = go to bottom, go-to =
// position seek) live with it rather than in the low-level motor driver — which
// is why the motor component no longer needs to know about zigbee or motion.
// These run in the zigbee task context, so they kick off motion and return
// immediately.

zigbee::CommandStatus handle_open()
{
    event_log::motion_open(is_homed());
    if (!is_homed()) {
        const bool started = begin_home();
        ESP_LOGI(TAG, "open while unhomed -> %s", started ? "homing" : "homing already active");
        return zigbee::CommandStatus::Success;
    }

    if (!begin_go_to_pct(0.0f)) {
        event_log::motion_reject(event_log::RejectReason::GoToBeginFailed, 0);
        ESP_LOGW(TAG, "open rejected — not homed / mm_per_rev unset");
        return zigbee::CommandStatus::Failure;
    }
    ESP_LOGI(TAG, "open → go-to 0%%");
    return zigbee::CommandStatus::Success;
}

zigbee::CommandStatus handle_close()
{
    event_log::motion_close(is_homed());
    if (!is_homed()) {
        const bool started = begin_home();
        ESP_LOGI(TAG, "close while unhomed -> %s", started ? "homing" : "homing already active");
        return zigbee::CommandStatus::Success;
    }

    if (!begin_go_to_pct(100.0f)) {
        event_log::motion_reject(event_log::RejectReason::GoToBeginFailed, 100);
        ESP_LOGW(TAG, "close rejected — not homed / mm_per_rev unset");
        return zigbee::CommandStatus::Failure;
    }
    ESP_LOGI(TAG, "close → go-to 100%%");
    return zigbee::CommandStatus::Success;
}

zigbee::CommandStatus handle_stop()
{
    ESP_LOGI(TAG, "stop");
    stop_with_source(event_log::StopSource::Cover);
    return zigbee::CommandStatus::Success;
}

zigbee::CommandStatus handle_go_to(std::uint8_t pct)
{
    // ZCL lift percentage: 0% = fully open (top), 100% = fully closed (bottom),
    // which matches go_to_pct's 0 = top / 100 = hard stop. Non-blocking start —
    // blocking the Zigbee callback for the whole move would stall the stack.
    //
    // We don't report position here: the position_report task pushes the actual
    // position as the blind travels and a final value on arrival, so the hub
    // tracks truth rather than the (optimistic) commanded target. Reporting from
    // this callback would also deadlock — report_position takes the stack lock
    // this handler already holds.
    event_log::motion_go_to(pct, is_homed());
    if (!is_homed()) {
        const bool started = begin_home();
        ESP_LOGI(TAG, "go-to %u%% while unhomed -> %s", pct,
                 started ? "homing" : "homing already active");
        return zigbee::CommandStatus::Success;
    }

    if (!begin_go_to_pct(static_cast<float>(pct))) {
        event_log::motion_reject(event_log::RejectReason::GoToBeginFailed, pct);
        ESP_LOGW(TAG, "go-to %u%% rejected (not homed / mm_per_rev unset)", pct);
        return zigbee::CommandStatus::Failure;
    }
    ESP_LOGI(TAG, "go-to %u%% -> move started", pct);
    return zigbee::CommandStatus::Success;
}

// Worker for begin_home(): run the blocking homing routine off the caller's
// thread, then self-delete. home() clears the `homing` flag when it finishes.
void home_task(void*)
{
    home();
    vTaskDelete(nullptr);
}

}  // namespace

void set_target(int rpm, Direction d)
{
    if (fault.load()) {
        event_log::motion_reject(event_log::RejectReason::Faulted, rpm, log_direction(d));
        ESP_LOGW(TAG, "set_target ignored — controller faulted; call stop() to clear");
        return;
    }
    if (homing.load()) {
        event_log::motion_reject(event_log::RejectReason::Homing, rpm, log_direction(d));
        ESP_LOGW(TAG, "set_target ignored — homing in progress");
        return;
    }
    // Down travel is unbounded until a home reference exists (no zero to enforce
    // the soft/hard stop against), so refuse to lower while unhomed — only raise
    // is allowed, which is also how the blind gets homed in the first place.
    if (d == Direction::Lower && !homed.load()) {
        event_log::motion_reject(event_log::RejectReason::LowerUnhomed, rpm, log_direction(d));
        ESP_LOGW(TAG, "lower ignored — not homed; raise or calibrate first");
        return;
    }
    if (rpm < 0) rpm = 0;
    target_rpm.store(0);
    direction.store(Direction::Stop);
    position_mode.store(false);
    request_motion_state_reset();
    arrived.store(false);
    target_rpm.store(rpm);
    direction.store(d == Direction::Stop ? Direction::Stop : d);
    event_log::motion_target(rpm, log_direction(d));
    ESP_LOGI(TAG, "target = %d RPM, dir = %s",
             rpm,
             d == Direction::Raise ? "raise" :
             d == Direction::Lower ? "lower" : "stop");
}

void stop()
{
    stop_with_source(event_log::StopSource::Controller);
}

HomeResult home()
{
    const auto tune = config::get().motion;
    const int  duty          = std::clamp(tune.home_duty_pct, 0, 100);
    const int  settle_ticks  = std::max(1, tune.home_settle_ms / MS_PER_TICK);
    const int  timeout_ticks = std::max(1, tune.home_timeout_s * 1000 / MS_PER_TICK);
    const int  stopped_max   = tune.stall_delta_max;

    // Take the motors away from the control task and clear any latched fault so
    // homing works even after a sync/stall fault. Also drop any active position
    // seek so the control task doesn't resume a stale target when we hand back.
    homing.store(true);
    target_rpm.store(0);
    direction.store(Direction::Stop);
    position_mode.store(false);
    fault.store(false);

    struct Mtr
    {
        motor::Side  side;
        const char*  label;
        bool         done;
        int          stopped;   // consecutive stopped ticks
        std::int32_t prev;
    };
    std::array<Mtr, 2> mtrs{
        Mtr{ motor::Side::Left,  "L", false, 0, encoder::count(motor::Side::Left) },
        Mtr{ motor::Side::Right, "R", false, 0, encoder::count(motor::Side::Right) },
    };

    ESP_LOGI(TAG, "homing: up at %d%% duty, settle %d ms, timeout %d s",
             duty, tune.home_settle_ms, tune.home_timeout_s);
    event_log::home_start(duty, tune.home_settle_ms, tune.home_timeout_s);

    TickType_t wake = xTaskGetTickCount();
    for (int t = 0; t < timeout_ticks; ++t) {
        vTaskDelayUntil(&wake, TICK);

        bool all_done = true;
        for (auto& m : mtrs) {
            if (m.done) {
                continue;  // already braked at its top
            }
            all_done = false;

            const std::int32_t now   = encoder::count(m.side);
            const std::int32_t delta = now - m.prev;
            m.prev = now;

            if (std::abs(delta) <= stopped_max) {
                if (++m.stopped >= settle_ticks) {
                    const std::int32_t settled_at = now;
                    motor::drive(m.side, motor::Mode::Brake, 0);
                    encoder::reset(m.side);
                    m.done = true;
                    event_log::home_settled(log_side(m.side), settled_at);
                    ESP_LOGI(TAG, "homing: %s settled at top", m.label);
                    continue;
                }
            } else {
                m.stopped = 0;
            }

            motor::drive(m.side, to_mode(Direction::Raise), duty);
        }

        if (all_done) {
            break;
        }
    }

    // Brake both and hand the motors back to the (idle) control task.
    motor::drive(motor::Side::Both, motor::Mode::Brake, 0);
    reset_motion_state();

    const HomeResult result{ mtrs[0].done, mtrs[1].done };
    // A valid zero reference exists only if both sides actually settled at the
    // top (encoders were zeroed there).
    const bool success = result.left && result.right;
    if (success) {
        // Settling each side independently can leave a few rebound counts after
        // the final all-motor brake. Make the home reference the actual resting
        // top position that future go-to-zero commands will target.
        vTaskDelay(pdMS_TO_TICKS(std::max(1, tune.home_settle_ms)));
        encoder::reset(motor::Side::Left);
        encoder::reset(motor::Side::Right);
    }

    homed.store(success);
    if (success) {
        post(Event::PositionReportRequested);
    }
    event_log::home_done(success, result.left, result.right);
    ESP_LOGI(TAG, "homing done: L=%s R=%s",
             result.left ? "ok" : "TIMEOUT", result.right ? "ok" : "TIMEOUT");
    homing.store(false);
    return result;
}

bool begin_home()
{
    // Claim the homing flag up front (CAS) so the control task stands down
    // before the worker starts driving, and so a second calibrate while one is
    // already running is a no-op rather than two runs fighting for the motors.
    bool expected = false;
    if (!homing.compare_exchange_strong(expected, true)) {
        event_log::home_begin(event_log::HomeBeginResult::AlreadyRunning);
        return false;
    }
    event_log::home_begin(event_log::HomeBeginResult::Claimed);
    const BaseType_t ok =
        xTaskCreate(&home_task, "home", STACK_SZ, nullptr, TASK_PRIO, nullptr);
    if (ok != pdPASS) {
        homing.store(false);
        event_log::home_begin(event_log::HomeBeginResult::TaskCreateFailed);
        return false;
    }
    return true;
}

GoToResult go_to_pct(float pct, int rpm)
{
    pct = std::clamp(pct, 0.0f, 100.0f);
    const auto tune = config::get().motion;
    return go_to_mm(pct / 100.0f * down_limit_mm(tune), rpm);
}

bool is_homed()
{
    return homed.load();
}

bool is_moving()
{
    if (homing.load() || position_mode.load()) {
        return true;
    }
    return direction.load() != Direction::Stop && target_rpm.load() > 0;
}

PositionMm position_mm()
{
    const auto m = config::get().motion;
    auto mm = [&](motor::Side s) {
        return static_cast<float>(encoder::count(s)) * m.mm_per_rev /
               static_cast<float>(encoder::COUNTS_PER_OUTPUT_REV);
    };
    return PositionMm{ homed.load() && m.mm_per_rev > 0.0f,
                       mm(motor::Side::Left), mm(motor::Side::Right) };
}

PositionPct position_pct()
{
    const auto m = config::get().motion;
    const float limit = down_limit_mm(m);
    if (!homed.load() || m.mm_per_rev <= 0.0f || limit <= 0.0f) {
        return PositionPct{ false, 0 };
    }
    const std::int32_t pos = (encoder::count(motor::Side::Left) +
                              encoder::count(motor::Side::Right)) / 2;
    const float mm  = static_cast<float>(pos) * m.mm_per_rev /
                      static_cast<float>(encoder::COUNTS_PER_OUTPUT_REV);
    const float pct = std::clamp(mm / limit * 100.0f, 0.0f, 100.0f);
    return PositionPct{ true, static_cast<std::uint8_t>(std::lround(pct)) };
}

bool begin_go_to_mm(float mm, int rpm)
{
    const auto tune = config::get().motion;
    if (fault.load()) {
        event_log::motion_reject(event_log::RejectReason::GoToFaulted);
        return false;
    }
    if (homing.load()) {
        event_log::motion_reject(event_log::RejectReason::GoToHoming);
        return false;
    }
    if (!homed.load()) {
        event_log::motion_reject(event_log::RejectReason::GoToUnhomed);
        return false;
    }
    if (tune.mm_per_rev <= 0.0f) {
        event_log::motion_reject(event_log::RejectReason::GoToNotCalibrated);
        return false;
    }

    mm = std::clamp(mm, 0.0f, down_limit_mm(tune));  // top .. soft/hard down limit
    const auto target = static_cast<std::int32_t>(
        mm * static_cast<float>(encoder::COUNTS_PER_OUTPUT_REV) / tune.mm_per_rev);
    const int cruise = (rpm > 0) ? std::clamp(rpm, 1, 300) : tune.cover_rpm;

    // Clean slate, then hand the seek to the control task.
    target_rpm.store(0);
    direction.store(Direction::Stop);
    request_motion_state_reset();
    arrived.store(false);
    target_counts.store(target);
    move_rpm.store(cruise);
    position_mode.store(true);
    event_log::go_to_begin(target, cruise, mm);

    ESP_LOGI(TAG, "go_to %.1f mm (target %ld counts) @ %d RPM",
             mm, static_cast<long>(target), cruise);
    return true;
}

bool begin_go_to_pct(float pct, int rpm)
{
    pct = std::clamp(pct, 0.0f, 100.0f);
    const auto tune = config::get().motion;
    return begin_go_to_mm(pct / 100.0f * down_limit_mm(tune), rpm);
}

GoToResult go_to_mm(float mm, int rpm)
{
    const auto tune = config::get().motion;

    auto pos_mm = [&](motor::Side s) {
        return static_cast<float>(encoder::count(s)) * tune.mm_per_rev /
               static_cast<float>(encoder::COUNTS_PER_OUTPUT_REV);
    };
    auto result = [&](GoToStatus st) {
        return GoToResult{ st, pos_mm(motor::Side::Left), pos_mm(motor::Side::Right) };
    };

    if (!homed.load()) {
        return result(GoToStatus::NotHomed);
    }
    if (tune.mm_per_rev <= 0.0f) {
        return result(GoToStatus::NotCalibrated);
    }

    if (!begin_go_to_mm(mm, rpm)) {
        return result(GoToStatus::Faulted);
    }

    // Generous cap: full travel at cover_rpm plus the slow-down tail.
    const int timeout_ticks = 30 * CONTROL_HZ;
    for (int t = 0; t < timeout_ticks; ++t) {
        vTaskDelay(TICK);
        if (arrived.load() || fault.load()) {
            break;
        }
    }

    // Ensure we're stopped and the control task is out of position mode.
    position_mode.store(false);
    motor::drive(motor::Side::Both, motor::Mode::Brake, 0);

    const GoToStatus st = arrived.load() ? GoToStatus::Arrived
                        : fault.load()   ? GoToStatus::Faulted
                                         : GoToStatus::Timeout;
    const auto r = result(st);
    event_log::go_to_done(static_cast<std::uint8_t>(st), r.mm_l, r.mm_r);
    ESP_LOGI(TAG, "go_to done: status=%d L=%.1fmm R=%.1fmm",
             static_cast<int>(st), r.mm_l, r.mm_r);
    return r;
}

void start()
{
    // Route ZCL cover commands onto the controller. Registered before
    // zigbee::start() (called later in app_main), so no command can arrive
    // before the handlers are in place.
    zigbee::register_cover_handlers({
        .open          = &handle_open,
        .close         = &handle_close,
        .stop          = &handle_stop,
        .go_to_percent = &handle_go_to,
    });

    BaseType_t ok = xTaskCreate(&control_task, "motion", STACK_SZ,
                                nullptr, TASK_PRIO, nullptr);
    configASSERT(ok == pdPASS);
    ESP_LOGI(TAG, "control loop started at %d Hz", CONTROL_HZ);
}

}  // namespace hvmrf01::motion
