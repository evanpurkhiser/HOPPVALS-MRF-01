#include "hv-mrf-01/motion.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "hv-mrf-01/config.hpp"
#include "hv-mrf-01/encoder.hpp"
#include "hv-mrf-01/motor.hpp"

namespace hvmrf01::motion {

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

// ── Shared command state (writer = anyone, reader = control task) ────────
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

// Consecutive ticks where both motors registered near-zero count delta
// while we're commanding motion. Resets when motion is detected.
int stall_ticks = 0;
// Ticks elapsed since motion became active (since last idle→active edge).
// Stall watchdog ignores readings until this exceeds the grace window.
int active_ticks = 0;

// ── Per-motor controller state (owned by the control task) ───────────────
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

// ── Helpers ──────────────────────────────────────────────────────────────

// Measured RPM over the last tick. Positive = encoder counts rising,
// regardless of commanded direction — the controller compares magnitudes.
float measured_rpm(std::int32_t delta_counts)
{
    const float counts_per_sec = static_cast<float>(delta_counts) / DT_S;
    return (counts_per_sec / static_cast<float>(encoder::COUNTS_PER_OUTPUT_REV)) * 60.0f;
}

motor::raw::Direction to_raw(Direction d)
{
    // As-built PCB wiring: motor::raw::Forward spins the motors in the
    // physical "wind cord up" direction (raise the blind), so Raise maps to
    // Forward. Raw forward produces *negative* encoder counts on both
    // motors; dir_sign below flips the PI's idea of "positive motion" to
    // match. (The breadboard prototype had this reversed.)
    switch (d) {
    case Direction::Raise: return motor::raw::Direction::Forward;
    case Direction::Lower: return motor::raw::Direction::Reverse;
    case Direction::Stop:  return motor::raw::Direction::Brake;
    }
    return motor::raw::Direction::Brake;
}

void brake_both()
{
    motor::raw::drive(motor::Side::Both, motor::raw::Direction::Brake, 0);
}

void reset_integrators()
{
    for (auto& c : controllers) c.i_accum = 0.0f;
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

// Sign convention for the speed loop: under Raise, both motors should
// produce positive count delta; under Lower, negative. We pass the
// "expected sign of count motion" into the loop so the same code handles
// both directions. The PI controller operates on magnitudes (always
// non-negative); the H-bridge handles direction via to_raw().
void run_tick(const config::Motion& m, Direction dir, int base_setpoint_rpm)
{
    // Read both encoders up front so cross-coupling sees consistent values.
    const std::int32_t count_l = encoder::count(motor::Side::Left);
    const std::int32_t count_r = encoder::count(motor::Side::Right);

    // Stop / zero-setpoint: idle state. Don't continuously re-apply brake
    // (that would fight raw debug / motor::raw drives) and don't run the
    // sync watchdog (raw drives can legitimately diverge the encoders).
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
    if (homed.load()) {
        const std::int32_t pos = (count_l + count_r) / 2;
        const std::int32_t dl  = down_limit_counts(m);
        const bool at_limit = (dir == Direction::Raise && pos <= 0) ||
                              (dir == Direction::Lower && dl >= 0 && pos >= dl);
        if (at_limit) {
            brake_both();
            reset_integrators();
            direction.store(Direction::Stop);
            target_rpm.store(0);
            stall_ticks  = 0;
            active_ticks = 0;
            for (auto& c : controllers) {
                c.prev_count = (c.side == motor::Side::Left) ? count_l : count_r;
            }
            return;
        }
    }

    // We're actively driving. Count time since the most recent idle→active
    // edge so the stall watchdog can skip the spin-up window.
    ++active_ticks;

    // Sync-watchdog: gross divergence while *actively* controlling → fault,
    // brake once, latch. Only meaningful when the controller is driving;
    // if a user is poking the motors via the raw debug API, divergence is
    // expected and not a fault.
    const int sync_err = std::abs(count_l - count_r);
    if (sync_err > m.sync_fault_limit) {
        ESP_LOGW(TAG, "sync fault: |%ld - %ld| = %d > %d; braking",
                 static_cast<long>(count_l), static_cast<long>(count_r),
                 sync_err, m.sync_fault_limit);
        fault.store(true);
        direction.store(Direction::Stop);
        brake_both();
        reset_integrators();
        stall_ticks = 0;
        return;
    }

    // Stall-watchdog: both encoders effectively frozen while we're commanding
    // motion = we've hit a hard stop (top endpoint, jam, blind hit a wall).
    // Don't burn the motor against it. Skip during the startup grace window
    // (motor stiction + PI integral ramp-up takes a moment before counts
    // start accumulating). Compute deltas here before the PI loop
    // overwrites prev_count.
    const std::int32_t delta_l = count_l - controllers[0].prev_count;
    const std::int32_t delta_r = count_r - controllers[1].prev_count;
    const int startup_grace_ticks = m.startup_grace_ms / MS_PER_TICK;
    const int stall_fault_ticks   = m.stall_fault_ms / MS_PER_TICK;
    if (active_ticks > startup_grace_ticks) {
        if (std::abs(delta_l) <= m.stall_delta_max && std::abs(delta_r) <= m.stall_delta_max) {
            if (++stall_ticks >= stall_fault_ticks) {
                ESP_LOGW(TAG, "stall fault: both motors stopped %d ms at target %d RPM",
                         m.stall_fault_ms, base_setpoint_rpm);
                fault.store(true);
                direction.store(Direction::Stop);
                brake_both();
                reset_integrators();
                stall_ticks  = 0;
                active_ticks = 0;
                return;
            }
        } else {
            stall_ticks = 0;
        }
    }

    // Sign of expected count motion per direction. Used to flip the
    // measured-RPM and sync-bias terms so the PI sees them in the
    // commanded reference frame (always "positive = on target").
    // Raise drives raw::Forward (counts go negative), Lower drives
    // raw::Reverse (counts go positive). dir_sign projects measured RPM
    // and sync bias into the commanded reference frame ("positive = on
    // target") regardless of encoder count direction.
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

        // Static breakaway offset (direction-dependent) plus the slope term.
        const float ff_offset = (dir == Direction::Raise) ? m.ff_offset_raise_pct
                                                           : m.ff_offset_lower_pct;
        const float ff_duty = ff_offset + setpoint * m.duty_per_rpm;
        float duty          = ff_duty + m.kp * error + m.ki * c.i_accum;
        duty                = std::clamp(duty, 0.0f, 100.0f);

        motor::raw::drive(c.side, to_raw(dir), static_cast<int>(duty));
    }
}

// One tick of a position-seek: drive both motors toward target_counts (counts
// below the homed top) via the synced speed loop, ramping the setpoint down
// over the final revolution and braking once within tolerance. Reuses run_tick
// for the actual PI + sync + stall/sync watchdogs, so a hit on the bottom stop
// faults out exactly as a normal move would.
std::int32_t mm_to_counts(const config::Motion& m, float mm, std::int32_t fallback)
{
    if (m.mm_per_rev <= 0.0f) {
        return fallback;  // shouldn't happen — position mode requires calibration
    }
    return std::max<std::int32_t>(
        1, static_cast<std::int32_t>(mm * encoder::COUNTS_PER_OUTPUT_REV / m.mm_per_rev));
}

void run_position_tick(const config::Motion& m)
{
    const std::int32_t pos = (encoder::count(motor::Side::Left) +
                              encoder::count(motor::Side::Right)) / 2;
    const std::int32_t err = target_counts.load() - pos;  // >0 ⇒ need to go down

    const std::int32_t tol_counts  = mm_to_counts(m, m.goto_tol_mm, 24);
    const std::int32_t slow_counts = mm_to_counts(m, m.goto_slow_mm, 896);

    if (std::abs(err) <= tol_counts) {
        motor::raw::drive(motor::Side::Both, motor::raw::Direction::Brake, 0);
        reset_integrators();
        position_mode.store(false);
        arrived.store(true);
        return;
    }

    // Down (lower) increases counts; up (raise) decreases them.
    const Direction dir = (err > 0) ? Direction::Lower : Direction::Raise;

    // Ease the speed setpoint down over the slow-down zone for a soft landing,
    // flooring at goto_min_rpm so it keeps creeping rather than stalling short.
    int setpoint = m.cover_rpm;
    const std::int32_t mag = std::abs(err);
    if (mag < slow_counts) {
        setpoint = std::max(m.goto_min_rpm,
                            static_cast<int>(m.cover_rpm * mag / slow_counts));
    }

    run_tick(m, dir, setpoint);
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

        // Faulted state is latched — the fault path inside run_tick (or the
        // sync watchdog) already braked once. Skip ticks until stop()
        // clears the latch.
        if (fault.load()) {
            continue;
        }

        if (position_mode.load()) {
            run_position_tick(tune);
        } else {
            run_tick(tune, direction.load(), target_rpm.load());
        }
    }
}

}  // namespace

void set_target(int rpm, Direction d)
{
    if (fault.load()) {
        ESP_LOGW(TAG, "set_target ignored — controller faulted; call stop() to clear");
        return;
    }
    if (rpm < 0) rpm = 0;
    target_rpm.store(rpm);
    direction.store(d == Direction::Stop ? Direction::Stop : d);
    ESP_LOGI(TAG, "target = %d RPM, dir = %s",
             rpm,
             d == Direction::Raise ? "raise" :
             d == Direction::Lower ? "lower" : "stop");
}

void stop()
{
    target_rpm.store(0);
    direction.store(Direction::Stop);
    position_mode.store(false);
    fault.store(false);
    // The control task picks up the new state on the next tick; brake
    // immediately here so the motors don't keep coasting through that
    // (≤10 ms) window.
    motor::raw::drive(motor::Side::Both, motor::raw::Direction::Brake, 0);
}

HomeResult home()
{
    const auto tune = config::get().motion;
    const int  duty          = std::clamp(tune.home_duty_pct, 0, 100);
    const int  settle_ticks  = std::max(1, tune.home_settle_ms / MS_PER_TICK);
    const int  timeout_ticks = std::max(1, tune.home_timeout_s * 1000 / MS_PER_TICK);
    const int  stopped_max   = tune.stall_delta_max;

    // Take the motors away from the control task and clear any latched fault so
    // homing works even after a sync/stall fault.
    homing.store(true);
    target_rpm.store(0);
    direction.store(Direction::Stop);
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
                    motor::raw::drive(m.side, motor::raw::Direction::Brake, 0);
                    encoder::reset(m.side);
                    m.done = true;
                    ESP_LOGI(TAG, "homing: %s settled at top", m.label);
                    continue;
                }
            } else {
                m.stopped = 0;
            }

            motor::raw::drive(m.side, to_raw(Direction::Raise), duty);
        }

        if (all_done) {
            break;
        }
    }

    // Brake both and hand the motors back to the (idle) control task.
    motor::raw::drive(motor::Side::Both, motor::raw::Direction::Brake, 0);
    reset_integrators();
    homing.store(false);

    const HomeResult result{ mtrs[0].done, mtrs[1].done };
    // A valid zero reference exists only if both sides actually settled at the
    // top (encoders were zeroed there).
    homed.store(result.left && result.right);
    ESP_LOGI(TAG, "homing done: L=%s R=%s",
             result.left ? "ok" : "TIMEOUT", result.right ? "ok" : "TIMEOUT");
    return result;
}

GoToResult go_to_pct(float pct)
{
    pct = std::clamp(pct, 0.0f, 100.0f);
    const float hard = config::get().motion.hard_stop_mm;
    // 100% maps to the hard stop; go_to_mm then clamps to the soft stop.
    return go_to_mm(pct / 100.0f * hard);
}

bool is_homed()
{
    return homed.load();
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

bool begin_go_to_mm(float mm)
{
    const auto tune = config::get().motion;
    if (!homed.load() || tune.mm_per_rev <= 0.0f) {
        return false;
    }

    mm = std::clamp(mm, 0.0f, down_limit_mm(tune));  // top .. soft/hard down limit
    const auto target = static_cast<std::int32_t>(
        mm * static_cast<float>(encoder::COUNTS_PER_OUTPUT_REV) / tune.mm_per_rev);

    // Clean slate, then hand the seek to the control task.
    fault.store(false);
    reset_integrators();
    arrived.store(false);
    target_counts.store(target);
    position_mode.store(true);

    ESP_LOGI(TAG, "go_to %.1f mm (target %ld counts)", mm, static_cast<long>(target));
    return true;
}

bool begin_go_to_pct(float pct)
{
    pct = std::clamp(pct, 0.0f, 100.0f);
    return begin_go_to_mm(pct / 100.0f * config::get().motion.hard_stop_mm);
}

GoToResult go_to_mm(float mm)
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

    begin_go_to_mm(mm);  // validated above, so this starts the seek

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
    motor::raw::drive(motor::Side::Both, motor::raw::Direction::Brake, 0);

    const GoToStatus st = arrived.load() ? GoToStatus::Arrived
                        : fault.load()   ? GoToStatus::Faulted
                                         : GoToStatus::Timeout;
    const auto r = result(st);
    ESP_LOGI(TAG, "go_to done: status=%d L=%.1fmm R=%.1fmm",
             static_cast<int>(st), r.mm_l, r.mm_r);
    return r;
}

void start()
{
    BaseType_t ok = xTaskCreate(&control_task, "motion", STACK_SZ,
                                nullptr, TASK_PRIO, nullptr);
    configASSERT(ok == pdPASS);
    ESP_LOGI(TAG, "control loop started at %d Hz", CONTROL_HZ);
}

}  // namespace hvmrf01::motion
