#include "blinds/motion.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "blinds/encoder.hpp"
#include "blinds/motor.hpp"

namespace blinds::motion {

namespace {

constexpr auto* TAG = "blinds.motion";

// ── Tuning constants (DESIGN.md "Closed-loop speed control") ──────────────
// Feedforward slope. Empirical iteration:
//   2.8 RPM/% duty no-load → DUTY_PER_RPM = 0.36
//   2.0 RPM/% duty light load → 0.50
//   1.0 RPM/% duty sheath + cord load → 1.0   ← current
// At 40 RPM target this starts FF at 40% duty, plus Kp/Ki for tracking.
// Re-fit once mechanicals are finalized via the CLI: drive a known RPM,
// note the steady-state duty in `state`, divide.
constexpr float DUTY_PER_RPM     = 1.0f;
constexpr float Kp               = 0.10f;   // % duty per RPM error
constexpr float Ki               = 0.50f;   // % duty per RPM-sec
// Anti-windup clamp on i_accum. Sized so the integrator can deliver up to
// ~50% duty on top of the feedforward — gives the controller headroom to
// overcome friction without ever pegging at the ceiling.
constexpr float I_MAX            = 100.0f;
constexpr float K_SYNC           = 0.05f;   // RPM bias per count of cross-error
constexpr int   SYNC_FAULT_LIMIT = 200;     // counts (~5 mm of cord)

// Stall detection: when both motors stop moving while we're commanding
// non-zero motion, we've hit a hard stop (top endpoint, jam, etc.). Fault
// out so we don't burn the motor against a wall. Skipped during a startup
// grace window so motor stiction + PI ramp-up don't false-fire it.
constexpr int   STALL_DELTA_MAX     = 1;    // |Δcount| per motor per tick at "stopped"
constexpr int   STALL_FAULT_MS      = 100;  // consecutive duration of stoppage
// Long enough to cover both motor stiction *and* integrator wind-up under
// real load. With Ki=0.5 and full error, integrator climbs ~20/sec — to
// add ~40% duty over feedforward takes ~2s. 1500ms is the floor; bump
// further if stalls fire spuriously on heavy-load starts.
constexpr int   STARTUP_GRACE_MS    = 1500;
constexpr int   CONTROL_HZ          = 100;
constexpr TickType_t TICK        = pdMS_TO_TICKS(1000 / CONTROL_HZ);
constexpr float DT_S             = 1.0f / static_cast<float>(CONTROL_HZ);

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

// Consecutive ticks where both motors registered near-zero count delta
// while we're commanding motion. Resets when motion is detected.
int stall_ticks = 0;
// Ticks elapsed since motion became active (since last idle→active edge).
// Stall watchdog ignores readings until this exceeds the grace window.
int active_ticks = 0;
constexpr int STALL_FAULT_TICKS   = STALL_FAULT_MS    / (1000 / CONTROL_HZ);
constexpr int STARTUP_GRACE_TICKS = STARTUP_GRACE_MS  / (1000 / CONTROL_HZ);

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
    // Wiring convention: motor::raw::Reverse spins the motors in the
    // physical "wind cord up" direction (i.e. raise the blind), so cover
    // Open ⇒ Raise maps to Reverse here. The encoder convention is
    // unchanged (Forward still produces positive counts), so dir_sign
    // below flips the PI's idea of "positive motion" to match.
    switch (d) {
    case Direction::Raise: return motor::raw::Direction::Reverse;
    case Direction::Lower: return motor::raw::Direction::Forward;
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

// Sign convention for the speed loop: under Raise, both motors should
// produce positive count delta; under Lower, negative. We pass the
// "expected sign of count motion" into the loop so the same code handles
// both directions. The PI controller operates on magnitudes (always
// non-negative); the H-bridge handles direction via to_raw().
void run_tick(Direction dir, int base_setpoint_rpm)
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

    // We're actively driving. Count time since the most recent idle→active
    // edge so the stall watchdog can skip the spin-up window.
    ++active_ticks;

    // Sync-watchdog: gross divergence while *actively* controlling → fault,
    // brake once, latch. Only meaningful when the controller is driving;
    // if a user is poking the motors via the raw debug API, divergence is
    // expected and not a fault.
    const int sync_err = std::abs(count_l - count_r);
    if (sync_err > SYNC_FAULT_LIMIT) {
        ESP_LOGW(TAG, "sync fault: |%ld - %ld| = %d > %d; braking",
                 static_cast<long>(count_l), static_cast<long>(count_r),
                 sync_err, SYNC_FAULT_LIMIT);
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
    if (active_ticks > STARTUP_GRACE_TICKS) {
        if (std::abs(delta_l) <= STALL_DELTA_MAX && std::abs(delta_r) <= STALL_DELTA_MAX) {
            if (++stall_ticks >= STALL_FAULT_TICKS) {
                ESP_LOGW(TAG, "stall fault: both motors stopped %d ms at target %d RPM",
                         STALL_FAULT_MS, base_setpoint_rpm);
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
    // Raise drives raw::Reverse (counts go negative), Lower drives
    // raw::Forward (counts go positive). dir_sign projects measured RPM
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
        const float bias = K_SYNC * dir_sign *
                           static_cast<float>(count_other - count_me);

        const float setpoint = static_cast<float>(base_setpoint_rpm) + bias;
        const float error    = setpoint - measured;

        c.i_accum = std::clamp(c.i_accum + error * DT_S, -I_MAX, I_MAX);

        const float ff_duty = setpoint * DUTY_PER_RPM;
        float duty          = ff_duty + Kp * error + Ki * c.i_accum;
        duty                = std::clamp(duty, 0.0f, 100.0f);

        motor::raw::drive(c.side, to_raw(dir), static_cast<int>(duty));
    }
}

void control_task(void*)
{
    // Initialize prev_count to the current readings so the first tick's
    // delta isn't a huge spurious value if the encoders have drifted
    // since boot.
    controllers[0].prev_count = encoder::count(motor::Side::Left);
    controllers[1].prev_count = encoder::count(motor::Side::Right);

    TickType_t last_wake = xTaskGetTickCount();
    while (true) {
        vTaskDelayUntil(&last_wake, TICK);

        const auto dir = direction.load();
        const auto rpm = target_rpm.load();

        // Faulted state is latched — the fault path inside run_tick (or the
        // sync watchdog) already braked once. Skip ticks until stop()
        // clears the latch.
        if (fault.load()) {
            continue;
        }

        run_tick(dir, rpm);
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
    fault.store(false);
    // The control task picks up the new state on the next tick; brake
    // immediately here so the motors don't keep coasting through that
    // (≤10 ms) window.
    motor::raw::drive(motor::Side::Both, motor::raw::Direction::Brake, 0);
}

void start()
{
    BaseType_t ok = xTaskCreate(&control_task, "motion", STACK_SZ,
                                nullptr, TASK_PRIO, nullptr);
    configASSERT(ok == pdPASS);
    ESP_LOGI(TAG, "control loop started at %d Hz", CONTROL_HZ);
}

}  // namespace blinds::motion
