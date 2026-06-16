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

        const float ff_duty = setpoint * m.duty_per_rpm;
        float duty          = ff_duty + m.kp * error + m.ki * c.i_accum;
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

    // Snapshot the tuning config locally so the 100 Hz loop never reads the
    // shared config concurrently with a writer. Re-snapshot only when the
    // generation counter bumps (a save() published new values) — lets the
    // CLI retune gains live without a reboot.
    config::Motion tune = config::get().motion;
    std::uint32_t  tune_gen = config::generation();

    TickType_t last_wake = xTaskGetTickCount();
    while (true) {
        vTaskDelayUntil(&last_wake, TICK);

        if (const auto g = config::generation(); g != tune_gen) {
            tune     = config::get().motion;
            tune_gen = g;
        }

        const auto dir = direction.load();
        const auto rpm = target_rpm.load();

        // Faulted state is latched — the fault path inside run_tick (or the
        // sync watchdog) already braked once. Skip ticks until stop()
        // clears the latch.
        if (fault.load()) {
            continue;
        }

        run_tick(tune, dir, rpm);
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

}  // namespace hvmrf01::motion
