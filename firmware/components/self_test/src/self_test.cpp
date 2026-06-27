#include "hv-mrf-01/self_test.hpp"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <string_view>
#include <utility>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "hv-mrf-01/current_sense.hpp"
#include "hv-mrf-01/encoder.hpp"
#include "hv-mrf-01/motion.hpp"
#include "hv-mrf-01/motor.hpp"

namespace hvmrf01::self_test {

namespace {

using hvmrf01::motor::Mode;
using hvmrf01::motor::Side;

// Thresholds. All values assume the board's 896 cpr / 32:1 gearbox, run on the
// bench with no blind mechanism attached — validate against a few known-good
// boards before trusting them as a production gate (run-current bands in
// particular shift a lot once the mechanism loads the motor).

constexpr int SPIN_DUTY = 50;  // shared drive duty for the spin + brake checks

// As-built PCB wiring (documented in motion.cpp): Mode::Forward winds the cord
// up and produces NEGATIVE encoder counts on both motors; Mode::Reverse
// produces positive. The closed-loop controller's dir_sign depends on this
// exact polarity, so the self-test asserts it — a board that counts the other
// way would make the speed loop run away, not just read "backwards".
constexpr std::int32_t FORWARD_SIGN = -1;

// idle_quiescent — pre-drive safety gate + current/encoder baseline
constexpr std::int32_t IDLE_MAX_MA    = 25;  // a quiet braked bridge draws ~0
constexpr std::int32_t IDLE_DRIFT_MAX = 4;   // at-rest PCNT counts over 100 ms

// forward_spin / reverse_spin
constexpr int          SPIN_SETTLE_MS  = 150;   // discard inrush + mechanical lag
constexpr int          SPIN_WIN_MS     = 400;   // measurement window
constexpr int          SPIN_SAMPLE_MS  = 20;    // current sample period
constexpr std::int32_t MIN_MOVE_COUNTS = 200;   // ~10x jitter, < quarter-rev seen
constexpr std::int32_t MIN_RUN_MA      = 15;    // no-load floor: forward current
                                                // jitters ~24-50 mA across boards
                                                // (IPROPI reads lower in the forward
                                                // FET pattern), so this only needs to
                                                // flag a dead IPROPI/ADC (~0 mA while
                                                // the encoder still turns) — motion
                                                // itself is proven by the count check
constexpr std::int32_t MAX_RUN_MA      = 2000;  // above this (no stall commanded) = short/jam

// ramp_linearity — stepped duty ladder + integer Pearson r
constexpr int          RAMP_STEPS       = 11;   // duty 0,10,...,100%
constexpr int          RAMP_SETTLE_MS   = 120;  // per-step accel-transient discard
constexpr int          RAMP_WIN_MS      = 180;  // per-step steady-state velocity window
constexpr std::int32_t RAMP_DEADBAND_V  = 15;   // ~2x jitter; leading steps below excised
constexpr int          RAMP_R_MIN_MILLI = 900;  // r >= 0.90
constexpr int          RAMP_MIN_FIT_PTS = 5;    // statistical meaningfulness
constexpr int          RAMP_MONO_NUM    = 6;    // a step may not fall below
constexpr int          RAMP_MONO_DEN    = 10;   //   60% of the previous step
constexpr std::int32_t RAMP_VMAX_MIN    = 60;   // full-duty velocity floor

// brake_stop
constexpr int          BRAKE_SPINUP_MS   = 300;  // reach steady speed first
constexpr int          BRAKE_PREMOVE_MS  = 80;   // window to confirm it's moving
constexpr std::int32_t BRAKE_PREMOVE_MIN = 50;   // proves real motion pre-brake
constexpr int          BRAKE_GRACE_MS    = 80;   // let geared inertia bleed
constexpr int          BRAKE_WIN_MS      = 200;  // residual-motion window
constexpr std::int32_t BRAKE_RESIDUAL    = 12;   // ~0.013 rev; coasting would be 100s

// both_track — joint two-motor ramp
constexpr int          BOTH_RAMP_MS    = 1500;
constexpr int          BOTH_HOLD_MS    = 500;
constexpr int          BOTH_HZ         = 100;
constexpr std::int32_t BOTH_MIN_COUNTS = 400;   // each side must clearly move
constexpr std::int32_t BOTH_TRACK_PCT  = 25;    // open-loop pair-mismatch tolerance
constexpr std::int32_t BOTH_TRACK_ABS  = 448;   // half-rev floor for short moves
constexpr std::int32_t BOTH_MAX_MA     = 4000;  // both-channels short / brownout cap

const char* side_label(Side s)
{
    return s == Side::Left ? "L" : s == Side::Right ? "R" : "both";
}

// Index a per-side array (idle baselines). Side::Both folds to Left.
int side_index(Side s)
{
    return s == Side::Right ? 1 : 0;
}

// printf into a std::string — used to build the machine-parseable note field.
std::string fmt(const char* f, ...)
{
    va_list args;
    va_start(args, f);

    char    buf[128];
    va_list copy;
    va_copy(copy, args);
    const int n = std::vsnprintf(buf, sizeof(buf), f, copy);
    va_end(copy);

    std::string out;
    if (n >= 0 && static_cast<std::size_t>(n) < sizeof(buf)) {
        out.assign(buf, static_cast<std::size_t>(n));
    } else if (n >= 0) {
        out.resize(static_cast<std::size_t>(n));
        std::vsnprintf(out.data(), out.size() + 1, f, args);
    }

    va_end(args);
    return out;
}

Result pass_result(const char* name, std::string detail)
{
    return {name, true, std::move(detail)};
}

Result fail_result(const char* name, const char* reason, const std::string& detail)
{
    return {name, false, "reason=" + std::string(reason) + " " + detail};
}

// Mean motor current over a window, in mA. Samples on `period_ms`.
std::int32_t avg_current_ma(Side s, int window_ms, int period_ms)
{
    const int    steps = std::max(1, window_ms / period_ms);
    std::int64_t sum   = 0;
    for (int i = 0; i < steps; i++) {
        vTaskDelay(pdMS_TO_TICKS(period_ms));
        sum += hvmrf01::current_sense::current_ma(s);
    }
    return static_cast<std::int32_t>(sum / steps);
}

// Drive one motor in a direction at SPIN_DUTY, discard the startup transient,
// then measure the encoder delta and offset-corrected mean current over the
// measurement window. Brakes the motor before returning. Returns the signed
// encoder delta; writes the corrected mean current to avg_ma_out.
std::int32_t spin_measure(Side s, Mode mode, std::int32_t idle_ma,
                          std::int32_t& avg_ma_out)
{
    hvmrf01::encoder::reset(s);
    hvmrf01::motor::drive(s, mode, SPIN_DUTY);
    vTaskDelay(pdMS_TO_TICKS(SPIN_SETTLE_MS));

    const auto c0  = hvmrf01::encoder::count(s);
    const auto raw = avg_current_ma(s, SPIN_WIN_MS, SPIN_SAMPLE_MS);
    const auto c1  = hvmrf01::encoder::count(s);

    hvmrf01::motor::drive(s, Mode::Brake, 0);

    avg_ma_out = raw > idle_ma ? raw - idle_ma : 0;
    return c1 - c0;
}

// Pre-drive safety gate and baseline. With the drivers awake and braked, no
// motor should draw current and no encoder should move. A bridge already
// sourcing current at rest is the documented nSLEEP/EN no-pulldown hazard; a
// drifting encoder would corrupt every count-based check downstream. Also
// records the per-side idle current used to offset-correct later readings.
Result check_idle_quiescent(std::int32_t idle_out[2])
{
    hvmrf01::motor::debug::set_enabled(true);
    hvmrf01::motor::debug::set_brake(Side::Both);
    vTaskDelay(pdMS_TO_TICKS(100));

    const auto l0 = hvmrf01::encoder::count(Side::Left);
    const auto r0 = hvmrf01::encoder::count(Side::Right);

    // The 100 ms current-averaging window doubles as the encoder-drift window.
    std::int64_t sum_l = 0, sum_r = 0;
    for (int i = 0; i < 10; i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
        sum_l += hvmrf01::current_sense::current_ma(Side::Left);
        sum_r += hvmrf01::current_sense::current_ma(Side::Right);
    }
    const std::int32_t idle_l = static_cast<std::int32_t>(sum_l / 10);
    const std::int32_t idle_r = static_cast<std::int32_t>(sum_r / 10);
    idle_out[0] = idle_l;
    idle_out[1] = idle_r;

    const auto drift_l = std::abs(hvmrf01::encoder::count(Side::Left) - l0);
    const auto drift_r = std::abs(hvmrf01::encoder::count(Side::Right) - r0);

    const auto detail = fmt("side=both idle_ma_l=%ld idle_ma_r=%ld drift_l=%ld drift_r=%ld",
                            static_cast<long>(idle_l), static_cast<long>(idle_r),
                            static_cast<long>(drift_l), static_cast<long>(drift_r));

    if (idle_l > IDLE_MAX_MA || idle_r > IDLE_MAX_MA) {
        return fail_result("idle_quiescent", "quiescent_high", detail);
    }
    if (drift_l > IDLE_DRIFT_MAX || drift_r > IDLE_DRIFT_MAX) {
        return fail_result("idle_quiescent", "encoder_noise", detail);
    }
    return pass_result("idle_quiescent", detail);
}

// Spin forward: the motor turns, the encoder counts up, and the current draw is
// in a plausible band. Exports the raw delta + current for encoder_wiring.
Result check_forward_spin(Side s, std::int32_t idle_ma,
                          std::int32_t& dcount_out, std::int32_t& ma_out)
{
    std::int32_t avg_ma = 0;
    const auto   dcount = spin_measure(s, Mode::Forward, idle_ma, avg_ma);
    dcount_out = dcount;
    ma_out     = avg_ma;

    const auto detail = fmt("side=%s dcount=%ld dir=fwd avg_ma=%ld",
                            side_label(s), static_cast<long>(dcount),
                            static_cast<long>(avg_ma));

    if (std::abs(dcount) < MIN_MOVE_COUNTS) return fail_result("forward_spin", "no_move", detail);
    if (dcount * FORWARD_SIGN < 0)          return fail_result("forward_spin", "wrong_sign", detail);
    if (avg_ma < MIN_RUN_MA)                return fail_result("forward_spin", "no_current", detail);
    if (avg_ma > MAX_RUN_MA)                return fail_result("forward_spin", "overcurrent", detail);
    return pass_result("forward_spin", detail);
}

// Spin reverse: same magnitude, opposite sign. A wrong sign with real motion is
// a swapped encoder A/B phase — the load-bearing assertion of this check.
Result check_reverse_spin(Side s, std::int32_t idle_ma,
                          std::int32_t& dcount_out, std::int32_t& ma_out)
{
    std::int32_t avg_ma = 0;
    const auto   dcount = spin_measure(s, Mode::Reverse, idle_ma, avg_ma);
    dcount_out = dcount;
    ma_out     = avg_ma;

    const auto detail = fmt("side=%s dcount=%ld dir=rev avg_ma=%ld",
                            side_label(s), static_cast<long>(dcount),
                            static_cast<long>(avg_ma));

    if (std::abs(dcount) < MIN_MOVE_COUNTS) return fail_result("reverse_spin", "no_move", detail);
    if (dcount * FORWARD_SIGN > 0)          return fail_result("reverse_spin", "wrong_sign", detail);
    if (avg_ma < MIN_RUN_MA)                return fail_result("reverse_spin", "no_current", detail);
    if (avg_ma > MAX_RUN_MA)                return fail_result("reverse_spin", "overcurrent", detail);
    return pass_result("reverse_spin", detail);
}

// Turn the forward + reverse results into an actionable wiring diagnosis. No
// extra motor time — it just classifies the deltas already measured.
Result check_encoder_wiring(Side s, std::int32_t fwd_dcount, std::int32_t fwd_ma,
                            std::int32_t rev_dcount, std::int32_t rev_ma)
{
    const bool fwd_moved   = std::abs(fwd_dcount) >= MIN_MOVE_COUNTS;
    const bool rev_moved   = std::abs(rev_dcount) >= MIN_MOVE_COUNTS;
    const bool current_ok  = fwd_ma >= MIN_RUN_MA || rev_ma >= MIN_RUN_MA;
    const bool opposite    = (fwd_dcount > 0) != (rev_dcount > 0);
    const bool fwd_correct = fwd_dcount * FORWARD_SIGN > 0;  // forward counts the spec way

    const char* cls;
    if (fwd_moved && rev_moved && opposite && fwd_correct) {
        cls = "ok";
    } else if (fwd_moved && rev_moved && opposite) {
        cls = "reversed";  // fwd/rev distinct but polarity inverted vs firmware (A/B or leads swapped)
    } else if (fwd_moved && rev_moved) {
        cls = "no_direction";  // counted the same sign both ways → not decoding direction
    } else if (!fwd_moved && !rev_moved && !current_ok) {
        cls = "dead";  // no counts and no current → motor/driver
    } else {
        cls = "disconnect";  // current flowed but the encoder didn't track it
    }

    const auto detail = fmt("side=%s class=%s", side_label(s), cls);
    if (std::string_view{cls} == "ok") {
        return pass_result("encoder_wiring", detail);
    }
    return fail_result("encoder_wiring", cls, detail);
}

// Pearson correlation (×1000) between commanded duty and steady-state velocity
// over the supra-deadband ladder steps, with (0,0) anchored to the origin.
// All accumulation is int64 (products stay < 1e8); one float divide+sqrt at the
// end. Also reports the fit-point count, deadband edge, and monotonicity.
int ramp_pearson_milli(const std::int32_t v[], int& fit_pts, int& k0_out, bool& mono_out)
{
    int k0 = 1;
    while (k0 < RAMP_STEPS && v[k0] < RAMP_DEADBAND_V) {
        ++k0;
    }
    k0_out = k0;

    // A step may not fall below MONO_FRAC of the best velocity seen so far —
    // comparing against the running max (not just the previous step) also
    // rejects a collapse-then-recover, and any negative (reverse) velocity
    // trivially fails it.
    bool         mono = true;
    std::int32_t peak = 0;
    for (int k = k0; k < RAMP_STEPS; k++) {
        if (v[k] * RAMP_MONO_DEN < RAMP_MONO_NUM * peak) {
            mono = false;
        }
        peak = std::max(peak, v[k]);
    }
    mono_out = mono;

    std::int64_t n = 0, sx = 0, sy = 0, sxx = 0, syy = 0, sxy = 0;
    const auto   add = [&](std::int64_t x, std::int64_t y) {
        ++n; sx += x; sy += y; sxx += x * x; syy += y * y; sxy += x * y;
    };
    add(0, 0);  // anchor: 0% duty must produce 0 velocity
    for (int k = k0; k < RAMP_STEPS; k++) {
        add(static_cast<std::int64_t>(k) * 10, v[k]);
    }
    fit_pts = static_cast<int>(n);

    const std::int64_t num  = n * sxy - sx * sy;
    const std::int64_t den2 = (n * sxx - sx * sx) * (n * syy - sy * sy);
    if (den2 <= 0) {
        return 0;
    }
    return static_cast<int>(std::llround(1000.0 * static_cast<double>(num) /
                                         std::sqrt(static_cast<double>(den2))));
}

// Ramp duty 0→100% as a steady-state ladder and verify velocity rises linearly
// with duty. Linear here means a high Pearson r AND a monotone climb AND the
// motor genuinely reached speed at full duty — so a flat/saturating/noise-only
// response can't pass on correlation alone.
Result check_ramp_linearity(Side s)
{
    hvmrf01::encoder::reset(s);
    hvmrf01::motor::drive(s, Mode::Forward, 0);

    std::int32_t v[RAMP_STEPS];
    for (int k = 0; k < RAMP_STEPS; k++) {
        hvmrf01::motor::debug::set_duty_pct(k * 10, s);
        vTaskDelay(pdMS_TO_TICKS(RAMP_SETTLE_MS));
        const auto c0 = hvmrf01::encoder::count(s);
        vTaskDelay(pdMS_TO_TICKS(RAMP_WIN_MS));
        // Project onto the forward direction so velocity is positive as duty
        // rises (Forward counts negative on this board — see FORWARD_SIGN).
        v[k] = (hvmrf01::encoder::count(s) - c0) * FORWARD_SIGN;
    }
    hvmrf01::motor::drive(s, Mode::Brake, 0);

    int  fit_pts = 0, k0 = 0;
    bool mono    = false;
    const int          r_milli = ramp_pearson_milli(v, fit_pts, k0, mono);
    const std::int32_t vmax    = v[RAMP_STEPS - 1];

    const auto detail = fmt("side=%s r=%d n=%d k0=%d vmax=%ld mono=%d",
                            side_label(s), r_milli, fit_pts, k0,
                            static_cast<long>(vmax), mono ? 1 : 0);

    if (vmax < RAMP_VMAX_MIN)       return fail_result("ramp_linearity", "no_speed", detail);
    if (fit_pts < RAMP_MIN_FIT_PTS) return fail_result("ramp_linearity", "too_few_points", detail);
    if (!mono)                      return fail_result("ramp_linearity", "non_monotonic", detail);
    if (r_milli < RAMP_R_MIN_MILLI) return fail_result("ramp_linearity", "not_linear", detail);
    return pass_result("ramp_linearity", detail);
}

// Drive at 50%, confirm the shaft is actually spinning, then active-brake and
// confirm it stops (residual ≈ 0) rather than coasting.
Result check_brake_stop(Side s)
{
    hvmrf01::encoder::reset(s);
    hvmrf01::motor::drive(s, Mode::Forward, SPIN_DUTY);
    vTaskDelay(pdMS_TO_TICKS(BRAKE_SPINUP_MS));

    const auto p0 = hvmrf01::encoder::count(s);
    vTaskDelay(pdMS_TO_TICKS(BRAKE_PREMOVE_MS));
    const auto pre_move = std::abs(hvmrf01::encoder::count(s) - p0);

    hvmrf01::motor::drive(s, Mode::Brake, 0);
    vTaskDelay(pdMS_TO_TICKS(BRAKE_GRACE_MS));
    const auto b0 = hvmrf01::encoder::count(s);
    vTaskDelay(pdMS_TO_TICKS(BRAKE_WIN_MS));
    const auto settle = std::abs(hvmrf01::encoder::count(s) - b0);

    const auto detail = fmt("side=%s pre_move=%ld settle=%ld",
                            side_label(s), static_cast<long>(pre_move),
                            static_cast<long>(settle));

    if (pre_move < BRAKE_PREMOVE_MIN) return fail_result("brake_stop", "no_spin", detail);
    if (settle > BRAKE_RESIDUAL)      return fail_result("brake_stop", "still_moving", detail);
    return pass_result("brake_stop", detail);
}

// Ramp both motors together and verify both move, track each other within an
// (open-loop, generous) tolerance, and the combined draw stays bounded. Catches
// faults that only appear with both bridges active: shared-nSLEEP glitches,
// supply sag, a cross-wired pair, or one materially weaker motor.
Result check_both_track(const std::int32_t idle[2])
{
    hvmrf01::encoder::reset(Side::Both);

    const int        total_ms = BOTH_RAMP_MS + BOTH_HOLD_MS;
    const int        samples  = total_ms * BOTH_HZ / 1000;
    const TickType_t period   = pdMS_TO_TICKS(1000 / BOTH_HZ);

    std::int32_t max_skew    = 0;
    std::int32_t peak_sum_ma = 0;

    const auto t0   = xTaskGetTickCount();
    TickType_t wake = t0;
    for (int i = 0; i < samples; i++) {
        const long t_ms = static_cast<long>(pdTICKS_TO_MS(xTaskGetTickCount() - t0));
        const int  duty = (t_ms >= BOTH_RAMP_MS) ? 100
                        : static_cast<int>(100L * t_ms / BOTH_RAMP_MS);
        hvmrf01::motor::drive(Side::Left, Mode::Forward, duty);
        hvmrf01::motor::drive(Side::Right, Mode::Forward, duty);

        const auto skew = std::abs(hvmrf01::encoder::count(Side::Left) -
                                   hvmrf01::encoder::count(Side::Right));
        max_skew = std::max(max_skew, skew);

        const auto lma = std::max<std::int32_t>(
            0, hvmrf01::current_sense::current_ma(Side::Left) - idle[0]);
        const auto rma = std::max<std::int32_t>(
            0, hvmrf01::current_sense::current_ma(Side::Right) - idle[1]);
        peak_sum_ma = std::max(peak_sum_ma, lma + rma);

        vTaskDelayUntil(&wake, period);
    }

    hvmrf01::motor::drive(Side::Both, Mode::Brake, 0);

    const auto cl  = std::abs(hvmrf01::encoder::count(Side::Left));
    const auto cr  = std::abs(hvmrf01::encoder::count(Side::Right));
    const auto tol = std::max<std::int32_t>(BOTH_TRACK_ABS,
                                            std::max(cl, cr) * BOTH_TRACK_PCT / 100);

    const auto detail = fmt("side=both dcount_l=%ld dcount_r=%ld max_skew=%ld peak_sum_ma=%ld",
                            static_cast<long>(cl), static_cast<long>(cr),
                            static_cast<long>(max_skew),
                            static_cast<long>(peak_sum_ma));

    if (cl < BOTH_MIN_COUNTS || cr < BOTH_MIN_COUNTS) {
        return fail_result("both_track", "one_side_dead", detail);
    }
    if (std::abs(cl - cr) > tol) {
        return fail_result("both_track", "mismatch", detail);
    }
    if (peak_sum_ma > BOTH_MAX_MA) {
        return fail_result("both_track", "dual_overcurrent", detail);
    }
    return pass_result("both_track", detail);
}

void record(SelfTestResult& out, ProgressFn cb, Result r)
{
    if (cb != nullptr) {
        cb(r);
    }
    out.results.push_back(std::move(r));
}

// The per-motor battery for one side. forward + reverse results feed the
// derived encoder_wiring diagnosis; every check still runs for a full picture.
void run_side(Side s, const std::int32_t idle[2], SelfTestResult& out, ProgressFn cb)
{
    const std::int32_t idle_ma = idle[side_index(s)];

    std::int32_t fwd_dcount = 0, fwd_ma = 0, rev_dcount = 0, rev_ma = 0;
    record(out, cb, check_forward_spin(s, idle_ma, fwd_dcount, fwd_ma));
    record(out, cb, check_reverse_spin(s, idle_ma, rev_dcount, rev_ma));
    record(out, cb, check_encoder_wiring(s, fwd_dcount, fwd_ma, rev_dcount, rev_ma));
    record(out, cb, check_ramp_linearity(s));
    record(out, cb, check_brake_stop(s));
}

}  // namespace

bool SelfTestResult::passed() const
{
    for (const auto& r : results) {
        if (!r.pass) return false;
    }
    return true;
}

int SelfTestResult::pass_count() const
{
    int n = 0;
    for (const auto& r : results) {
        if (r.pass) ++n;
    }
    return n;
}

int SelfTestResult::fail_count() const
{
    int n = 0;
    for (const auto& r : results) {
        if (!r.pass) ++n;
    }
    return n;
}

SelfTestResult run(Side side, ProgressFn on_result)
{
    // Own the motors for the whole test, like cmd_profile — otherwise the 100 Hz
    // controller fights every drive command.
    hvmrf01::motion::stop();
    hvmrf01::motor::debug::set_enabled(true);

    SelfTestResult out;

    std::int32_t idle[2] = {0, 0};
    const Result quiescent = check_idle_quiescent(idle);
    const bool   quiescent_ok = quiescent.pass;
    record(out, on_result, quiescent);

    // Hard safety gate: never energize a board whose bridge is already drawing
    // current at rest or whose encoder free-runs.
    if (!quiescent_ok) {
        hvmrf01::motor::debug::set_brake(Side::Both);
        return out;
    }

    if (side == Side::Both) {
        run_side(Side::Left, idle, out, on_result);
        run_side(Side::Right, idle, out, on_result);
        record(out, on_result, check_both_track(idle));
    } else {
        run_side(side, idle, out, on_result);
    }

    hvmrf01::motor::debug::set_brake(Side::Both);
    return out;
}

}  // namespace hvmrf01::self_test
