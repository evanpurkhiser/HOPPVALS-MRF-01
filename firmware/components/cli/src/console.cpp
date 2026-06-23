#include "hv-mrf-01/console.hpp"

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <string_view>

#include "esp_console.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "hv-mrf-01/config.hpp"
#include "hv-mrf-01/current_sense.hpp"
#include "hv-mrf-01/encoder.hpp"
#include "hv-mrf-01/motion.hpp"
#include "hv-mrf-01/motor.hpp"

namespace hvmrf01::console {

namespace {

constexpr auto* TAG = "hv-mrf-01.console";

// ── Output capture ────────────────────────────────────────────────────────
//
// Commands print via emit() rather than printf(). Normally that goes straight
// to stdout (the USB-JTAG REPL). While run_line() executes a command on behalf
// of a remote transport, it points capture_buf at a string so the output is
// collected and returned instead.
//
// capture_buf is thread_local: the target belongs to whichever task is running
// run_line(), so a command dispatched on one task can never redirect output
// that another task is producing. capture_mutex separately serializes
// run_line() itself, because esp_console_run() parses into shared internal
// state and is not re-entrant.
std::mutex                capture_mutex;
thread_local std::string* capture_buf = nullptr;

// printf-compatible output for command implementations. Captured to a string
// during remote dispatch, otherwise written to stdout.
int emit(const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);

    if (capture_buf == nullptr) {
        const int n = std::vprintf(fmt, args);
        va_end(args);
        return n;
    }

    char    stack[256];
    va_list copy;
    va_copy(copy, args);
    const int n = std::vsnprintf(stack, sizeof(stack), fmt, copy);
    va_end(copy);

    if (n >= 0 && static_cast<std::size_t>(n) < sizeof(stack)) {
        capture_buf->append(stack, static_cast<std::size_t>(n));
    } else if (n >= 0) {
        // Output didn't fit the stack buffer; render it exactly once more into
        // a right-sized heap buffer.
        std::string big(static_cast<std::size_t>(n) + 1, '\0');
        std::vsnprintf(big.data(), big.size(), fmt, args);
        capture_buf->append(big.data(), static_cast<std::size_t>(n));
    }

    va_end(args);
    return n;
}

// Parse an optional "L" / "R" / "both" arg; default to Both.
hvmrf01::motor::Side parse_side(int argc, char** argv, int idx)
{
    if (argc <= idx) return hvmrf01::motor::Side::Both;
    const std::string_view a{argv[idx]};
    if (a == "L" || a == "l" || a == "left")  return hvmrf01::motor::Side::Left;
    if (a == "R" || a == "r" || a == "right") return hvmrf01::motor::Side::Right;
    return hvmrf01::motor::Side::Both;
}

const char* side_label(hvmrf01::motor::Side s)
{
    using S = hvmrf01::motor::Side;
    return s == S::Left ? "L" : s == S::Right ? "R" : "both";
}

int cmd_fwd(int argc, char** argv)
{
    const auto s = parse_side(argc, argv, 1);
    hvmrf01::motor::debug::set_forward(s);
    emit("→ %s forward at current duty\n", side_label(s));
    return 0;
}

int cmd_rev(int argc, char** argv)
{
    const auto s = parse_side(argc, argv, 1);
    hvmrf01::motor::debug::set_reverse(s);
    emit("→ %s reverse at current duty\n", side_label(s));
    return 0;
}

int cmd_brake(int argc, char** argv)
{
    const auto s = parse_side(argc, argv, 1);
    hvmrf01::motor::debug::set_brake(s);
    emit("→ %s brake (EN=0, active hold)\n", side_label(s));
    return 0;
}

int cmd_coast(int, char**)
{
    // Coast is the shared nSLEEP going low — inherently both motors.
    hvmrf01::motor::debug::set_coast();
    emit("→ coast (nSLEEP low, both motors Hi-Z)\n");
    return 0;
}

// Drive the shared driver-enable (nSLEEP) line. on = both drivers awake
// (required before drive/brake do anything); off = both coast.
int cmd_enable(int argc, char** argv)
{
    if (argc < 2) {
        emit("driver enable (nSLEEP) = %s\n",
               hvmrf01::motor::debug::enabled() ? "on" : "off");
        return 0;
    }
    const std::string_view a{argv[1]};
    const bool on = (a == "on" || a == "1" || a == "true");
    const bool off = (a == "off" || a == "0" || a == "false");
    if (!on && !off) {
        emit("usage: enable [on|off]\n");
        return 1;
    }
    hvmrf01::motor::debug::set_enabled(on);
    emit("driver enable (nSLEEP) = %s\n", on ? "on" : "off");
    return 0;
}

int cmd_pwm(int argc, char** argv)
{
    if (argc < 2) {
        emit("usage: pwm <0-100> [L|R]\n");
        return 1;
    }
    const int pct = std::atoi(argv[1]);
    if (pct < 0 || pct > 100) {
        emit("out of range: 0-100\n");
        return 1;
    }
    const auto s = parse_side(argc, argv, 2);
    hvmrf01::motor::debug::set_duty_pct(pct, s);
    emit("duty = %d%% (%s)\n", pct, side_label(s));
    return 0;
}

int cmd_freq(int argc, char** argv)
{
    if (argc != 2) {
        emit("usage: freq <hz>\n");
        return 1;
    }
    const int hz = std::atoi(argv[1]);
    if (!hvmrf01::motor::debug::set_freq_hz(hz)) {
        emit("freq %d Hz rejected (try 100..200000)\n", hz);
        return 1;
    }
    emit("freq = %d Hz\n", hz);
    return 0;
}

int cmd_state(int, char**)
{
    hvmrf01::motor::debug::print_state();
    return 0;
}

// Encoder: read count, reset, or stream live counts. Side defaults to L
// for the single-arg / poll forms; reset with no side resets both.
//   enc                  → print both motors' counts
//   enc [L|R]            → print one motor's count
//   enc reset [L|R]      → zero one (or both, default) encoders
//   enc poll [n] [L|R]   → stream count every 200ms for n samples (default 25)
int cmd_enc(int argc, char** argv)
{
    if (argc == 1) {
        emit("L=%ld R=%ld\n",
               static_cast<long>(hvmrf01::encoder::count(hvmrf01::motor::Side::Left)),
               static_cast<long>(hvmrf01::encoder::count(hvmrf01::motor::Side::Right)));
        return 0;
    }
    if (std::string_view{argv[1]} == "reset") {
        const auto s = parse_side(argc, argv, 2);
        hvmrf01::encoder::reset(s);
        emit("reset %s\n", side_label(s));
        return 0;
    }
    if (std::string_view{argv[1]} == "poll") {
        const int samples = argc >= 3 ? std::atoi(argv[2]) : 25;
        const auto side   = parse_side(argc, argv, 3);
        // Side::Both isn't meaningful here — pick Left.
        const auto s      = side == hvmrf01::motor::Side::Both ? hvmrf01::motor::Side::Left : side;
        std::int32_t prev  = hvmrf01::encoder::count(s);
        const auto   start = xTaskGetTickCount();
        for (int i = 0; i < samples; i++) {
            vTaskDelay(pdMS_TO_TICKS(200));
            const auto now      = hvmrf01::encoder::count(s);
            const auto delta    = now - prev;
            const auto cps      = delta * 5;  // 200ms window → counts/sec
            prev                = now;
            const auto elapsed  = pdTICKS_TO_MS(xTaskGetTickCount() - start);
            emit("[%s] t=%lums  count=%ld  Δ=%ld  cps=%ld\n",
                   side_label(s),
                   static_cast<unsigned long>(elapsed),
                   static_cast<long>(now), static_cast<long>(delta),
                   static_cast<long>(cps));
        }
        return 0;
    }
    // Single-arg side ("enc L" / "enc R").
    const auto s = parse_side(argc, argv, 1);
    if (s == hvmrf01::motor::Side::Both) {
        emit("usage: enc | enc [L|R] | enc reset [L|R] | enc poll [samples] [L|R]\n");
        return 1;
    }
    emit("%s = %ld\n", side_label(s),
           static_cast<long>(hvmrf01::encoder::count(s)));
    return 0;
}

// Measure output-shaft RPM by sampling the encoder count over a 1s window.
int cmd_rpm(int argc, char** argv)
{
    const auto side = parse_side(argc, argv, 1);
    const auto s    = side == hvmrf01::motor::Side::Both ? hvmrf01::motor::Side::Left : side;
    const auto start = hvmrf01::encoder::count(s);
    vTaskDelay(pdMS_TO_TICKS(1000));
    const auto end   = hvmrf01::encoder::count(s);
    const auto cps   = end - start;
    const auto rpm   = (cps * 60) / hvmrf01::encoder::COUNTS_PER_OUTPUT_REV;
    emit("[%s] Δcount=%ld cps=%ld output_rpm=%ld (motor_rpm=%ld)\n",
           side_label(s),
           static_cast<long>(cps), static_cast<long>(cps),
           static_cast<long>(rpm), static_cast<long>(rpm * 32));
    return 0;
}

// Read motor current draw from the DRV8876 IPROPI sense (sampled by the
// current_sense task and converted to mA). Defaults to both motors.
//   cur                  → print both motors' current
//   cur [L|R]            → print one motor
//   cur poll [n] [L|R]   → stream current every 200ms for n samples (default 25)
int cmd_current(int argc, char** argv)
{
    using hvmrf01::motor::Side;

    auto print_one = [](Side s) {
        emit("[%s] %ld mA  (%ld mV)\n", side_label(s),
               static_cast<long>(hvmrf01::current_sense::current_ma(s)),
               static_cast<long>(hvmrf01::current_sense::voltage_mv(s)));
    };

    if (argc == 1) {
        print_one(Side::Left);
        print_one(Side::Right);
        return 0;
    }
    if (std::string_view{argv[1]} == "poll") {
        const int  samples = argc >= 3 ? std::atoi(argv[2]) : 25;
        const auto side    = parse_side(argc, argv, 3);
        for (int i = 0; i < samples; i++) {
            if (side == Side::Both) {
                emit("L=%ld mA  R=%ld mA\n",
                       static_cast<long>(hvmrf01::current_sense::current_ma(Side::Left)),
                       static_cast<long>(hvmrf01::current_sense::current_ma(Side::Right)));
            } else {
                print_one(side);
            }
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        return 0;
    }
    const auto s = parse_side(argc, argv, 1);
    if (s == Side::Both) {
        print_one(Side::Left);
        print_one(Side::Right);
        return 0;
    }
    print_one(s);
    return 0;
}

// Bench-test a single motor open-loop: drive it at a fixed duty/direction for
// a fixed time while streaming measured RPM (from the encoder) and current
// draw, then brake. Fights the motion task — run `motion stop` first.
//   spin <L|R> <fwd|rev> <duty 0-100> [ms=1500]
int cmd_spin(int argc, char** argv)
{
    using hvmrf01::motor::Side;

    if (argc < 4) {
        emit("usage: spin <L|R> <fwd|rev> <duty 0-100> [ms=1500]\n");
        return 1;
    }
    const auto side = parse_side(argc, argv, 1);
    if (side == Side::Both) {
        emit("spin targets one motor — use L or R\n");
        return 1;
    }
    const std::string_view dir{argv[2]};
    const bool forward = (dir == "fwd" || dir == "f" || dir == "forward");
    const bool reverse = (dir == "rev" || dir == "r" || dir == "reverse");
    if (!forward && !reverse) {
        emit("direction must be fwd|rev\n");
        return 1;
    }
    const int duty = std::atoi(argv[3]);
    if (duty < 0 || duty > 100) {
        emit("duty out of range: 0-100\n");
        return 1;
    }
    int ms = argc > 4 ? std::atoi(argv[4]) : 1500;
    if (ms < 100)   ms = 100;
    if (ms > 10000) ms = 10000;

    constexpr int window_ms = 100;
    const int     steps     = ms / window_ms;

    emit("spin %s %s duty=%d%% for %d ms\n",
           side_label(side), forward ? "fwd" : "rev", duty, ms);
    emit("t_ms,rpm,current_ma\n");

    hvmrf01::motor::debug::set_duty_pct(duty, side);
    if (forward) hvmrf01::motor::debug::set_forward(side);
    else         hvmrf01::motor::debug::set_reverse(side);

    std::int32_t prev  = hvmrf01::encoder::count(side);
    const auto   t0    = xTaskGetTickCount();
    for (int i = 0; i < steps; i++) {
        vTaskDelay(pdMS_TO_TICKS(window_ms));
        const auto now   = hvmrf01::encoder::count(side);
        const auto delta = now - prev;
        prev             = now;
        const auto cps   = delta * (1000 / window_ms);
        const auto rpm   = (cps * 60) / hvmrf01::encoder::COUNTS_PER_OUTPUT_REV;
        const auto t_ms  = pdTICKS_TO_MS(xTaskGetTickCount() - t0);
        emit("%lu,%ld,%ld\n",
               static_cast<unsigned long>(t_ms),
               static_cast<long>(rpm),
               static_cast<long>(hvmrf01::current_sense::current_ma(side)));
    }

    hvmrf01::motor::debug::set_brake(side);
    emit("spin done; %s braked.\n", side_label(side));
    return 0;
}

// Open-loop ramp test: drive one motor from 0%→100% duty over `ramp_s`, then
// hold at 100% for `hold_s`, while streaming raw encoder count + current draw
// at `hz` in a CSV envelope. Host tooling differentiates count → velocity and
// acceleration and plots it alongside current. Fights the motion task — run
// `motion stop` first.
//   ramp <L|R> [ramp_s=3] [hold_s=1] [hz=100] [fwd|rev]
int cmd_ramp(int argc, char** argv)
{
    using hvmrf01::motor::Side;

    if (argc < 2) {
        emit("usage: ramp <L|R> [ramp_s=3] [hold_s=1] [hz=100] [fwd|rev]\n");
        return 1;
    }
    const auto side = parse_side(argc, argv, 1);
    if (side == Side::Both) {
        emit("ramp targets one motor — use L or R\n");
        return 1;
    }
    const int ramp_s = argc > 2 ? std::atoi(argv[2]) : 3;
    const int hold_s = argc > 3 ? std::atoi(argv[3]) : 1;
    const int hz     = argc > 4 ? std::atoi(argv[4]) : 100;

    bool forward = true;
    if (argc > 5) {
        const std::string_view d{argv[5]};
        if (d == "rev" || d == "r" || d == "reverse") {
            forward = false;
        } else if (!(d == "fwd" || d == "f" || d == "forward")) {
            emit("direction must be fwd|rev\n");
            return 1;
        }
    }
    if (ramp_s < 0 || hold_s < 0 || (ramp_s + hold_s) <= 0 ||
        (ramp_s + hold_s) > 30 || hz < 1 || hz > 500) {
        emit("usage: ramp <L|R> [ramp_s=3 (0..30 total)] [hold_s=1] [hz=100 (1..500)] [fwd|rev]\n");
        return 1;
    }

    const int  ramp_ms        = ramp_s * 1000;
    const int  total_ms       = (ramp_s + hold_s) * 1000;
    const int  total_samples  = total_ms * hz / 1000;
    const auto period         = pdMS_TO_TICKS(1000 / hz);

    hvmrf01::encoder::reset(side);
    if (forward) hvmrf01::motor::debug::set_forward(side);
    else         hvmrf01::motor::debug::set_reverse(side);

    emit("RAMP_BEGIN hz=%d ramp_s=%d hold_s=%d side=%s dir=%s\n",
           hz, ramp_s, hold_s, side_label(side), forward ? "fwd" : "rev");
    emit("t_ms,duty,count,current_ma\n");

    const auto t0   = xTaskGetTickCount();
    TickType_t wake = t0;
    int emitted = 0;
    for (int i = 0; i < total_samples; i++) {
        const long t_ms = static_cast<long>(pdTICKS_TO_MS(xTaskGetTickCount() - t0));
        const int  duty = (t_ms >= ramp_ms) ? 100
                        : static_cast<int>(100L * t_ms / ramp_ms);
        hvmrf01::motor::debug::set_duty_pct(duty, side);

        const auto count = hvmrf01::encoder::count(side);
        const auto cur   = hvmrf01::current_sense::current_ma(side);
        emit("%ld,%d,%ld,%ld\n", t_ms, duty,
               static_cast<long>(count), static_cast<long>(cur));
        emitted++;
        vTaskDelayUntil(&wake, period);
    }

    hvmrf01::motor::debug::set_brake(side);
    emit("RAMP_END samples=%d\n", emitted);
    return 0;
}

// Stream encoder counts at a fixed rate for a fixed duration, in a
// machine-parseable CSV envelope. Doesn't touch motors — the operator
// runs `motion` (or HA) concurrently and this just records.
//   trace [duration_s=3] [hz=100]
int cmd_trace(int argc, char** argv)
{
    constexpr int default_duration_s = 3;
    constexpr int default_hz         = 100;
    constexpr int max_duration_s     = 30;
    constexpr int max_hz             = 500;

    const int duration_s = (argc > 1) ? std::atoi(argv[1]) : default_duration_s;
    const int hz         = (argc > 2) ? std::atoi(argv[2]) : default_hz;

    if (duration_s <= 0 || duration_s > max_duration_s || hz < 1 || hz > max_hz) {
        emit("usage: trace [duration_s=3 (1..30)] [hz=100 (1..500)]\n");
        return 1;
    }

    const int    total_samples = duration_s * hz;
    const auto   period_ticks  = pdMS_TO_TICKS(1000 / hz);
    const auto   t0_ticks      = xTaskGetTickCount();
    TickType_t   wake          = t0_ticks;

    emit("TRACE_BEGIN hz=%d duration_s=%d\n", hz, duration_s);
    emit("t_ms,L,R\n");

    int emitted = 0;
    for (int i = 0; i < total_samples; i++) {
        const auto l = hvmrf01::encoder::count(hvmrf01::motor::Side::Left);
        const auto r = hvmrf01::encoder::count(hvmrf01::motor::Side::Right);
        const auto t_ms = pdTICKS_TO_MS(xTaskGetTickCount() - t0_ticks);
        emit("%lu,%ld,%ld\n",
               static_cast<unsigned long>(t_ms),
               static_cast<long>(l), static_cast<long>(r));
        emitted++;
        vTaskDelayUntil(&wake, period_ticks);
    }

    emit("TRACE_END samples=%d\n", emitted);
    return 0;
}

// Motion controller commands.
//   motion <rpm> raise|lower   → set_target
//   motion stop                → stop()
int cmd_motion(int argc, char** argv)
{
    if (argc >= 2 && std::string_view{argv[1]} == "stop") {
        hvmrf01::motion::stop();
        emit("motion: stopped\n");
        return 0;
    }
    if (argc < 3) {
        emit("usage: motion <rpm> raise|lower | motion stop\n");
        return 1;
    }
    const int rpm = std::atoi(argv[1]);
    if (rpm <= 0 || rpm > 300) {
        emit("rpm out of range (1..300)\n");
        return 1;
    }
    const std::string_view dir{argv[2]};
    hvmrf01::motion::Direction d;
    if (dir == "raise" || dir == "up" || dir == "open") {
        d = hvmrf01::motion::Direction::Raise;
    } else if (dir == "lower" || dir == "down" || dir == "close") {
        d = hvmrf01::motion::Direction::Lower;
    } else {
        emit("direction must be raise|lower\n");
        return 1;
    }
    hvmrf01::motion::set_target(rpm, d);
    emit("motion: target = %d RPM %s\n", rpm,
           d == hvmrf01::motion::Direction::Raise ? "raise" : "lower");
    return 0;
}

// Sweep PWM frequency while driving the motor so you can listen for the
// quietest band. Usage: sweep [start_hz] [end_hz] [step_hz] [dwell_ms]
int cmd_sweep(int argc, char** argv)
{
    const int start_hz = (argc > 1) ? std::atoi(argv[1]) : 5000;
    const int end_hz   = (argc > 2) ? std::atoi(argv[2]) : 50000;
    const int step_hz  = (argc > 3) ? std::atoi(argv[3]) : 1000;
    const int dwell_ms = (argc > 4) ? std::atoi(argv[4]) : 1500;

    if (step_hz <= 0 || end_hz <= start_hz) {
        emit("usage: sweep [start_hz=5000] [end_hz=50000] [step_hz=1000] [dwell_ms=1500]\n");
        return 1;
    }

    emit("sweep %d → %d Hz, step %d, dwell %d ms. Driving forward.\n",
           start_hz, end_hz, step_hz, dwell_ms);
    hvmrf01::motor::debug::set_forward();

    for (int hz = start_hz; hz <= end_hz; hz += step_hz) {
        if (!hvmrf01::motor::debug::set_freq_hz(hz)) {
            emit("  %d Hz rejected, stopping sweep\n", hz);
            break;
        }
        emit("  %d Hz\n", hz);
        vTaskDelay(pdMS_TO_TICKS(dwell_ms));
    }

    hvmrf01::motor::debug::set_brake();
    emit("sweep done; braked.\n");
    return 0;
}

// Working copy of the configuration that `config set` mutates and `config
// save` persists. Seeded from the live config on first use.
config::Config& work_config()
{
    static config::Config w = config::get();
    return w;
}

void print_config(const config::Config& c)
{
    emit("motion.duty_per_rpm = %.3f\n", c.motion.duty_per_rpm);
    emit("motion.ff_off_raise = %.2f\n", c.motion.ff_offset_raise_pct);
    emit("motion.ff_off_lower = %.2f\n", c.motion.ff_offset_lower_pct);
    emit("motion.ff_trim_l    = %.3f\n", c.motion.ff_trim_l);
    emit("motion.ff_trim_r    = %.3f\n", c.motion.ff_trim_r);
    emit("motion.kp           = %.3f\n", c.motion.kp);
    emit("motion.ki           = %.3f\n", c.motion.ki);
    emit("motion.i_max        = %.1f\n", c.motion.i_max);
    emit("motion.k_sync       = %.3f\n", c.motion.k_sync);
    emit("motion.cover_rpm    = %d\n", c.motion.cover_rpm);
    emit("motion.sync_fault   = %d\n", c.motion.sync_fault_limit);
    emit("motion.stall_delta  = %d\n", c.motion.stall_delta_max);
    emit("motion.stall_ms     = %d\n", c.motion.stall_fault_ms);
    emit("motion.grace_ms     = %d\n", c.motion.startup_grace_ms);
    emit("motion.home_duty    = %d\n", c.motion.home_duty_pct);
    emit("motion.home_settle  = %d\n", c.motion.home_settle_ms);
    emit("motion.home_to      = %d\n", c.motion.home_timeout_s);
    emit("motion.mm_per_rev   = %.3f\n", c.motion.mm_per_rev);
    emit("motion.hard_stop_mm = %.1f\n", c.motion.hard_stop_mm);
    emit("motion.soft_stop_mm = %.1f\n", c.motion.soft_stop_mm);
    emit("motion.goto_slow_mm = %.1f\n", c.motion.goto_slow_mm);
    emit("motion.goto_min_rpm = %d\n", c.motion.goto_min_rpm);
    emit("motion.goto_tol_mm  = %.1f\n", c.motion.goto_tol_mm);
    emit("net.ssid            = %s\n", c.network.ssid);
    emit("net.pass            = %s\n", c.network.pass[0] ? "(set)" : "(unset)");
    emit("net.conn_to         = %d\n", c.network.connect_timeout_s);
}

// Apply one "key value" assignment to the working config. Returns false if the
// key is unknown.
bool set_config_field(std::string_view key, const char* value)
{
    auto& c = work_config();

    if (key == "motion.duty_per_rpm") c.motion.duty_per_rpm = std::atof(value);
    else if (key == "motion.ff_off_raise") c.motion.ff_offset_raise_pct = std::atof(value);
    else if (key == "motion.ff_off_lower") c.motion.ff_offset_lower_pct = std::atof(value);
    else if (key == "motion.ff_trim_l")    c.motion.ff_trim_l = std::atof(value);
    else if (key == "motion.ff_trim_r")    c.motion.ff_trim_r = std::atof(value);
    else if (key == "motion.kp")          c.motion.kp = std::atof(value);
    else if (key == "motion.ki")          c.motion.ki = std::atof(value);
    else if (key == "motion.i_max")       c.motion.i_max = std::atof(value);
    else if (key == "motion.k_sync")      c.motion.k_sync = std::atof(value);
    else if (key == "motion.cover_rpm")   c.motion.cover_rpm = std::atoi(value);
    else if (key == "motion.sync_fault")  c.motion.sync_fault_limit = std::atoi(value);
    else if (key == "motion.stall_delta") c.motion.stall_delta_max = std::atoi(value);
    else if (key == "motion.stall_ms")    c.motion.stall_fault_ms = std::atoi(value);
    else if (key == "motion.grace_ms")    c.motion.startup_grace_ms = std::atoi(value);
    else if (key == "motion.home_duty")   c.motion.home_duty_pct = std::atoi(value);
    else if (key == "motion.home_settle") c.motion.home_settle_ms = std::atoi(value);
    else if (key == "motion.home_to")     c.motion.home_timeout_s = std::atoi(value);
    else if (key == "motion.mm_per_rev")  c.motion.mm_per_rev = std::atof(value);
    else if (key == "motion.hard_stop_mm") c.motion.hard_stop_mm = std::atof(value);
    else if (key == "motion.soft_stop_mm") c.motion.soft_stop_mm = std::atof(value);
    else if (key == "motion.goto_slow_mm") c.motion.goto_slow_mm = std::atof(value);
    else if (key == "motion.goto_min_rpm") c.motion.goto_min_rpm = std::atoi(value);
    else if (key == "motion.goto_tol_mm")  c.motion.goto_tol_mm = std::atof(value);
    else if (key == "net.ssid")    std::snprintf(c.network.ssid, sizeof(c.network.ssid), "%s", value);
    else if (key == "net.pass")    std::snprintf(c.network.pass, sizeof(c.network.pass), "%s", value);
    else if (key == "net.conn_to") c.network.connect_timeout_s = std::atoi(value);
    else return false;

    return true;
}

// View, set, and persist configuration.
//   config                  → dump the working config
//   config set <key> <val>  → update one field in the working copy
//   config save             → persist the working copy to NVS (and publish it)
//   config reset            → discard edits, reload the live config
int cmd_config(int argc, char** argv)
{
    if (argc == 1) {
        print_config(work_config());
        return 0;
    }

    const std::string_view sub{argv[1]};

    if (sub == "save") {
        if (auto r = config::save(work_config()); !r) {
            emit("save failed (err %d)\n", static_cast<int>(r.error()));
            return 1;
        }
        emit("config saved\n");
        return 0;
    }

    if (sub == "reset") {
        work_config() = config::get();
        emit("working config reloaded from live values\n");
        return 0;
    }

    if (sub == "set") {
        if (argc < 4) {
            emit("usage: config set <key> <value>\n");
            return 1;
        }
        if (!set_config_field(argv[2], argv[3])) {
            emit("unknown key: %s\n", argv[2]);
            return 1;
        }
        emit("set %s = %s (unsaved; run `config save`)\n", argv[2], argv[3]);
        return 0;
    }

    emit("usage: config | config set <key> <value> | config save | config reset\n");
    return 1;
}

// Reboot into WiFi debug mode by setting the one-shot boot flag.
int cmd_debug(int, char**)
{
    if (auto r = config::request_debug_boot(); !r) {
        emit("failed to arm debug boot (err %d)\n", static_cast<int>(r.error()));
        return 1;
    }
    emit("debug boot armed; rebooting into WiFi debug mode...\n");
    // Stop the loop and sleep the drivers so the motors aren't mid-drive across
    // the reset.
    hvmrf01::motion::stop();
    hvmrf01::motor::disable();
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
    return 0;  // unreached
}

// Forward-declared so they can appear in the command table.
int cmd_help(int argc, char** argv);
int cmd_home(int argc, char** argv);
int cmd_profile(int argc, char** argv);
int cmd_goto(int argc, char** argv);
int cmd_gotopct(int argc, char** argv);
int cmd_pos(int argc, char** argv);

// The full command table, shared by register_commands() and cmd_help().
const esp_console_cmd_t COMMANDS[] = {
        {.command = "fwd",    .help = "Raw forward at current duty (fights motion task)", .hint = "[L|R]", .func = &cmd_fwd,    .argtable = nullptr},
        {.command = "rev",    .help = "Raw reverse at current duty (fights motion task)", .hint = "[L|R]", .func = &cmd_rev,    .argtable = nullptr},
        {.command = "brake",  .help = "Active brake (EN=0; fights motion task)",          .hint = "[L|R]", .func = &cmd_brake,  .argtable = nullptr},
        {.command = "coast",  .help = "Coast both motors (nSLEEP low; fights motion task)", .hint = nullptr, .func = &cmd_coast,  .argtable = nullptr},
        {.command = "enable", .help = "Get/set driver enable (shared nSLEEP)",            .hint = "[on|off]", .func = &cmd_enable, .argtable = nullptr},
        {.command = "pwm",    .help = "Set PWM duty 0-100% on EN (fights motion task)",   .hint = "<pct> [L|R]", .func = &cmd_pwm, .argtable = nullptr},
        {.command = "freq",   .help = "Set PWM frequency in Hz",                          .hint = "<hz>",  .func = &cmd_freq,   .argtable = nullptr},
        {.command = "state",  .help = "Print current motor pin/duty state",               .hint = nullptr, .func = &cmd_state,  .argtable = nullptr},
        {.command = "sweep",  .help = "Sweep PWM freq while driving forward",             .hint = "[start] [end] [step] [dwell_ms]", .func = &cmd_sweep, .argtable = nullptr},
        {.command = "enc",    .help = "Read encoder count (also: reset, poll)",           .hint = "[L|R | reset [L|R] | poll [samples] [L|R]]", .func = &cmd_enc, .argtable = nullptr},
        {.command = "rpm",    .help = "Measure output-shaft RPM over 1s",                 .hint = "[L|R]", .func = &cmd_rpm,    .argtable = nullptr},
        {.command = "cur",    .help = "Read motor current draw via IPROPI (mA)",          .hint = "[L|R | poll [samples] [L|R]]", .func = &cmd_current, .argtable = nullptr},
        {.command = "spin",   .help = "Bench-test one motor: drive + log RPM/current",    .hint = "<L|R> <fwd|rev> <duty> [ms]", .func = &cmd_spin, .argtable = nullptr},
        {.command = "ramp",   .help = "Ramp 0→100% duty; stream count+current (CSV)",     .hint = "<L|R> [ramp_s] [hold_s] [hz] [fwd|rev]", .func = &cmd_ramp, .argtable = nullptr},
        {.command = "trace",  .help = "Stream encoder counts (CSV) for capture+plot",     .hint = "[duration_s=3] [hz=100]", .func = &cmd_trace, .argtable = nullptr},
        {.command = "motion", .help = "Closed-loop speed control",                        .hint = "<rpm> raise|lower | stop", .func = &cmd_motion, .argtable = nullptr},
        {.command = "config", .help = "View/set/save persisted config",                   .hint = "[set <key> <val> | save | reset]", .func = &cmd_config, .argtable = nullptr},
        {.command = "debug",  .help = "Reboot into WiFi debug mode",                       .hint = nullptr, .func = &cmd_debug,  .argtable = nullptr},
        {.command = "home",   .help = "Home both motors up to the top hard stop",          .hint = nullptr, .func = &cmd_home,   .argtable = nullptr},
        {.command = "goto",   .help = "Go to an absolute position (mm below the homed top)", .hint = "<mm> [rpm]", .func = &cmd_goto, .argtable = nullptr},
        {.command = "gotopct",.help = "Go to a position as % of full travel (clamped to soft stop)", .hint = "<0-100> [rpm]", .func = &cmd_gotopct, .argtable = nullptr},
        {.command = "pos",    .help = "Print current position (mm below the homed top)",   .hint = nullptr, .func = &cmd_pos, .argtable = nullptr},
        {.command = "profile",.help = "Open-loop both-motor drive; stream pos+current CSV", .hint = "<up|down> <duty> <rotations> [hz] [max_s]", .func = &cmd_profile, .argtable = nullptr},
        {.command = "help",   .help = "List available commands",                           .hint = nullptr, .func = &cmd_help,   .argtable = nullptr},
};

// Replaces ESP-IDF's built-in help, whose output goes straight to stdout and
// so isn't captured by emit() for the websocket console. Walks our own table.
int cmd_help(int, char**)
{
    for (const auto& c : COMMANDS) {
        if (c.hint != nullptr && c.hint[0] != '\0') {
            emit("%-7s %s\n            %s\n", c.command, c.hint, c.help);
        } else {
            emit("%-7s %s\n", c.command, c.help);
        }
    }
    return 0;
}

// Home both motors up against the top hard stop, zeroing the encoders. Blocks
// for up to the configured timeout; reports which sides settled.
int cmd_home(int, char**)
{
    emit("homing: driving up until both motors settle...\n");
    const auto r = hvmrf01::motion::home();
    emit("home: L=%s R=%s\n", r.left ? "ok" : "TIMEOUT", r.right ? "ok" : "TIMEOUT");
    if (!r.left || !r.right) {
        emit("warning: a motor never settled — check the mechanism\n");
        return 1;
    }
    emit("homed; encoders zeroed at top\n");
    return 0;
}

// Common reply for the fire-and-forget position commands. The move runs in the
// control task; this returns immediately so the console/websocket stays free to
// process a `motion stop` mid-move (a blocking move would wedge the single
// websocket handler and STOP couldn't get through).
int report_started(bool started)
{
    if (!started) {
        emit("rejected — run `home` first and set motion.mm_per_rev\n");
        return 1;
    }
    emit("move started (use `motion stop` to halt, `pos` to check progress)\n");
    return 0;
}

// Move to an absolute position, mm below the homed top. Non-blocking.
// Optional rpm sets the cruise speed (default: cover_rpm).
int cmd_goto(int argc, char** argv)
{
    if (argc < 2) {
        emit("usage: goto <mm below top> [rpm]\n");
        return 1;
    }
    const int rpm = (argc > 2) ? std::atoi(argv[2]) : 0;
    return report_started(
        hvmrf01::motion::begin_go_to_mm(static_cast<float>(std::atof(argv[1])), rpm));
}

// Move to a position as a percentage of full travel (100% = hard_stop_mm,
// clamped to the soft stop). Non-blocking; same prerequisites as `goto`.
// Optional rpm sets the cruise speed (default: cover_rpm).
int cmd_gotopct(int argc, char** argv)
{
    if (argc < 2) {
        emit("usage: gotopct <0-100> [rpm]\n");
        return 1;
    }
    const int rpm = (argc > 2) ? std::atoi(argv[2]) : 0;
    return report_started(
        hvmrf01::motion::begin_go_to_pct(static_cast<float>(std::atof(argv[1])), rpm));
}

// Print the current position in mm below the homed top.
int cmd_pos(int, char**)
{
    const auto p = hvmrf01::motion::position_mm();
    emit("L=%.1f mm R=%.1f mm%s\n", p.mm_l, p.mm_r,
         p.valid ? "" : "  (not homed / mm_per_rev unset — value meaningless)");
    return 0;
}

// Open-loop system characterization. Drives BOTH motors at a fixed duty in one
// direction with no PI/sync/stall feedback, each braking once its own encoder
// has turned `rotations` output revs (or the time cap hits). Streams a CSV
// envelope of raw per-motor position + current so the host can derive velocity
// and acceleration and see how the two sides track open-loop.
//   profile <up|down> <duty 0-100> <rotations> [hz=50] [max_s=25]
int cmd_profile(int argc, char** argv)
{
    using hvmrf01::motor::Mode;
    using hvmrf01::motor::Side;

    if (argc < 4) {
        emit("usage: profile <up|down> <duty 0-100> <rotations> [hz=50] [max_s=25]\n");
        return 1;
    }

    const std::string_view dir{argv[1]};
    const bool up   = (dir == "up" || dir == "raise");
    const bool down = (dir == "down" || dir == "lower");
    if (!up && !down) {
        emit("direction must be up|down\n");
        return 1;
    }
    // up = raise = Forward; down = lower = Reverse (matches motion).
    const Mode drive_mode = up ? Mode::Forward : Mode::Reverse;

    const int duty = std::atoi(argv[2]);
    if (duty < 0 || duty > 100) {
        emit("duty out of range: 0-100\n");
        return 1;
    }
    const int rotations = std::atoi(argv[3]);
    if (rotations <= 0 || rotations > 100) {
        emit("rotations out of range: 1-100\n");
        return 1;
    }
    int hz = argc > 4 ? std::atoi(argv[4]) : 50;
    if (hz < 1 || hz > 200) hz = 50;
    int max_s = argc > 5 ? std::atoi(argv[5]) : 25;
    if (max_s < 1 || max_s > 60) max_s = 25;

    const std::int32_t target = static_cast<std::int32_t>(rotations) *
                                hvmrf01::encoder::COUNTS_PER_OUTPUT_REV;
    const int        total_samples = max_s * hz;
    const TickType_t period        = pdMS_TO_TICKS(1000 / hz);

    // Take the motors from the (idle) controller and start from a clean zero.
    hvmrf01::motion::stop();
    hvmrf01::encoder::reset(Side::Both);

    emit("PROFILE_BEGIN dir=%s duty=%d rotations=%d hz=%d max_s=%d cpr=%ld\n",
         up ? "up" : "down", duty, rotations, hz, max_s,
         static_cast<long>(hvmrf01::encoder::COUNTS_PER_OUTPUT_REV));
    emit("t_ms,count_l,count_r,cur_l_ma,cur_r_ma\n");

    bool done_l = false, done_r = false;
    const auto t0  = xTaskGetTickCount();
    TickType_t wake = t0;
    int emitted = 0;

    for (int i = 0; i < total_samples; i++) {
        const std::int32_t cl = hvmrf01::encoder::count(Side::Left);
        const std::int32_t cr = hvmrf01::encoder::count(Side::Right);

        if (!done_l && std::abs(cl) >= target) {
            hvmrf01::motor::drive(Side::Left, Mode::Brake, 0);
            done_l = true;
        }
        if (!done_r && std::abs(cr) >= target) {
            hvmrf01::motor::drive(Side::Right, Mode::Brake, 0);
            done_r = true;
        }
        if (!done_l) hvmrf01::motor::drive(Side::Left, drive_mode, duty);
        if (!done_r) hvmrf01::motor::drive(Side::Right, drive_mode, duty);

        const auto t_ms = pdTICKS_TO_MS(xTaskGetTickCount() - t0);
        emit("%lu,%ld,%ld,%ld,%ld\n",
             static_cast<unsigned long>(t_ms),
             static_cast<long>(cl), static_cast<long>(cr),
             static_cast<long>(hvmrf01::current_sense::current_ma(Side::Left)),
             static_cast<long>(hvmrf01::current_sense::current_ma(Side::Right)));
        emitted++;

        if (done_l && done_r) {
            break;
        }
        vTaskDelayUntil(&wake, period);
    }

    hvmrf01::motor::debug::set_brake(Side::Both);
    emit("PROFILE_END samples=%d done_l=%d done_r=%d\n", emitted, done_l, done_r);
    return 0;
}

void register_commands()
{
    for (const auto& cmd : COMMANDS) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
    }
}

}  // namespace

void start()
{
    esp_console_repl_t* repl = nullptr;

    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt              = "hv-mrf-01> ";
    repl_config.max_cmdline_length  = 80;

    esp_console_dev_usb_serial_jtag_config_t hw_config =
        ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&hw_config, &repl_config, &repl));
    register_commands();
    ESP_ERROR_CHECK(esp_console_start_repl(repl));

    ESP_LOGI(TAG, "REPL ready on USB-Serial-JTAG");
}

void init_for_remote()
{
    esp_console_config_t console_config = ESP_CONSOLE_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_init(&console_config));
    register_commands();
    ESP_LOGI(TAG, "console ready for remote dispatch");
}

std::string run_line(const std::string& line)
{
    std::lock_guard<std::mutex> lock(capture_mutex);

    std::string out;
    capture_buf = &out;
    int       ret = 0;
    const esp_err_t err = esp_console_run(line.c_str(), &ret);
    capture_buf = nullptr;

    if (err == ESP_ERR_NOT_FOUND) {
        out += "unknown command (try `help`)\n";
    } else if (err == ESP_ERR_INVALID_ARG) {
        // Empty line — nothing to do, return empty output.
    } else if (err != ESP_OK) {
        out += "command failed to run\n";
    }

    return out;
}

}  // namespace hvmrf01::console
