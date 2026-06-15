#include "blinds/console.hpp"

#include <cstdio>
#include <cstdlib>
#include <string_view>

#include "esp_console.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "blinds/current_sense.hpp"
#include "blinds/encoder.hpp"
#include "blinds/motion.hpp"
#include "blinds/motor.hpp"

namespace blinds::console {

namespace {

constexpr auto* TAG = "blinds.console";

// Parse an optional "L" / "R" / "both" arg; default to Both.
blinds::motor::Side parse_side(int argc, char** argv, int idx)
{
    if (argc <= idx) return blinds::motor::Side::Both;
    const std::string_view a{argv[idx]};
    if (a == "L" || a == "l" || a == "left")  return blinds::motor::Side::Left;
    if (a == "R" || a == "r" || a == "right") return blinds::motor::Side::Right;
    return blinds::motor::Side::Both;
}

const char* side_label(blinds::motor::Side s)
{
    using S = blinds::motor::Side;
    return s == S::Left ? "L" : s == S::Right ? "R" : "both";
}

int cmd_fwd(int argc, char** argv)
{
    const auto s = parse_side(argc, argv, 1);
    blinds::motor::debug::set_forward(s);
    printf("→ %s forward at current duty\n", side_label(s));
    return 0;
}

int cmd_rev(int argc, char** argv)
{
    const auto s = parse_side(argc, argv, 1);
    blinds::motor::debug::set_reverse(s);
    printf("→ %s reverse at current duty\n", side_label(s));
    return 0;
}

int cmd_brake(int argc, char** argv)
{
    const auto s = parse_side(argc, argv, 1);
    blinds::motor::debug::set_brake(s);
    printf("→ %s brake (EN=0, active hold)\n", side_label(s));
    return 0;
}

int cmd_coast(int, char**)
{
    // Coast is the shared nSLEEP going low — inherently both motors.
    blinds::motor::debug::set_coast();
    printf("→ coast (nSLEEP low, both motors Hi-Z)\n");
    return 0;
}

// Drive the shared driver-enable (nSLEEP) line. on = both drivers awake
// (required before drive/brake do anything); off = both coast.
int cmd_enable(int argc, char** argv)
{
    if (argc < 2) {
        printf("driver enable (nSLEEP) = %s\n",
               blinds::motor::debug::enabled() ? "on" : "off");
        return 0;
    }
    const std::string_view a{argv[1]};
    const bool on = (a == "on" || a == "1" || a == "true");
    const bool off = (a == "off" || a == "0" || a == "false");
    if (!on && !off) {
        printf("usage: enable [on|off]\n");
        return 1;
    }
    blinds::motor::debug::set_enabled(on);
    printf("driver enable (nSLEEP) = %s\n", on ? "on" : "off");
    return 0;
}

int cmd_pwm(int argc, char** argv)
{
    if (argc < 2) {
        printf("usage: pwm <0-100> [L|R]\n");
        return 1;
    }
    const int pct = std::atoi(argv[1]);
    if (pct < 0 || pct > 100) {
        printf("out of range: 0-100\n");
        return 1;
    }
    const auto s = parse_side(argc, argv, 2);
    blinds::motor::debug::set_duty_pct(pct, s);
    printf("duty = %d%% (%s)\n", pct, side_label(s));
    return 0;
}

int cmd_freq(int argc, char** argv)
{
    if (argc != 2) {
        printf("usage: freq <hz>\n");
        return 1;
    }
    const int hz = std::atoi(argv[1]);
    if (!blinds::motor::debug::set_freq_hz(hz)) {
        printf("freq %d Hz rejected (try 100..200000)\n", hz);
        return 1;
    }
    printf("freq = %d Hz\n", hz);
    return 0;
}

int cmd_state(int, char**)
{
    blinds::motor::debug::print_state();
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
        printf("L=%ld R=%ld\n",
               static_cast<long>(blinds::encoder::count(blinds::motor::Side::Left)),
               static_cast<long>(blinds::encoder::count(blinds::motor::Side::Right)));
        return 0;
    }
    if (std::string_view{argv[1]} == "reset") {
        const auto s = parse_side(argc, argv, 2);
        blinds::encoder::reset(s);
        printf("reset %s\n", side_label(s));
        return 0;
    }
    if (std::string_view{argv[1]} == "poll") {
        const int samples = argc >= 3 ? std::atoi(argv[2]) : 25;
        const auto side   = parse_side(argc, argv, 3);
        // Side::Both isn't meaningful here — pick Left.
        const auto s      = side == blinds::motor::Side::Both ? blinds::motor::Side::Left : side;
        std::int32_t prev  = blinds::encoder::count(s);
        const auto   start = xTaskGetTickCount();
        for (int i = 0; i < samples; i++) {
            vTaskDelay(pdMS_TO_TICKS(200));
            const auto now      = blinds::encoder::count(s);
            const auto delta    = now - prev;
            const auto cps      = delta * 5;  // 200ms window → counts/sec
            prev                = now;
            const auto elapsed  = pdTICKS_TO_MS(xTaskGetTickCount() - start);
            printf("[%s] t=%lums  count=%ld  Δ=%ld  cps=%ld\n",
                   side_label(s),
                   static_cast<unsigned long>(elapsed),
                   static_cast<long>(now), static_cast<long>(delta),
                   static_cast<long>(cps));
        }
        return 0;
    }
    // Single-arg side ("enc L" / "enc R").
    const auto s = parse_side(argc, argv, 1);
    if (s == blinds::motor::Side::Both) {
        printf("usage: enc | enc [L|R] | enc reset [L|R] | enc poll [samples] [L|R]\n");
        return 1;
    }
    printf("%s = %ld\n", side_label(s),
           static_cast<long>(blinds::encoder::count(s)));
    return 0;
}

// Measure output-shaft RPM by sampling the encoder count over a 1s window.
int cmd_rpm(int argc, char** argv)
{
    const auto side = parse_side(argc, argv, 1);
    const auto s    = side == blinds::motor::Side::Both ? blinds::motor::Side::Left : side;
    const auto start = blinds::encoder::count(s);
    vTaskDelay(pdMS_TO_TICKS(1000));
    const auto end   = blinds::encoder::count(s);
    const auto cps   = end - start;
    const auto rpm   = (cps * 60) / blinds::encoder::COUNTS_PER_OUTPUT_REV;
    printf("[%s] Δcount=%ld cps=%ld output_rpm=%ld (motor_rpm=%ld)\n",
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
    using blinds::motor::Side;

    auto print_one = [](Side s) {
        printf("[%s] %ld mA  (%ld mV)\n", side_label(s),
               static_cast<long>(blinds::current_sense::current_ma(s)),
               static_cast<long>(blinds::current_sense::voltage_mv(s)));
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
                printf("L=%ld mA  R=%ld mA\n",
                       static_cast<long>(blinds::current_sense::current_ma(Side::Left)),
                       static_cast<long>(blinds::current_sense::current_ma(Side::Right)));
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
    using blinds::motor::Side;

    if (argc < 4) {
        printf("usage: spin <L|R> <fwd|rev> <duty 0-100> [ms=1500]\n");
        return 1;
    }
    const auto side = parse_side(argc, argv, 1);
    if (side == Side::Both) {
        printf("spin targets one motor — use L or R\n");
        return 1;
    }
    const std::string_view dir{argv[2]};
    const bool forward = (dir == "fwd" || dir == "f" || dir == "forward");
    const bool reverse = (dir == "rev" || dir == "r" || dir == "reverse");
    if (!forward && !reverse) {
        printf("direction must be fwd|rev\n");
        return 1;
    }
    const int duty = std::atoi(argv[3]);
    if (duty < 0 || duty > 100) {
        printf("duty out of range: 0-100\n");
        return 1;
    }
    int ms = argc > 4 ? std::atoi(argv[4]) : 1500;
    if (ms < 100)   ms = 100;
    if (ms > 10000) ms = 10000;

    constexpr int window_ms = 100;
    const int     steps     = ms / window_ms;

    printf("spin %s %s duty=%d%% for %d ms\n",
           side_label(side), forward ? "fwd" : "rev", duty, ms);
    printf("t_ms,rpm,current_ma\n");

    blinds::motor::debug::set_duty_pct(duty, side);
    if (forward) blinds::motor::debug::set_forward(side);
    else         blinds::motor::debug::set_reverse(side);

    std::int32_t prev  = blinds::encoder::count(side);
    const auto   t0    = xTaskGetTickCount();
    for (int i = 0; i < steps; i++) {
        vTaskDelay(pdMS_TO_TICKS(window_ms));
        const auto now   = blinds::encoder::count(side);
        const auto delta = now - prev;
        prev             = now;
        const auto cps   = delta * (1000 / window_ms);
        const auto rpm   = (cps * 60) / blinds::encoder::COUNTS_PER_OUTPUT_REV;
        const auto t_ms  = pdTICKS_TO_MS(xTaskGetTickCount() - t0);
        printf("%lu,%ld,%ld\n",
               static_cast<unsigned long>(t_ms),
               static_cast<long>(rpm),
               static_cast<long>(blinds::current_sense::current_ma(side)));
    }

    blinds::motor::debug::set_brake(side);
    printf("spin done; %s braked.\n", side_label(side));
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
    using blinds::motor::Side;

    if (argc < 2) {
        printf("usage: ramp <L|R> [ramp_s=3] [hold_s=1] [hz=100] [fwd|rev]\n");
        return 1;
    }
    const auto side = parse_side(argc, argv, 1);
    if (side == Side::Both) {
        printf("ramp targets one motor — use L or R\n");
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
            printf("direction must be fwd|rev\n");
            return 1;
        }
    }
    if (ramp_s < 0 || hold_s < 0 || (ramp_s + hold_s) <= 0 ||
        (ramp_s + hold_s) > 30 || hz < 1 || hz > 500) {
        printf("usage: ramp <L|R> [ramp_s=3 (0..30 total)] [hold_s=1] [hz=100 (1..500)] [fwd|rev]\n");
        return 1;
    }

    const int  ramp_ms        = ramp_s * 1000;
    const int  total_ms       = (ramp_s + hold_s) * 1000;
    const int  total_samples  = total_ms * hz / 1000;
    const auto period         = pdMS_TO_TICKS(1000 / hz);

    blinds::encoder::reset(side);
    if (forward) blinds::motor::debug::set_forward(side);
    else         blinds::motor::debug::set_reverse(side);

    printf("RAMP_BEGIN hz=%d ramp_s=%d hold_s=%d side=%s dir=%s\n",
           hz, ramp_s, hold_s, side_label(side), forward ? "fwd" : "rev");
    printf("t_ms,duty,count,current_ma\n");

    const auto t0   = xTaskGetTickCount();
    TickType_t wake = t0;
    int emitted = 0;
    for (int i = 0; i < total_samples; i++) {
        const long t_ms = static_cast<long>(pdTICKS_TO_MS(xTaskGetTickCount() - t0));
        const int  duty = (t_ms >= ramp_ms) ? 100
                        : static_cast<int>(100L * t_ms / ramp_ms);
        blinds::motor::debug::set_duty_pct(duty, side);

        const auto count = blinds::encoder::count(side);
        const auto cur   = blinds::current_sense::current_ma(side);
        printf("%ld,%d,%ld,%ld\n", t_ms, duty,
               static_cast<long>(count), static_cast<long>(cur));
        emitted++;
        vTaskDelayUntil(&wake, period);
    }

    blinds::motor::debug::set_brake(side);
    printf("RAMP_END samples=%d\n", emitted);
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
        printf("usage: trace [duration_s=3 (1..30)] [hz=100 (1..500)]\n");
        return 1;
    }

    const int    total_samples = duration_s * hz;
    const auto   period_ticks  = pdMS_TO_TICKS(1000 / hz);
    const auto   t0_ticks      = xTaskGetTickCount();
    TickType_t   wake          = t0_ticks;

    printf("TRACE_BEGIN hz=%d duration_s=%d\n", hz, duration_s);
    printf("t_ms,L,R\n");

    int emitted = 0;
    for (int i = 0; i < total_samples; i++) {
        const auto l = blinds::encoder::count(blinds::motor::Side::Left);
        const auto r = blinds::encoder::count(blinds::motor::Side::Right);
        const auto t_ms = pdTICKS_TO_MS(xTaskGetTickCount() - t0_ticks);
        printf("%lu,%ld,%ld\n",
               static_cast<unsigned long>(t_ms),
               static_cast<long>(l), static_cast<long>(r));
        emitted++;
        vTaskDelayUntil(&wake, period_ticks);
    }

    printf("TRACE_END samples=%d\n", emitted);
    return 0;
}

// Motion controller commands.
//   motion <rpm> raise|lower   → set_target
//   motion stop                → stop()
int cmd_motion(int argc, char** argv)
{
    if (argc >= 2 && std::string_view{argv[1]} == "stop") {
        blinds::motion::stop();
        printf("motion: stopped\n");
        return 0;
    }
    if (argc < 3) {
        printf("usage: motion <rpm> raise|lower | motion stop\n");
        return 1;
    }
    const int rpm = std::atoi(argv[1]);
    if (rpm <= 0 || rpm > 300) {
        printf("rpm out of range (1..300)\n");
        return 1;
    }
    const std::string_view dir{argv[2]};
    blinds::motion::Direction d;
    if (dir == "raise" || dir == "up" || dir == "open") {
        d = blinds::motion::Direction::Raise;
    } else if (dir == "lower" || dir == "down" || dir == "close") {
        d = blinds::motion::Direction::Lower;
    } else {
        printf("direction must be raise|lower\n");
        return 1;
    }
    blinds::motion::set_target(rpm, d);
    printf("motion: target = %d RPM %s\n", rpm,
           d == blinds::motion::Direction::Raise ? "raise" : "lower");
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
        printf("usage: sweep [start_hz=5000] [end_hz=50000] [step_hz=1000] [dwell_ms=1500]\n");
        return 1;
    }

    printf("sweep %d → %d Hz, step %d, dwell %d ms. Driving forward.\n",
           start_hz, end_hz, step_hz, dwell_ms);
    blinds::motor::debug::set_forward();

    for (int hz = start_hz; hz <= end_hz; hz += step_hz) {
        if (!blinds::motor::debug::set_freq_hz(hz)) {
            printf("  %d Hz rejected, stopping sweep\n", hz);
            break;
        }
        printf("  %d Hz\n", hz);
        vTaskDelay(pdMS_TO_TICKS(dwell_ms));
    }

    blinds::motor::debug::set_brake();
    printf("sweep done; braked.\n");
    return 0;
}

void register_commands()
{
    const esp_console_cmd_t cmds[] = {
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
    };
    for (const auto& cmd : cmds) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
    }
}

}  // namespace

void start()
{
    esp_console_repl_t* repl = nullptr;

    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt              = "blinds> ";
    repl_config.max_cmdline_length  = 80;

    esp_console_dev_usb_serial_jtag_config_t hw_config =
        ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&hw_config, &repl_config, &repl));
    ESP_ERROR_CHECK(esp_console_register_help_command());
    register_commands();
    ESP_ERROR_CHECK(esp_console_start_repl(repl));

    ESP_LOGI(TAG, "REPL ready on USB-Serial-JTAG");
}

}  // namespace blinds::console
