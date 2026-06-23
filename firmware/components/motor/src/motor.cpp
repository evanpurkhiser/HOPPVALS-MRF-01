#include "hv-mrf-01/motor.hpp"

#include <array>
#include <cstdint>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

namespace hvmrf01::motor {

namespace {

constexpr auto* TAG = "hv-mrf-01.motor";

// Per-motor hardware mapping (PCB netlist, PH/EN mode)

struct Motor
{
    gpio_num_t     en;       // EN/IN1 — PWM (LEDC); EN=0 brakes
    gpio_num_t     ph;       // PH/IN2 — direction
    ledc_channel_t channel;
    const char*    label;
};

// Motor L: DRV_L1 → EN on D9 (GPIO20), PH on D10 (GPIO18).
// Motor R: DRV_R1 → EN on D7 (GPIO17), PH on D8  (GPIO19).
constexpr Motor MOTOR_L{
    .en = GPIO_NUM_20, .ph = GPIO_NUM_18,
    .channel = LEDC_CHANNEL_1, .label = "L",
};
constexpr Motor MOTOR_R{
    .en = GPIO_NUM_17, .ph = GPIO_NUM_19,
    .channel = LEDC_CHANNEL_2, .label = "R",
};

constexpr std::array<Motor, 2> BOTH{ MOTOR_L, MOTOR_R };

// Shared nSLEEP for both drivers (D2 / GPIO2, "MOTOR_SLEEP"). Active high:
// drive it high to wake/enable both H-bridges, low to coast both. The
// spring-less blind relies on the drivers being awake so EN=0 short-brakes
// and holds position — so we keep this high except for an explicit coast.
constexpr gpio_num_t MOTOR_SLEEP = GPIO_NUM_2;

// PWM (LEDC) config
// Both motors share LEDC_TIMER_1 so they always PWM at the same frequency.
// Each motor's EN pin gets its own channel. LED uses TIMER_0/CHAN_0; we're
// on TIMER_1 with CHAN_1 (L) and CHAN_2 (R).

constexpr auto LEDC_MODE       = LEDC_LOW_SPEED_MODE;
constexpr auto LEDC_TIMER      = LEDC_TIMER_1;
constexpr auto LEDC_RESOLUTION = LEDC_TIMER_8_BIT;
constexpr int  LEDC_MAX_DUTY   = (1 << 8) - 1; // 255
constexpr int  LEDC_FREQ_HZ    = 25000;        // above most adult hearing

// Last commanded duty / freq / enable, tracked for accurate state readback.
int  current_duty_pct[2] = { 0, 0 };   // [0]=L, [1]=R
int  current_freq_hz     = LEDC_FREQ_HZ;
bool drivers_enabled     = false;

constexpr std::size_t idx(const Motor& m) { return &m == &MOTOR_L ? 0 : 1; }

// Low-level driver helpers

void apply_duty(const Motor& m, int pct)
{
    if (pct < 0)   pct = 0;
    if (pct > 100) pct = 100;
    current_duty_pct[idx(m)] = pct;
    const uint32_t duty = static_cast<uint32_t>(pct) * LEDC_MAX_DUTY / 100;
    ledc_set_duty(LEDC_MODE, m.channel, duty);
    ledc_update_duty(LEDC_MODE, m.channel);
}

// Drive (or release) the shared nSLEEP line. High wakes both drivers.
void set_sleep(bool awake)
{
    drivers_enabled = awake;
    gpio_set_level(MOTOR_SLEEP, awake ? 1 : 0);
}

void drive_forward(const Motor& m, int duty_pct)
{
    set_sleep(true);
    gpio_set_level(m.ph, 1);
    apply_duty(m, duty_pct);
}

void drive_reverse(const Motor& m, int duty_pct)
{
    set_sleep(true);
    gpio_set_level(m.ph, 0);
    apply_duty(m, duty_pct);
}

// Active brake: EN=0 with the driver awake turns both low-side FETs on,
// shorting the motor terminals so any rotation generates resisting current.
void brake_stop(const Motor& m)
{
    set_sleep(true);
    apply_duty(m, 0);
}

// Coast puts both H-bridges Hi-Z via the shared nSLEEP — it is inherently
// all-or-nothing, so we ignore the per-motor distinction and stop both.
void coast_stop()
{
    set_sleep(false);
    for (const auto& m : BOTH) apply_duty(m, 0);
}

// Apply an action to one motor or both, depending on Side.
template <typename Fn>
void each(Side s, Fn&& fn)
{
    if (s == Side::Left  || s == Side::Both) fn(MOTOR_L);
    if (s == Side::Right || s == Side::Both) fn(MOTOR_R);
}

// Hardware init

void configure_gpio()
{
    std::uint64_t mask = 1ULL << MOTOR_SLEEP;
    for (const auto& m : BOTH) mask |= (1ULL << m.ph);

    const gpio_config_t cfg{
        .pin_bit_mask = mask,
        // INPUT_OUTPUT keeps the input buffer enabled so gpio_get_level()
        // reads back the driven level — used by the `state` console command.
        .mode         = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));

    // Keep the drivers ASLEEP (coast) through pin setup. Waking them here would
    // be dangerous: the EN pins aren't driven until configure_ledc() runs, so a
    // floating-high EN with nSLEEP awake and PH=0 drives the motor full-tilt in
    // one direction until LEDC clamps EN to 0. start() wakes the drivers into
    // brake only after configure_ledc() has forced EN=0.
    set_sleep(false);
    for (const auto& m : BOTH) gpio_set_level(m.ph, 0);
}

void configure_ledc()
{
    const ledc_timer_config_t timer_cfg{
        .speed_mode      = LEDC_MODE,
        .duty_resolution = LEDC_RESOLUTION,
        .timer_num       = LEDC_TIMER,
        .freq_hz         = LEDC_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
        .deconfigure     = false,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    for (const auto& m : BOTH) {
        const ledc_channel_config_t channel_cfg{
            .gpio_num   = m.en,
            .speed_mode = LEDC_MODE,
            .channel    = m.channel,
            .intr_type  = LEDC_INTR_DISABLE,
            .timer_sel  = LEDC_TIMER,
            .duty       = 0,
            .hpoint     = 0,
            .flags      = { .output_invert = 0 },
        };
        ESP_ERROR_CHECK(ledc_channel_config(&channel_cfg));
    }
}

}  // namespace

void start()
{
    configure_gpio();   // pins set up, drivers asleep (coast), PH=0
    configure_ledc();   // EN driven to 0

    // Now that EN is solidly 0, wake the drivers — nSLEEP high + EN=0 is brake,
    // the hold state the spring-less blind needs. Done in this order so the
    // motors never see "awake + undefined EN" (which would drive them).
    set_sleep(true);

    ESP_LOGI(TAG, "motors wired: L=(EN=%d PH=%d) R=(EN=%d PH=%d) nSLEEP=%d (enabled)",
             MOTOR_L.en, MOTOR_L.ph, MOTOR_R.en, MOTOR_R.ph, MOTOR_SLEEP);
}

void disable()
{
    // nSLEEP low: both H-bridges go Hi-Z and stay there until a drive wakes
    // them. Stop the motion controller first so its 100 Hz loop doesn't wake
    // nSLEEP back up on the next tick.
    set_sleep(false);
    ESP_LOGI(TAG, "motor drivers disabled (nSLEEP low)");
}

void drive(Side s, Mode mode, int duty_pct)
{
    if (mode == Mode::Coast) {
        coast_stop();
        return;
    }
    each(s, [mode, duty_pct](const Motor& m) {
        switch (mode) {
        case Mode::Forward: drive_forward(m, duty_pct); break;
        case Mode::Reverse: drive_reverse(m, duty_pct); break;
        case Mode::Brake:   brake_stop(m);              break;
        case Mode::Coast:   break;  // handled above
        }
    });
}

namespace debug {

void set_forward(Side s)
{
    set_sleep(true);
    each(s, [](const Motor& m) { gpio_set_level(m.ph, 1); });
}

void set_reverse(Side s)
{
    set_sleep(true);
    each(s, [](const Motor& m) { gpio_set_level(m.ph, 0); });
}

void set_brake(Side s)
{
    set_sleep(true);
    each(s, [](const Motor& m) { apply_duty(m, 0); });
}

void set_coast(Side)
{
    coast_stop();
}

void set_duty_pct(int pct, Side s)
{
    each(s, [pct](const Motor& m) { apply_duty(m, pct); });
}

bool set_freq_hz(int hz)
{
    if (hz < 100 || hz > 200000) return false;
    if (ledc_set_freq(LEDC_MODE, LEDC_TIMER, hz) != ESP_OK) return false;
    current_freq_hz = hz;
    return true;
}

void set_enabled(bool on)
{
    set_sleep(on);
}

bool enabled()
{
    return drivers_enabled;
}

void print_state()
{
    const int sleep_lvl = gpio_get_level(MOTOR_SLEEP);
    for (const auto& m : BOTH) {
        const int   ph    = gpio_get_level(m.ph);
        const int   duty  = current_duty_pct[idx(m)];
        const char* mode  = !sleep_lvl   ? "COAST"
                          : (duty == 0)  ? "BRAKE"
                          : ph           ? "FORWARD"
                                         : "REVERSE";
        printf("Motor %s: PH=%d  duty=%d%%  mode=%s\n", m.label, ph, duty, mode);
    }
    printf("nSLEEP=%d (%s)  freq=%d Hz (shared)\n",
           sleep_lvl, sleep_lvl ? "enabled" : "coast", current_freq_hz);
}

}  // namespace debug

}  // namespace hvmrf01::motor
