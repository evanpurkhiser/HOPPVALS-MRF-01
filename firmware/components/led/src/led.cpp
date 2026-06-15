#include "blinds/led.hpp"

#include <atomic>
#include <cmath>
#include <cstdint>
#include <numbers>

#include "driver/ledc.h"
#include "esp_event.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

#include "blinds/zigbee.hpp"

namespace blinds::led {

namespace {

constexpr auto* TAG = "blinds.led";

// XIAO ESP32-C6 user LED is on GPIO15, wired active-low (LED to VCC,
// MCU pin sinks current). We invert duty cycles internally so the
// public Brightness API behaves intuitively (0 = off, 100 = full).
constexpr int  LED_GPIO          = 15;
constexpr bool LED_ACTIVE_LOW    = true;
constexpr auto LEDC_MODE         = LEDC_LOW_SPEED_MODE;
constexpr auto LEDC_TIMER        = LEDC_TIMER_0;
constexpr auto LEDC_CHANNEL      = LEDC_CHANNEL_0;
constexpr auto LEDC_RESOLUTION   = LEDC_TIMER_10_BIT;
constexpr int  LEDC_MAX_DUTY     = (1 << 10) - 1;  // 1023
constexpr int  LEDC_FREQUENCY_HZ = 5000;

// State-machine tick rate. 50 Hz gives smooth breathing fades.
constexpr TickType_t TICK_MS = pdMS_TO_TICKS(20);

// Effect timings (milliseconds).
constexpr std::uint32_t BLINK_TOTAL_MS    = 1000;          // single 1s blink
constexpr std::uint32_t BREATHE_PERIOD_MS = 1000;          // one fade cycle
constexpr std::uint32_t BREATHE_TOTAL_MS  = BREATHE_PERIOD_MS * 15;
constexpr std::uint32_t OKAY_PULSE_MS     = 100;           // on then off then on then off
constexpr std::uint32_t OKAY_TOTAL_MS     = OKAY_PULSE_MS * 4;
constexpr std::uint32_t FLASH_TOTAL_MS    = 200;           // one quick flash

// What's currently running. Touched from both the event loop (which sets it
// when an Identify event arrives) and the FreeRTOS timer task (which reads
// it on every tick), so it's atomic.
enum class Effect : std::uint8_t { None, Blink, Breathe, Okay, Flash };
std::atomic<Effect>        current_effect{ Effect::None };
std::atomic<std::uint32_t> elapsed_ms{ 0 };
TimerHandle_t              tick_timer = nullptr;

void write_brightness(std::uint8_t pct)
{
    const std::uint32_t scaled = (static_cast<std::uint32_t>(pct) * LEDC_MAX_DUTY) / 100;
    const std::uint32_t duty   = LED_ACTIVE_LOW ? (LEDC_MAX_DUTY - scaled) : scaled;
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

// Half-cosine ease-in/ease-out: 0 → 100 → 0 over one period.
std::uint8_t breathe_brightness(std::uint32_t phase_ms)
{
    constexpr float two_pi = 2.0f * std::numbers::pi_v<float>;
    const float     angle  = (static_cast<float>(phase_ms) / BREATHE_PERIOD_MS) * two_pi;
    const float     v      = (1.0f - std::cos(angle)) * 0.5f;  // 0..1
    return static_cast<std::uint8_t>(v * 100.0f);
}

void finish()
{
    current_effect.store(Effect::None);
    elapsed_ms.store(0);
    write_brightness(0);
}

void on_tick(TimerHandle_t)
{
    const auto effect = current_effect.load();
    if (effect == Effect::None) {
        return;
    }

    const auto t = elapsed_ms.fetch_add(pdTICKS_TO_MS(TICK_MS)) + pdTICKS_TO_MS(TICK_MS);

    switch (effect) {
    case Effect::Blink:
        if (t >= BLINK_TOTAL_MS) { finish(); break; }
        write_brightness(t < BLINK_TOTAL_MS / 2 ? 100 : 0);
        break;
    case Effect::Breathe:
        if (t >= BREATHE_TOTAL_MS) { finish(); break; }
        write_brightness(breathe_brightness(t % BREATHE_PERIOD_MS));
        break;
    case Effect::Okay: {
        if (t >= OKAY_TOTAL_MS) { finish(); break; }
        const auto pulse = (t / OKAY_PULSE_MS) % 2;  // 0 = on, 1 = off
        write_brightness(pulse == 0 ? 100 : 0);
    } break;
    case Effect::Flash:
        if (t >= FLASH_TOTAL_MS) { finish(); break; }
        write_brightness(100);
        break;
    case Effect::None:
        break;
    }
}

void start_effect(Effect e)
{
    current_effect.store(e);
    elapsed_ms.store(0);
    if (e == Effect::None) {
        write_brightness(0);
    }
}

Effect map_zcl_effect(blinds::zigbee::IdentifyEffect e) noexcept
{
    using ZE = blinds::zigbee::IdentifyEffect;
    switch (e) {
    case ZE::Blink:         return Effect::Blink;
    case ZE::Breathe:       return Effect::Breathe;
    case ZE::Okay:          return Effect::Okay;
    case ZE::ChannelChange: return Effect::Flash;
    case ZE::FinishEffect:  return current_effect.load();  // let it complete naturally
    case ZE::StopEffect:    return Effect::None;
    }
    return Effect::None;
}

void on_identify(void*, esp_event_base_t, std::int32_t id, void* data)
{
    if (static_cast<blinds::zigbee::Event>(id) != blinds::zigbee::Event::Identify) {
        return;
    }
    const auto raw    = *static_cast<std::uint8_t*>(data);
    const auto zcl_e  = static_cast<blinds::zigbee::IdentifyEffect>(raw);
    const auto effect = map_zcl_effect(zcl_e);
    ESP_LOGI(TAG, "Identify effect 0x%02x → %d", raw, static_cast<int>(effect));
    start_effect(effect);
}

void configure_ledc()
{
    const ledc_timer_config_t timer_cfg{
        .speed_mode      = LEDC_MODE,
        .duty_resolution = LEDC_RESOLUTION,
        .timer_num       = LEDC_TIMER,
        .freq_hz         = LEDC_FREQUENCY_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
        .deconfigure     = false,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    const ledc_channel_config_t channel_cfg{
        .gpio_num   = LED_GPIO,
        .speed_mode = LEDC_MODE,
        .channel    = LEDC_CHANNEL,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER,
        // Start off (active-low → max duty = LED off).
        .duty       = LED_ACTIVE_LOW ? static_cast<std::uint32_t>(LEDC_MAX_DUTY) : 0u,
        .hpoint     = 0,
        .flags      = { .output_invert = 0 },
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_cfg));
}

}  // namespace

void start()
{
    configure_ledc();
    write_brightness(0);

    tick_timer = xTimerCreate("led_tick", TICK_MS, pdTRUE, nullptr, on_tick);
    configASSERT(tick_timer != nullptr);
    xTimerStart(tick_timer, 0);

    ESP_ERROR_CHECK(esp_event_handler_register(blinds::zigbee::EVENTS,
                                               static_cast<std::int32_t>(
                                                   blinds::zigbee::Event::Identify),
                                               &on_identify, nullptr));
    ESP_LOGI(TAG, "LED driver up on GPIO%d (active-%s)", LED_GPIO,
             LED_ACTIVE_LOW ? "low" : "high");
}

}  // namespace blinds::led
