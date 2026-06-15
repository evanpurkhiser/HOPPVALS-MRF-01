#include "blinds/encoder.hpp"

#include <array>
#include <cstdint>

#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_err.h"
#include "esp_log.h"

namespace blinds::encoder {

namespace {

constexpr auto* TAG = "blinds.encoder";

// Per-motor encoder pin mapping (DESIGN.md). Wired with internal pull-ups
// since the common motor-shaft Hall encoders have open-drain outputs.
struct EncoderCfg
{
    gpio_num_t  pin_a;
    gpio_num_t  pin_b;
    const char* label;
};

constexpr EncoderCfg ENC_L{ .pin_a = GPIO_NUM_23, .pin_b = GPIO_NUM_16, .label = "L" }; // D5/D6
constexpr EncoderCfg ENC_R{ .pin_a = GPIO_NUM_22, .pin_b = GPIO_NUM_21, .label = "R" }; // D4/D3

constexpr std::array<EncoderCfg, 2> ENCODERS{ ENC_L, ENC_R };

// PCNT count limits. The hardware counter rolls between ±10000 and the
// `accum_count` flag tells PCNT to accumulate full rollovers internally,
// so pcnt_unit_get_count() returns the true unbounded count.
constexpr int PCNT_HIGH_LIMIT = 10000;
constexpr int PCNT_LOW_LIMIT  = -10000;

struct UnitState
{
    pcnt_unit_handle_t    unit      = nullptr;
    pcnt_channel_handle_t channel_a = nullptr;  // edge on A, level reads B
    pcnt_channel_handle_t channel_b = nullptr;  // edge on B, level reads A
};

std::array<UnitState, 2> units{};

constexpr std::size_t idx(Side s)
{
    return s == Side::Right ? 1 : 0;
}

void configure_unit(const EncoderCfg& cfg, UnitState& state)
{
    const gpio_config_t io_cfg{
        .pin_bit_mask = (1ULL << cfg.pin_a) | (1ULL << cfg.pin_b),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_cfg));

    const pcnt_unit_config_t unit_cfg{
        .low_limit     = PCNT_LOW_LIMIT,
        .high_limit    = PCNT_HIGH_LIMIT,
        .intr_priority = 0,
        .flags         = { .accum_count = true },
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &state.unit));

    const pcnt_glitch_filter_config_t filter_cfg{ .max_glitch_ns = 1000 };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(state.unit, &filter_cfg));

    const pcnt_chan_config_t cha_cfg{
        .edge_gpio_num  = cfg.pin_a,
        .level_gpio_num = cfg.pin_b,
        .flags          = {},
    };
    ESP_ERROR_CHECK(pcnt_new_channel(state.unit, &cha_cfg, &state.channel_a));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(state.channel_a,
                                                 PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                                 PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(state.channel_a,
                                                  PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                  PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    const pcnt_chan_config_t chb_cfg{
        .edge_gpio_num  = cfg.pin_b,
        .level_gpio_num = cfg.pin_a,
        .flags          = {},
    };
    ESP_ERROR_CHECK(pcnt_new_channel(state.unit, &chb_cfg, &state.channel_b));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(state.channel_b,
                                                 PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                                 PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(state.channel_b,
                                                  PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                  PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(state.unit, PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(state.unit, PCNT_LOW_LIMIT));

    ESP_ERROR_CHECK(pcnt_unit_enable(state.unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(state.unit));
    ESP_ERROR_CHECK(pcnt_unit_start(state.unit));
}

}  // namespace

void start()
{
    for (std::size_t i = 0; i < ENCODERS.size(); ++i) {
        configure_unit(ENCODERS[i], units[i]);
        ESP_LOGI(TAG, "PCNT quadrature reader up: %s A=GPIO%d B=GPIO%d",
                 ENCODERS[i].label, ENCODERS[i].pin_a, ENCODERS[i].pin_b);
    }
}

std::int32_t count(Side s)
{
    int n = 0;
    pcnt_unit_get_count(units[idx(s)].unit, &n);
    // Motor R's encoder A/B is wired with opposite quadrature direction
    // vs Motor L — same physical motion produces opposite count signs.
    // Negate here so both motors share a single convention: positive count
    // = "raise" direction. (Could be fixed by swapping the encoder A/B
    // wires on Motor R instead; doing it in software keeps the harness
    // simple and is just as effective.)
    return s == Side::Right ? -n : n;
}

void reset(Side s)
{
    if (s == Side::Both) {
        for (auto& u : units) pcnt_unit_clear_count(u.unit);
        return;
    }
    pcnt_unit_clear_count(units[idx(s)].unit);
}

}  // namespace blinds::encoder
