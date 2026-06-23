#include "hv-mrf-01/current_sense.hpp"

#include <array>
#include <atomic>
#include <cstdint>

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace hvmrf01::current_sense {

namespace {

constexpr auto* TAG = "hv-mrf-01.current";

// IPROPI → current conversion (PCB values, DRV8876 datasheet §7.3.3.1)
// V_IPROPI = I_OUT × A_IPROPI × R_IPROPI = I_OUT × 1000 µA/A × 560 Ω
//          = I_OUT × 0.56 V/A. Invert: I_OUT(mA) = V(mV) × 1000 / 560.
//
// Validity (datasheet-confirmed): IPROPI senses low-side FET current. In our
// PH/EN drive↔brake (slow-decay) PWM pattern a low-side FET conducts in both
// phases, so IPROPI is *continuously* valid — async sampling at 100 Hz needs
// no PWM synchronization. BUT in coast/Hi-Z (shared nSLEEP low) IPROPI reads
// ~0; that's "no data", not "zero current". Accuracy is gain-limited (~±6.5%)
// above 0.5 A and offset-limited (fixed ±~7.5 mA) below ~150 mA.

constexpr int R_IPROPI_OHMS = 560;
constexpr int A_IPROPI_UA_A = 1000;  // µA of IPROPI per A of output

constexpr std::int32_t mv_to_ma(int mv)
{
    return static_cast<std::int32_t>(mv) * 1'000'000 / (A_IPROPI_UA_A * R_IPROPI_OHMS);
}

// ADC channel mapping
// IPROPI_L → GPIO1 → ADC1_CH1, IPROPI_R → GPIO0 → ADC1_CH0.

struct Channel
{
    adc_channel_t chan;
    const char*   label;
};

constexpr Channel CH_L{ ADC_CHANNEL_1, "L" };
constexpr Channel CH_R{ ADC_CHANNEL_0, "R" };
constexpr std::array<Channel, 2> CHANNELS{ CH_L, CH_R };  // [0]=L, [1]=R

constexpr std::size_t idx(Side s) { return s == Side::Right ? 1 : 0; }

constexpr int        SAMPLE_HZ     = 100;
constexpr TickType_t SAMPLE_PERIOD = pdMS_TO_TICKS(1000 / SAMPLE_HZ);

constexpr UBaseType_t   TASK_PRIO = 6;
constexpr std::uint32_t STACK_SZ  = 3072;

// State

adc_oneshot_unit_handle_t adc_unit = nullptr;
std::array<adc_cali_handle_t, 2> cali{ nullptr, nullptr };

std::array<std::atomic<std::int32_t>, 2> latest_mv{};
std::array<std::atomic<std::int32_t>, 2> latest_ma{};

int read_mv(std::size_t i)
{
    int raw = 0;
    if (adc_oneshot_read(adc_unit, CHANNELS[i].chan, &raw) != ESP_OK) {
        return latest_mv[i].load();  // keep last good reading on a transient error
    }
    int mv = 0;
    adc_cali_raw_to_voltage(cali[i], raw, &mv);
    return mv;
}

void sample_task(void*)
{
    TickType_t wake = xTaskGetTickCount();
    while (true) {
        vTaskDelayUntil(&wake, SAMPLE_PERIOD);

        for (std::size_t i = 0; i < CHANNELS.size(); ++i) {
            const int          mv = read_mv(i);
            const std::int32_t ma = mv_to_ma(mv);
            latest_mv[i].store(mv);
            latest_ma[i].store(ma);
        }
    }
}

void configure_channel(std::size_t i)
{
    const adc_oneshot_chan_cfg_t chan_cfg{
        .atten    = ADC_ATTEN_DB_12,        // full ~0..3.1 V input range
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_unit, CHANNELS[i].chan, &chan_cfg));

    const adc_cali_curve_fitting_config_t cali_cfg{
        .unit_id  = ADC_UNIT_1,
        .chan     = CHANNELS[i].chan,
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_cfg, &cali[i]));
}

}  // namespace

void start()
{
    const adc_oneshot_unit_init_cfg_t init_cfg{
        .unit_id  = ADC_UNIT_1,
        .clk_src  = ADC_DIGI_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_unit));

    for (std::size_t i = 0; i < CHANNELS.size(); ++i) configure_channel(i);

    const BaseType_t ok = xTaskCreate(&sample_task, "current", STACK_SZ,
                                      nullptr, TASK_PRIO, nullptr);
    configASSERT(ok == pdPASS);

    ESP_LOGI(TAG, "current sense up: L=ADC1_CH%d R=ADC1_CH%d @ %d Hz",
             CH_L.chan, CH_R.chan, SAMPLE_HZ);
}

std::int32_t current_ma(Side s)
{
    if (s == Side::Both) return 0;
    return latest_ma[idx(s)].load();
}

std::int32_t voltage_mv(Side s)
{
    if (s == Side::Both) return 0;
    return latest_mv[idx(s)].load();
}

}  // namespace hvmrf01::current_sense
