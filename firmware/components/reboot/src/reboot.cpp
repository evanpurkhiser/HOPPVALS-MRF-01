#include "hv-mrf-01/reboot.hpp"

#include "esp_check.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "hv-mrf-01/motion.hpp"
#include "hv-mrf-01/motor.hpp"

namespace hvmrf01::reboot {

namespace {

constexpr auto* TAG = "hv-mrf-01.reboot";
constexpr auto  REBOOT_DELAY_US = 1000 * 1000;

bool rebooting = false;
esp_timer_handle_t reset_timer = nullptr;

void do_reset(void*)
{
    esp_restart();
}

void ensure_timer()
{
    if (reset_timer != nullptr) {
        return;
    }

    const esp_timer_create_args_t args{.callback              = &do_reset,
                                       .arg                   = nullptr,
                                       .dispatch_method       = ESP_TIMER_TASK,
                                       .name                  = "reset",
                                       .skip_unhandled_events = false};
    ESP_ERROR_CHECK(esp_timer_create(&args, &reset_timer));
}

const char* mode_label(Mode mode)
{
    return mode == Mode::Debug ? "debug" : "normal Zigbee";
}

}  // namespace

std::expected<void, config::Error> async(Mode mode, const char* why)
{
    if (mode == Mode::Debug) {
        if (auto r = config::request_debug_boot(); !r) {
            return std::unexpected(r.error());
        }
    } else {
        config::take_debug_boot();
    }

    ESP_LOGW(TAG, "%s; rebooting into %s mode", why, mode_label(mode));
    motion::stop();
    motor::disable();

    if (rebooting) {
        return {};
    }
    rebooting = true;

    ensure_timer();
    ESP_ERROR_CHECK(esp_timer_start_once(reset_timer, REBOOT_DELAY_US));
    return {};
}

}  // namespace hvmrf01::reboot
