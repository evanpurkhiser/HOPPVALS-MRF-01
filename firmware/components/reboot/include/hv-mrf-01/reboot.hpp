#pragma once

#include <expected>

#include "hv-mrf-01/config.hpp"

namespace hvmrf01::reboot {

enum class Mode
{
    Normal,
    Debug,
};

// Stop motion, select the next boot's radio mode, and schedule a near-future
// reset. Returning before esp_restart() lets Zigbee and WebSocket command
// responses flush instead of looking like command timeouts to their callers.
std::expected<void, config::Error> async(Mode mode, const char* why);

}  // namespace hvmrf01::reboot
