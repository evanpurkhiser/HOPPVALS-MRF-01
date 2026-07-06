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
// reset. The grace period lets Zigbee's Default Response/APS ack path and
// WebSocket replies flush instead of looking like command timeouts to callers.
std::expected<void, config::Error> async(Mode mode, const char* why);

}  // namespace hvmrf01::reboot
