#pragma once

// User-LED driver. Listens on the blinds::zigbee event bus for Identify
// events and runs the corresponding ZCL visual effect on GPIO15.
//
// The XIAO ESP32-C6 has a single user LED (yellow/orange, active-low). We
// drive it via LEDC PWM so we can do smooth breathing fades, not just
// hard on/off.

namespace blinds::led {

// Initialize LEDC, register the Identify event handler. Call once from
// app_main, after the default event loop is created.
void start();

}  // namespace blinds::led
