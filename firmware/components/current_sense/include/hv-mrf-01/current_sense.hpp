#pragma once

#include <cstdint>

#include "hv-mrf-01/types.hpp"

// Motor current sensing via the DRV8876 IPROPI pins. Each driver mirrors its
// output current onto IPROPI, which the PCB turns into a voltage across a
// 560 Ω resistor (A_IPROPI = 1000 µA/A → 0.56 V/A) and routes to an ESP32-C6
// ADC1 channel:
//
//   IPROPI_L → GPIO1 (A1, ADC1_CH1)
//   IPROPI_R → GPIO0 (A0, ADC1_CH0)
//
// A 100 Hz task samples both channels (same cadence as the speed loop) so the
// motion controller can correlate commanded duty, measured RPM, and measured
// current for confident stall detection. Readings are the latest instantaneous
// sample; callers that want a stable number should average over a window.

namespace hvmrf01::current_sense {

// Reuse the shared Side enum so callers address either motor uniformly.
using Side = hvmrf01::Side;

// Configure ADC1 + calibration and start the 100 Hz sampling task. Call once
// from app_main.
void start();

// Latest sampled motor current in milliamps. Side::Both is invalid (returns 0)
// — callers must specify which motor to read.
std::int32_t current_ma(Side s);

// Latest raw IPROPI voltage in millivolts (diagnostic / calibration aid).
std::int32_t voltage_mv(Side s);

}  // namespace hvmrf01::current_sense
