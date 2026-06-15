#pragma once

#include <cstdint>

#include "blinds/motor.hpp"

// Quadrature encoder reader using the ESP32-C6 PCNT (Pulse Counter)
// peripheral. PCNT decodes A+B in hardware with no CPU cost — we just
// read the accumulated count whenever we want.
//
// Two PCNT units are allocated: unit 0 → Motor L (D5/D6), unit 1 →
// Motor R (D4/D3). Each unit is independent.
//
// "Counts" are raw 4x-decoded ticks. One full output-shaft revolution
// (post-gearbox) corresponds to COUNTS_PER_OUTPUT_REV.

namespace blinds::encoder {

// Both motors share the same encoder PPR and gearbox ratio:
//   encoder PPR (7) × quadrature multiplier (4) × gearbox ratio (32) = 896
constexpr std::int32_t COUNTS_PER_OUTPUT_REV = 896;

// Reuse the motor::Side enum so callers can address either motor uniformly.
using Side = blinds::motor::Side;

// Initialize PCNT for both motors. Call once from app_main.
void start();

// Current signed count for one motor, since its last reset. Side::Both is
// invalid here — callers must specify which motor to read.
std::int32_t count(Side s);

// Zero the counter(s). Side::Both resets both encoders simultaneously.
void reset(Side s = Side::Both);

}  // namespace blinds::encoder
