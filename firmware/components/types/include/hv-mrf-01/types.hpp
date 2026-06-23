#pragma once

#include <cstdint>

// Shared vocabulary types — no behavior, owned by no driver. Lets the sensor
// components (encoder, current_sense) and the motion controller address a motor
// by side without depending on the motor driver just to borrow an enum.

namespace hvmrf01 {

// Which physical motor — left or right side of the blind. Cover commands drive
// both in lockstep; lower-level calls can target one side.
enum class Side : std::uint8_t { Left, Right, Both };

}  // namespace hvmrf01
