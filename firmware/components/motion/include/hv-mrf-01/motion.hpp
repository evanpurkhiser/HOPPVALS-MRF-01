#pragma once

#include <cstdint>

// Closed-loop velocity controller for the two blind motors.
//
// Architecture: a 100 Hz FreeRTOS task samples both encoders, computes
// per-motor measured RPM, runs an independent PI+feedforward speed loop
// per motor, and drives the motor PWM via `hvmrf01::motor::raw::drive`.
//
// Cross-coupled sync: each motor's RPM setpoint is biased by K_SYNC ×
// (count_other - count_me). The lagging motor speeds up and the leading
// motor slows down — symmetric, no master. See DESIGN.md "Two-motor
// synchronization (C)" for the rationale.
//
// Safety: if the count delta between motors exceeds SYNC_FAULT_LIMIT we
// brake both and enter a faulted state. Subsequent set_target() calls
// are ignored until stop() clears the fault.
//
// Scope (v0): constant target RPM in one direction. Trajectory generation
// (trapezoid + easing, spring) and position targeting layer on top later.

namespace hvmrf01::motion {

enum class Direction : std::uint8_t { Raise, Lower, Stop };

// Set the target output-shaft RPM and direction. Thread-safe; the control
// task picks this up on the next 10 ms tick. RPM is the post-gearbox shaft
// speed (matches encoder::COUNTS_PER_OUTPUT_REV convention).
//
// Direction::Stop is equivalent to calling stop() — both motors brake.
void set_target(int rpm, Direction d);

// Halt the controller, brake both motors, zero the PI integrators, and
// clear any fault flag.
void stop();

// Start the 100 Hz control task. Call once from app_main, after motor and
// encoder are up.
void start();

}  // namespace hvmrf01::motion
