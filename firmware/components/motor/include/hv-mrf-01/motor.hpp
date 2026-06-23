#pragma once

#include <cstdint>

#include "hv-mrf-01/types.hpp"

// Motor driver. Owns the two DRV8876 control pins for both Motor L (left)
// and Motor R (right) and exposes the drive() and debug surfaces. The
// closed-loop cover semantics (open/close/stop/go-to) live in the motion
// component, which routes ZCL cover commands onto this driver.
//
// The drivers run in PH/EN mode (PMODE strapped to GND on the PCB):
//   - PH  (direction): high = forward, low = reverse.
//   - EN  (speed):     PWM'd via LEDC. EN toggles drive ↔ brake (slow decay),
//                      so duty maps to average voltage. EN=0 = active brake.
//   - nSLEEP (shared): one GPIO gates *both* drivers. High = awake (drive or
//                      brake), low = coast (both H-bridges Hi-Z). Because it's
//                      shared, coast is all-or-nothing across both motors —
//                      which is fine: the two cords must move in lockstep, so
//                      there's never a reason to coast just one.
//
namespace hvmrf01::motor {

// Re-export the shared side enum so callers can keep using motor::Side; the
// canonical definition lives in hvmrf01::types (hv-mrf-01/types.hpp) so the
// sensor components don't have to depend on this driver just for the type.
using Side = hvmrf01::Side;

// Configure GPIO + LEDC for both motors and enable the drivers (nSLEEP high,
// landing in brake). Call once from app_main.
void start();

// Put both drivers to sleep (nSLEEP low): outputs go Hi-Z and the motors
// cannot be driven until woken again. Stop the motion controller first, or its
// next tick re-wakes nSLEEP. Used to guarantee motors are de-energized during
// an OTA write + reboot.
void disable();

// H-bridge output mode for drive(). Forward/Reverse are directions; Brake and
// Coast are stop modes — hence "mode" rather than "direction".
enum class Mode : std::uint8_t { Forward, Reverse, Brake, Coast };

// The primary programmatic drive surface — the motion controller calls this
// every tick. Applies mode + duty atomically for one motor (or both):
//   Forward/Reverse — wake the drivers (nSLEEP high), set PH, PWM EN.
//   Brake           — wake the drivers, EN=0 (low-side short, active hold).
//   Coast           — nSLEEP low. Affects BOTH motors regardless of `s`.
// Duty values are clamped 0..100. No state machine, no auto-stop: whatever you
// set stays set until changed again.
void drive(Side s, Mode m, int duty_pct);

// Interactive bench surface for the serial console — the same drive primitives
// decomposed into independent setters (set direction without touching duty,
// tweak the PWM frequency, read state back) for hands-on PWM tuning, whine
// debugging, and brake validation.
//
// Note: these fight the motion controller if it's running. Use `motion stop`
// from the CLI first, or expect tug-of-war.
namespace debug {

void set_forward(Side s = Side::Both);   // PH=H, EN=duty — drive at current duty
void set_reverse(Side s = Side::Both);   // PH=L, EN=duty
void set_brake  (Side s = Side::Both);   // EN=0, nSLEEP=H — short-brake (active hold)
void set_coast  (Side s = Side::Both);   // nSLEEP=L — Hi-Z (freewheel); always both

void set_duty_pct(int pct, Side s = Side::Both);  // 0-100, clamped
bool set_freq_hz (int hz);                         // shared timer — both motors

// Drive the shared nSLEEP line. on=true wakes both drivers (required before
// any drive/brake takes effect); on=false coasts both.
void set_enabled(bool on);
bool enabled();

void print_state();                                // dumps both motors

}  // namespace debug

}  // namespace hvmrf01::motor
