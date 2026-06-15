#pragma once

#include <cstdint>

// Motor driver. Owns the two DRV8876 control pins for both Motor L (left)
// and Motor R (right) and exposes cover semantics to the Zigbee component.
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
// v1 behavior (closed-loop): cover commands forward to the motion
// controller, which drives both motors via the `raw` sub-namespace below.
//   - Cover Open  → motion::set_target(40, Direction::Raise)
//   - Cover Close → motion::set_target(40, Direction::Lower)
//   - Cover Stop  → motion::stop()
//   - GoToPercent → not implemented yet; returns Failure.

namespace hvmrf01::motor {

// Which physical motor — left or right side of the blind. Cover commands
// always drive both in lockstep; the debug + raw surfaces let you target one.
enum class Side : std::uint8_t { Left, Right, Both };

// Configure GPIO + LEDC for both motors, enable the drivers (nSLEEP high,
// landing in brake), and register cover handlers with the zigbee component.
// Call once from app_main, before zigbee::start().
void start();

// ── Raw drive surface ────────────────────────────────────────────────────
// The closed-loop motion controller calls these every tick. They behave
// like the debug surface (no auto-stop, no state machine), but skip the
// "cancel auto-stop" courtesy that the debug surface adds — the motion
// controller never engages it. Duty values are clamped 0..100.
namespace raw {

enum class Direction : std::uint8_t { Forward, Reverse, Brake, Coast };

// Apply direction + duty atomically for one motor (or both).
//   Forward/Reverse — wake the drivers (nSLEEP high), set PH, PWM EN.
//   Brake           — wake the drivers, EN=0 (low-side short, active hold).
//   Coast           — nSLEEP low. Affects BOTH motors regardless of `s`.
void drive(Side s, Direction d, int duty_pct);

}  // namespace raw

// Raw control surface for the serial console. Bypasses cover semantics:
// no auto-stop timer, no state machine. Whatever you set stays set until
// changed again. Use for PWM tuning / whine debugging / brake validation.
//
// Note: these fight the motion controller if it's running. Use `motion
// stop` from the CLI first, or expect tug-of-war.
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
