#pragma once

// Interactive serial REPL over the XIAO's native USB-Serial-JTAG. Useful
// for hands-on motor tuning (PWM duty, frequency, direction) without
// reflashing between iterations.
//
// Connect from your host with:  make monitor   (or screen /dev/cu.usbmodem* 115200)
//
// Commands once attached:
//   fwd / rev / brake / coast    — drive Motor A in that mode
//   pwm <0-100>                  — set PWM duty %
//   freq <hz>                    — set PWM frequency (e.g. 25000)
//   state                        — print current pin levels + duty + freq
//   help                         — list all commands

namespace blinds::console {

void start();

}  // namespace blinds::console
