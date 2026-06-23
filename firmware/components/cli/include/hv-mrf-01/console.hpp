#pragma once

#include <string>

// Command console for the firmware. The same command set is reachable two
// ways: an interactive REPL over the XIAO's native USB-Serial-JTAG (normal
// bench use), and one-shot dispatch from the http_debug websocket (remote use
// in WiFi debug mode). Command output goes through emit(), which the websocket
// path captures into a string instead of writing to the serial port.
//
// Connect over USB with:  make monitor  (or screen /dev/cu.usbmodem* 115200)
// Run `help` once attached for the command list (the COMMANDS table in
// console.cpp is the source of truth).

namespace hvmrf01::console {

// Initialize the console and register all commands, then start the interactive
// USB-Serial-JTAG REPL. Used in normal mode. Call once, after motor + encoder.
void start();

// Initialize the console and register all commands *without* starting the USB
// REPL — for debug mode, where the websocket is the transport. Call once
// before run_line(). Mutually exclusive with start() (one console per boot).
void init_for_remote();

// Run a single command line, returning everything the command emitted as a
// string. Thread-safe via an internal lock; intended for the websocket handler.
std::string run_line(const std::string& line);

}  // namespace hvmrf01::console
