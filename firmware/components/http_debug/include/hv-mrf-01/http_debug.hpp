#pragma once

// WiFi debug mode. When the device boots with the one-shot debug flag set
// (see config::take_debug_boot), app_main calls run() instead of starting
// Zigbee. It joins the configured WiFi network as a station and exposes the
// command console over a websocket, so the full CLI (spin, ramp, motion,
// config, …) is reachable remotely without a USB cable.
//
// The 2.4 GHz radio is shared with Zigbee, so this and Zigbee never run in the
// same boot. If the network can't be joined within the configured timeout,
// run() reboots — and because the debug flag is one-shot (already cleared on
// entry), the device comes back up in normal Zigbee mode.
//
// Transport: HTTP server on port 80 with a websocket at /ws. The client sends
// a command line as a text frame; the server runs it and returns the captured
// output as a text frame. No OTA, auth, or TLS yet.

namespace hvmrf01::http_debug {

// Enter debug mode: bring up WiFi STA, and on success start the websocket
// console + OTA-push server. On WiFi failure this reboots (back into normal
// mode) and does not return. Call once from app_main; the HTTP server runs in
// its own task afterward, so this returns once the server is up.
void run();

// Mark the running image valid so the bootloader won't roll it back (and so the
// next OTA is permitted). A no-op unless the image is in "pending verify" after
// an OTA. Called here once the debug server is up; call it from normal mode too
// once the firmware proves healthy (e.g. after the Zigbee network rejoin).
void confirm_running_image();

}  // namespace hvmrf01::http_debug
