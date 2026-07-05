#pragma once

#include <cstddef>

// TEMPORARY debugging aid: tee the ESP log stream into a RAM ring buffer and
// persist it to NVS, so logs from a normal (Zigbee) session can be read back
// over the WiFi debug console's /log endpoint after a reboot into debug mode —
// useful when the device is mounted and USB serial isn't available.
//
// Not meant to ship: it hooks the global log vprintf and writes NVS on a timer.

namespace hvmrf01::logtap {

// Install the log hook. Call once, as early as possible in app_main, so the
// capture covers the whole session. Idempotent-ish; call only once.
void install();

// Snapshot the current ring buffer into the NVS blob. Call periodically (normal
// mode only, so a debug-mode session doesn't overwrite the captured logs) and
// once more right before a reboot into debug mode.
void save();

// Copy the current session's ring buffer (chronological) into out; returns the
// number of bytes written (up to cap).
std::size_t copy_live(char* out, std::size_t cap);

// Copy the persisted (previous session) NVS blob into out; returns bytes read.
std::size_t copy_persisted(char* out, std::size_t cap);

}  // namespace hvmrf01::logtap
