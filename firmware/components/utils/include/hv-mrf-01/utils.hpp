#pragma once

#include <cstddef>

namespace hvmrf01::utils {

// Stable per-device identifier: the low 4 bytes of the chip's EUI-64 (the
// 802.15.4 MAC) rendered as 8 lowercase hex chars. Ordered to match the tail of
// the IEEE long address a Zigbee hub (ZHA/Z2M) shows, so the same string keys
// the device on both its Zigbee and WiFi surfaces.
//
// Derived from eFuse, not generated, so it is fixed for the life of the silicon
// and survives reboots, a full NVS erase, and OTA flashes. Computed once on
// first call; the returned pointer is NUL-terminated and lives for the process.
const char* device_id();

// Render `in` as a DNS-safe hostname label into `out` (a buffer of `cap`
// bytes, always NUL-terminated). Lowercases ASCII letters, keeps digits, and
// collapses every run of other characters — spaces, punctuation, "Bedroom
// Right" — into a single hyphen, with leading/trailing hyphens trimmed. The
// result is truncated to fit `cap` (and any hyphen left dangling by truncation
// is dropped). Returns the number of characters written. An input that reduces
// to nothing (empty or all-separator) yields an empty string.
std::size_t slugify(const char* in, char* out, std::size_t cap);

// Log the full EUI-64 in both byte orders plus the derived device_id. The two
// orders exist because esp_read_mac and ZHA disagree on endianness; logging
// both lets us confirm device_id() really is the tail ZHA displays. Diagnostic
// only — call once at boot.
void log_identity();

}  // namespace hvmrf01::utils
