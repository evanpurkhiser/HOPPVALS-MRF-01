#pragma once

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

// Log the full EUI-64 in both byte orders plus the derived device_id. The two
// orders exist because esp_read_mac and ZHA disagree on endianness; logging
// both lets us confirm device_id() really is the tail ZHA displays. Diagnostic
// only — call once at boot.
void log_identity();

}  // namespace hvmrf01::utils
