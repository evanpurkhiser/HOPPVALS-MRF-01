#pragma once

#include <cstdint>
#include <expected>

// Device configuration: compile-time defaults overlaid with persisted
// overrides read from NVS at boot.
//
// Layering / precedence (lowest → highest):
//   1. struct member initializers below   (compile-time defaults)
//   2. values persisted in NVS             (applied by init())
//   3. runtime overrides via save()        (e.g. CLI tuning)
//
// Access is lock-free via a double-buffered active pointer: get() returns a
// const reference to the live config, and save() atomically publishes a new
// one. Hot-path readers (the 100 Hz motion task) snapshot get() into a local
// and re-snapshot only when generation() changes, so a mid-tick swap can
// never tear a read.
//
// Storage: each sub-struct maps to an NVS namespace and each field to a key,
// so adding a field is backward-compatible (a missing key simply keeps its
// default) and individual values stay human-inspectable.

namespace hvmrf01::config {

// Failure modes for the fallible config operations (init / save). Reads via
// get() are infallible — defaults guarantee a valid config at all times.
enum class Error : std::uint8_t {
    NvsInit,   // nvs_flash_init() failed even after an erase-and-retry
    NvsOpen,   // could not open the NVS namespace for writing
    NvsWrite,  // a key write failed
    NvsCommit, // nvs_commit() failed to flush the namespace
};

// Closed-loop motion controller tuning. These are the constants the PI +
// feedforward speed loop reads every tick; re-fit them under real cord load
// via the diagnostics CLI. Stored in NVS namespace "motion".
struct Motion {
    // Feedforward slope: percent PWM duty applied per RPM of target speed.
    // Carries the bulk of the command so the integrator only trims. Equal to
    // 1 / (RPM-per-%duty); ~0.33 measured open-loop under cord load.
    float duty_per_rpm = 0.33f;

    // Static feedforward offset (% duty) added on top of the slope to overcome
    // direction-dependent breakaway. Measured open-loop: raising needs ~10% to
    // start moving (friction + gravity), lowering ~0 (gravity assists). Applied
    // only while actively driving, so the integrator starts near the right duty
    // instead of winding up from zero each move.
    float ff_offset_raise_pct = 10.0f;
    float ff_offset_lower_pct = 0.0f;

    // Proportional gain: percent duty added per RPM of speed error. Provides
    // the immediate response; raise for snappier tracking, lower if it whines
    // or oscillates around the setpoint.
    float kp = 0.10f;

    // Integral gain: percent duty added per RPM-second of accumulated error.
    // Eliminates steady-state offset (friction, FF mismatch). Too high winds
    // up and overshoots — the "fast then slow" surge on start.
    float ki = 0.50f;

    // Anti-windup clamp on the integrator accumulator (in RPM-seconds). Bounds
    // how much duty the integral term can contribute (Ki × i_max) so it can't
    // saturate the output and lag on direction changes.
    float i_max = 100.0f;

    // Cross-coupling sync gain: RPM bias added to one motor's setpoint per
    // encoder count it lags the other. Keeps the two sides mechanically in
    // step. Set to 0 to tune each motor as an independent speed loop first.
    float k_sync = 0.05f;

    // Open/close speed in output-shaft RPM, used by the Zigbee cover Open and
    // Close commands. The post-gearbox shaft speed, not motor RPM.
    int cover_rpm = 40;

    // Sync watchdog limit: if the two encoders diverge by more than this many
    // counts while actively driving, fault and brake (a cord slipped or one
    // side jammed). ~896 counts = one output revolution.
    int sync_fault_limit = 200;

    // Stall watchdog sensitivity: a motor whose per-tick |Δcount| stays at or
    // below this is treated as "not moving". 1 absorbs encoder jitter.
    int stall_delta_max = 1;

    // Stall watchdog trip time: both motors must read stopped this many
    // consecutive milliseconds (while commanded to move) before faulting, so
    // we don't burn the motor against a hard endpoint or jam.
    int stall_fault_ms = 100;

    // Startup grace: the stall watchdog is suppressed for this long after
    // motion begins, covering motor stiction plus the PI integrator's ramp-up
    // before counts start accumulating. Prevents false stalls on heavy starts.
    int startup_grace_ms = 1500;

    // Homing: open-loop duty each motor is driven upward at, against the top
    // hard stop. High enough to break stiction and move immediately so the
    // settle detection is crisp.
    int home_duty_pct = 50;

    // A motor is "homed" once its encoder has been stopped (per-tick |Δcount|
    // <= stall_delta_max) continuously for this long, i.e. it's wedged against
    // the top stop. Its encoder is then zeroed (top = 0).
    int home_settle_ms = 250;

    // Overall safety cap on a homing run: if a motor never settles within this
    // (free-spinning, encoder fault), homing aborts and brakes.
    int home_timeout_s = 30;

    // Millimetres of cord travel per output-shaft revolution (effective spool
    // circumference). The one calibration mapping encoder counts to physical
    // distance for go-to-position. Measure: home, drive a known number of revs,
    // measure the drop, divide. 0 = uncalibrated; go_to_mm refuses until set.
    float mm_per_rev = 0.0f;

    // Hard limit on downward travel from the homed top, in mm — the full range
    // of the blind. go_to_pct(100) maps to this, and downward motion never
    // passes it. Per-device (each blind's drop may differ).
    float hard_stop_mm = 1600.0f;

    // Optional operational down limit, in mm, for a blind with an obstruction
    // below: downward motion (go_to_mm and a manual lower) stops here instead
    // of at hard_stop_mm, even though the hard range is larger. 0 = unset (the
    // hard stop is the only down limit). Should be < hard_stop_mm when set.
    float soft_stop_mm = 0.0f;
};

// WiFi station credentials for debug mode. The radio is shared with Zigbee,
// so WiFi only comes up when the device boots into debug mode (see the
// one-shot flag below); normal operation never touches these. Stored in NVS
// namespace "network". Provision them once over USB with the `config` CLI.
struct Network {
    // SSID of the 2.4 GHz network to join in debug mode. 32 chars + NUL is the
    // 802.11 maximum.
    char ssid[33] = "";

    // WPA2 passphrase for that network. 63 chars + NUL is the PSK maximum.
    // Stored in plaintext NVS — fine on a personal device; switch to an
    // encrypted NVS partition if that ever matters.
    char pass[64] = "";

    // How long debug mode waits for the WiFi association before giving up and
    // rebooting back into normal Zigbee mode (so a bad SSID can't strand an
    // installed device that has no USB access).
    int connect_timeout_s = 30;
};

// The full device configuration. Grows by composition — add a sub-struct
// (e.g. Ota) and a matching NVS namespace as features land.
struct Config {
    Motion  motion;
    Network network;
};

// ── One-shot "boot into debug mode" flag ──────────────────────────────────
//
// A persisted boolean, separate from Config, that records the intent to enter
// debug mode on the next boot. Set it (over Zigbee or the CLI) and reboot;
// app_main calls take_debug_boot() early, which reads *and clears* it so the
// intent is consumed exactly once — any later crash or power-cycle comes back
// up in normal Zigbee mode rather than looping into debug.

// Persist the request to enter debug mode on the next boot.
std::expected<void, Error> request_debug_boot();

// Read and clear the one-shot flag. Returns true if debug mode was requested.
// Call after init() (it needs the NVS flash layer up); the flag lives in its
// own namespace, independent of the published Config.
bool take_debug_boot();

// Bring up the NVS default partition (erasing and retrying once if its format
// is stale) and overlay any persisted values onto the defaults, publishing the
// result as the active config. Call once, early in app_main, before any
// consumer reads get(). On failure the defaults remain active.
std::expected<void, Error> init();

// A snapshot of the live, process-wide configuration. Returned *by value*: the
// active config lives in a recycled double-buffer slot, so handing out a
// reference would let a later save() overwrite the slot mid-read. A copy is
// cheap and every caller wants a stable view anyway. Valid before init() — it
// returns the compile-time defaults until init() publishes the merged config.
Config get();

// Monotonic counter bumped on every publish (init/save). Hot-path readers
// (the motion task) compare it against a cached value to know when to
// re-snapshot get(), so they avoid copying on every tick.
std::uint32_t generation();

// Persist `cfg` to NVS field-by-field and publish it as the new active config.
// Returns the merged config to its caller on success.
std::expected<void, Error> save(const Config& cfg);

}  // namespace hvmrf01::config
