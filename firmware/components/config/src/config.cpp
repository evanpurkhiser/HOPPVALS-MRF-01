#include "hv-mrf-01/config.hpp"

#include <atomic>
#include <bit>
#include <cstdint>

#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

namespace hvmrf01::config {

namespace {

constexpr auto* TAG = "hv-mrf-01.config";

// NVS namespaces, one per config sub-struct. Keys below mirror the field
// names (each ≤ 15 chars, the NVS key limit).
constexpr auto* NS_MOTION  = "motion";
constexpr auto* NS_NETWORK = "network";

// Dedicated namespace + key for the one-shot debug-boot flag (kept apart from
// Config so it can be read before init() and cleared on its own).
constexpr auto* NS_SYS         = "sys";
constexpr auto* KEY_DEBUG_BOOT = "dbg_boot";

// Double buffer
//
// Two storage slots and an atomic pointer to the active one. Writers (init /
// save, single-threaded) fill the inactive slot then publish it with a release
// store; readers acquire-load the pointer. The active slot is never mutated in
// place, so a reader copying through the pointer always sees a consistent
// Config even if a publish races it.

std::atomic<const Config*> active{ nullptr };
std::atomic<std::uint32_t> gen{ 0 };
Config                     slots[2]{};
int                        write_slot = 0;

// Compile-time defaults, returned by get() before init() has published.
const Config defaults{};

void publish(const Config& cfg)
{
    slots[write_slot] = cfg;
    active.store(&slots[write_slot], std::memory_order_release);
    write_slot ^= 1;
    gen.fetch_add(1, std::memory_order_release);
}

// NVS field helpers
//
// NVS has no native float type, so floats round-trip through their u32 bit
// pattern. A missing key (ESP_ERR_NVS_NOT_FOUND) leaves the default in place.

void load_f32(nvs_handle_t h, const char* key, float& out)
{
    std::uint32_t raw = 0;
    if (nvs_get_u32(h, key, &raw) == ESP_OK) {
        out = std::bit_cast<float>(raw);
    }
}

void load_i32(nvs_handle_t h, const char* key, int& out)
{
    std::int32_t v = 0;
    if (nvs_get_i32(h, key, &v) == ESP_OK) {
        out = v;
    }
}

esp_err_t save_f32(nvs_handle_t h, const char* key, float v)
{
    return nvs_set_u32(h, key, std::bit_cast<std::uint32_t>(v));
}

esp_err_t save_i32(nvs_handle_t h, const char* key, int v)
{
    return nvs_set_i32(h, key, v);
}

// Overlay persisted Motion values onto `cfg.motion`. Best-effort: if the
// namespace doesn't exist yet (first boot) or a key is absent, the defaults
// already in `cfg` stand.
void load_motion(Config& cfg)
{
    nvs_handle_t h;
    const esp_err_t err = nvs_open(NS_MOTION, NVS_READONLY, &h);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "no persisted '%s' config (%s); using defaults",
                 NS_MOTION, esp_err_to_name(err));
        return;
    }

    load_f32(h, "duty_per_rpm", cfg.motion.duty_per_rpm);
    load_f32(h, "ff_off_raise", cfg.motion.ff_offset_raise_pct);
    load_f32(h, "ff_off_lower", cfg.motion.ff_offset_lower_pct);
    load_f32(h, "ff_trim_l", cfg.motion.ff_trim_l);
    load_f32(h, "ff_trim_r", cfg.motion.ff_trim_r);
    load_f32(h, "kp", cfg.motion.kp);
    load_f32(h, "ki", cfg.motion.ki);
    load_f32(h, "i_max", cfg.motion.i_max);
    load_f32(h, "k_sync", cfg.motion.k_sync);
    load_i32(h, "cover_rpm", cfg.motion.cover_rpm);
    load_i32(h, "sync_fault", cfg.motion.sync_fault_limit);
    load_i32(h, "stall_delta", cfg.motion.stall_delta_max);
    load_i32(h, "stall_ms", cfg.motion.stall_fault_ms);
    load_i32(h, "grace_ms", cfg.motion.startup_grace_ms);
    load_i32(h, "home_duty", cfg.motion.home_duty_pct);
    load_i32(h, "home_settle", cfg.motion.home_settle_ms);
    load_i32(h, "home_to", cfg.motion.home_timeout_s);
    load_f32(h, "mm_per_rev", cfg.motion.mm_per_rev);
    load_f32(h, "hard_stop_mm", cfg.motion.hard_stop_mm);
    load_f32(h, "soft_stop_mm", cfg.motion.soft_stop_mm);
    load_f32(h, "goto_slow_mm", cfg.motion.goto_slow_mm);
    load_i32(h, "goto_min_rpm", cfg.motion.goto_min_rpm);
    load_f32(h, "goto_tol_mm", cfg.motion.goto_tol_mm);

    nvs_close(h);
}

// Load a NUL-terminated string key into a fixed char buffer. A missing key or
// an oversized stored value leaves the default in place.
void load_str(nvs_handle_t h, const char* key, char* out, std::size_t cap)
{
    std::size_t len = cap;
    if (nvs_get_str(h, key, nullptr, &len) == ESP_OK && len <= cap) {
        nvs_get_str(h, key, out, &len);
    }
}

void load_network(Config& cfg)
{
    nvs_handle_t h;
    const esp_err_t err = nvs_open(NS_NETWORK, NVS_READONLY, &h);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "no persisted '%s' config (%s); using defaults",
                 NS_NETWORK, esp_err_to_name(err));
        return;
    }

    load_str(h, "ssid", cfg.network.ssid, sizeof(cfg.network.ssid));
    load_str(h, "pass", cfg.network.pass, sizeof(cfg.network.pass));
    load_i32(h, "conn_to", cfg.network.connect_timeout_s);

    nvs_close(h);
}

std::expected<void, Error> save_network(const Network& n)
{
    nvs_handle_t h;
    if (nvs_open(NS_NETWORK, NVS_READWRITE, &h) != ESP_OK) {
        return std::unexpected(Error::NvsOpen);
    }

    const esp_err_t writes = nvs_set_str(h, "ssid", n.ssid) |
                             nvs_set_str(h, "pass", n.pass) |
                             save_i32(h, "conn_to", n.connect_timeout_s);
    if (writes != ESP_OK) {
        nvs_close(h);
        return std::unexpected(Error::NvsWrite);
    }

    const esp_err_t committed = nvs_commit(h);
    nvs_close(h);
    if (committed != ESP_OK) {
        return std::unexpected(Error::NvsCommit);
    }
    return {};
}

std::expected<void, Error> save_motion(const Motion& m)
{
    nvs_handle_t h;
    if (nvs_open(NS_MOTION, NVS_READWRITE, &h) != ESP_OK) {
        return std::unexpected(Error::NvsOpen);
    }

    const esp_err_t writes =
        save_f32(h, "duty_per_rpm", m.duty_per_rpm) |
        save_f32(h, "ff_off_raise", m.ff_offset_raise_pct) |
        save_f32(h, "ff_off_lower", m.ff_offset_lower_pct) | save_f32(h, "kp", m.kp) |
        save_f32(h, "ki", m.ki) | save_f32(h, "i_max", m.i_max) |
        save_f32(h, "k_sync", m.k_sync) | save_i32(h, "cover_rpm", m.cover_rpm) |
        save_i32(h, "sync_fault", m.sync_fault_limit) |
        save_i32(h, "stall_delta", m.stall_delta_max) |
        save_i32(h, "stall_ms", m.stall_fault_ms) |
        save_i32(h, "grace_ms", m.startup_grace_ms) |
        save_i32(h, "home_duty", m.home_duty_pct) |
        save_i32(h, "home_settle", m.home_settle_ms) |
        save_i32(h, "home_to", m.home_timeout_s) |
        save_f32(h, "mm_per_rev", m.mm_per_rev) |
        save_f32(h, "hard_stop_mm", m.hard_stop_mm) |
        save_f32(h, "soft_stop_mm", m.soft_stop_mm) |
        save_f32(h, "goto_slow_mm", m.goto_slow_mm) |
        save_i32(h, "goto_min_rpm", m.goto_min_rpm) |
        save_f32(h, "goto_tol_mm", m.goto_tol_mm);

    if (writes != ESP_OK) {
        nvs_close(h);
        return std::unexpected(Error::NvsWrite);
    }

    const esp_err_t committed = nvs_commit(h);
    nvs_close(h);
    if (committed != ESP_OK) {
        return std::unexpected(Error::NvsCommit);
    }
    return {};
}

}  // namespace

std::expected<void, Error> init()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition stale (%s); erasing", esp_err_to_name(err));
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        return std::unexpected(Error::NvsInit);
    }

    Config cfg{};  // defaults
    load_motion(cfg);
    load_network(cfg);
    publish(cfg);

    ESP_LOGI(TAG, "config loaded: duty_per_rpm=%.2f kp=%.2f ki=%.2f i_max=%.0f "
                  "k_sync=%.3f cover_rpm=%d",
             cfg.motion.duty_per_rpm, cfg.motion.kp, cfg.motion.ki,
             cfg.motion.i_max, cfg.motion.k_sync, cfg.motion.cover_rpm);
    return {};
}

Config get()
{
    // Copy under the acquire-loaded pointer: the slot can be recycled by a
    // later publish(), but the copy completes against a consistent snapshot
    // (publish() never mutates a slot in place, only swaps the active pointer).
    const Config* p = active.load(std::memory_order_acquire);
    return p ? *p : defaults;
}

std::uint32_t generation()
{
    return gen.load(std::memory_order_acquire);
}

std::expected<void, Error> save(const Config& cfg)
{
    if (auto r = save_motion(cfg.motion); !r) {
        return r;
    }
    if (auto r = save_network(cfg.network); !r) {
        return r;
    }
    publish(cfg);
    return {};
}

std::expected<void, Error> request_debug_boot()
{
    nvs_handle_t h;
    if (nvs_open(NS_SYS, NVS_READWRITE, &h) != ESP_OK) {
        return std::unexpected(Error::NvsOpen);
    }
    const esp_err_t err = nvs_set_u8(h, KEY_DEBUG_BOOT, 1);
    const esp_err_t committed = (err == ESP_OK) ? nvs_commit(h) : err;
    nvs_close(h);
    if (committed != ESP_OK) {
        return std::unexpected(Error::NvsWrite);
    }
    return {};
}

bool take_debug_boot()
{
    nvs_handle_t h;
    if (nvs_open(NS_SYS, NVS_READWRITE, &h) != ESP_OK) {
        return false;
    }

    std::uint8_t flag = 0;
    const bool requested = (nvs_get_u8(h, KEY_DEBUG_BOOT, &flag) == ESP_OK) && flag != 0;
    if (requested) {
        nvs_erase_key(h, KEY_DEBUG_BOOT);
        nvs_commit(h);
    }

    nvs_close(h);
    return requested;
}

}  // namespace hvmrf01::config
