#include "hv-mrf-01/zigbee.hpp"

#include "hv-mrf-01/config.hpp"
#include "hv-mrf-01/event_log.hpp"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <utility>

#include "esp_app_desc.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_zigbee.h"
#include "ezbee/af.h"
#include "ezbee/zha.h"
#include "ezbee/zcl/cluster/basic_desc.h"
#include "ezbee/zcl/cluster/custom.h"
#include "ezbee/zcl/cluster/identify.h"
#include "ezbee/zcl/cluster/identify_desc.h"
#include "ezbee/zcl/cluster/window_covering_desc.h"
#include "ezbee/zdo/zdo_dev_srv_disc.h"

namespace hvmrf01::zigbee {

// Define the base with an explicit, component-unique string rather than via
// ESP_EVENT_DEFINE_BASE(EVENTS) — that macro stringizes to "EVENTS", and a
// second component doing the same gets the identical literal. esp_event matches
// bases by pointer, and the linker pools identical string literals to one
// address, so two "EVENTS" bases collide and cross-deliver events. Keep these
// distinct (see position_report, which had the same name).
const esp_event_base_t EVENTS = "hvmrf01.zigbee";

namespace {

constexpr auto* TAG = "hv-mrf-01.zigbee";

// Zigbee character strings: byte 0 is the length, then the chars.
constexpr auto MANUFACTURER_NAME           = "\x0E"
                                             "Evan Purkhiser";
constexpr auto MODEL_IDENTIFIER            = "\x09"
                                             "HV-MRF-01";

constexpr std::uint8_t  EP_WINDOW_COVERING = 10;
constexpr std::uint32_t PRIMARY_CHANNEL    = 0x07FFF800U; // scan all of ch 11..26
constexpr std::uint32_t SECONDARY_CHANNELS = 0;           // primary already covers all
constexpr auto          STORAGE_PARTITION  = "zb_storage";

// Manufacturer-specific "device config" cluster
//
// A custom server cluster on the Window Covering endpoint that surfaces the
// handful of motion settings worth changing from the hub — speed and the two
// travel limits — plus a command to reboot into the WiFi debug console. The
// cluster ID lives in the custom range (>= 0x8000); the attributes and command
// are tagged manufacturer-specific so they don't collide with standard ZCL.
//
// Zigbee attributes carry no names over the air, so a matching ZHA quirk (keyed
// on the Basic-cluster ManufacturerName/ModelIdentifier above) gives these IDs
// their labels, units, and ranges in Home Assistant. Without the quirk they
// still work — they just appear as raw attribute IDs in the cluster inspector.
//
// 0x131B is Espressif's allocated manufacturer code; reused here since the quirk
// matches on the name/model strings, so the numeric code only needs to be
// stable and non-zero.
constexpr std::uint16_t MANUF_CODE      = EZB_ZCL_ESP_MANUF_CODE;
constexpr std::uint16_t CLUSTER_CONFIG  = 0xFC00;

constexpr std::uint16_t ATTR_COVER_SPEED  = 0x0000; // uint16, output-shaft RPM
constexpr std::uint16_t ATTR_SOFT_STOP_MM = 0x0001; // uint16, mm down limit (0 = off)
constexpr std::uint16_t ATTR_HARD_STOP_MM = 0x0002; // uint16, mm full travel

constexpr std::uint8_t  CMD_REBOOT_DEBUG  = 0x00;   // server-received, no payload
constexpr std::uint8_t  CMD_CALIBRATE     = 0x01;   // server-received, no payload; homes to the top

// Accepted speed range, mirrored by the quirk's number entity. mm limits accept
// any uint16, so motion's own clamping is the only bound there.
constexpr int COVER_RPM_MIN = 8;
constexpr int COVER_RPM_MAX = 300;

// Registered cover handlers. Set via register_cover_handlers(). Defaults to
// all-null; a supported command with no handler set returns Failure, while a
// command we don't implement returns UnsupportedCommand (see
// dispatch_cover_command).
CoverHandlers cover_handlers{};

// Backing store for the config cluster's attribute values. The SDK holds the
// pointers we hand it at registration, so this must outlive the stack — hence
// file scope. Seeded from the persisted config when the endpoint is built and
// kept in sync as writes land.
struct ConfigAttrValues
{
    std::uint16_t cover_speed;
    std::uint16_t soft_stop_mm;
    std::uint16_t hard_stop_mm;
};
ConfigAttrValues config_attrs{};

// RAII wrapper around the Zigbee stack lock. Acquire on construction, release
// on scope exit — no manual pairing, no leaks on early return.
class StackLock
{
  public:
    StackLock() noexcept { esp_zigbee_lock_acquire(portMAX_DELAY); }
    ~StackLock() noexcept { esp_zigbee_lock_release(); }
    StackLock(const StackLock&)            = delete;
    StackLock& operator=(const StackLock&) = delete;
};

// Forward-declare the compat alarm scheduler. Its header refuses direct
// inclusion in the new SDK, but the symbol is still exported from the lib.
using esp_zb_callback_t = void (*)(std::uint8_t param);
extern "C" void esp_zb_scheduler_alarm(esp_zb_callback_t cb, std::uint8_t param,
                                       std::uint32_t time_ms);

// Helper: post a typed event with no payload.
void post(Event ev) noexcept
{
    esp_event_post(EVENTS, std::to_underlying(ev), nullptr, 0, portMAX_DELAY);
}

template <typename T> void post(Event ev, const T& payload) noexcept
{
    esp_event_post(EVENTS, std::to_underlying(ev), &payload, sizeof(payload), portMAX_DELAY);
}

// SDK callbacks (must be C-callable; use extern "C" linkage for clarity).

extern "C" void alarm_restart_commissioning(std::uint8_t mode)
{
    StackLock lock;
    static_cast<void>(ezb_bdb_start_top_level_commissioning(mode));
}

const char* leave_type_name(ezb_zdo_leave_type_t type) noexcept
{
    switch (type) {
    case EZB_ZDO_LEAVE_TYPE_RESET:  return "reset (no rejoin)";
    case EZB_ZDO_LEAVE_TYPE_REJOIN: return "rejoin";
    default:                        return "?";
    }
}

// Log the current network identity (only meaningful once joined): PAN,
// extended PAN, channel, and our short address.
void log_network_info(const char* context) noexcept
{
    ezb_extpanid_t extpanid;
    ezb_nwk_get_extended_panid(&extpanid);
    ESP_LOGI(TAG, "%s: PAN 0x%04hx (ext 0x%016llx), ch %d, addr 0x%04hx", context,
             ezb_nwk_get_panid(), extpanid.u64, ezb_nwk_get_current_channel(),
             ezb_nwk_get_short_address());
}

void device_announce_done(const ezb_zdo_device_annce_req_result_t* result, void* user_ctx)
{
    const auto* context = static_cast<const char*>(user_ctx);
    if (result != nullptr && result->error == EZB_ERR_NONE) {
        ESP_LOGI(TAG, "%s: device announce sent", context);
        post(Event::JoinedNetwork);
        return;
    }

    ESP_LOGW(TAG, "%s: device announce failed (0x%04x); keeping recovery armed", context,
             result ? result->error : EZB_ERR_FAIL);
}

void announce_joined_network(const char* context) noexcept
{
    if (!ezb_bdb_dev_joined()) {
        ESP_LOGW(TAG, "%s: BDB says device is not joined; keeping recovery armed", context);
        return;
    }

    log_network_info(context);

    const ezb_zdo_device_annce_req_t req{
        .cb       = &device_announce_done,
        .user_ctx = const_cast<char*>(context),
    };
    const ezb_err_t err = ezb_zdo_device_annce_req(&req);
    if (err != EZB_ERR_NONE) {
        ESP_LOGW(TAG, "%s: failed to request device announce (0x%04x); keeping recovery armed",
                 context, err);
    }
}

extern "C" bool signal_handler(const ezb_app_signal_t* app_signal)
{
    using enum Event;
    const auto signal_type = ezb_app_signal_get_type(app_signal);

    switch (signal_type) {
    case EZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Zigbee stack initialized");
        ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_INITIALIZATION);
        break;

    case EZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case EZB_BDB_SIGNAL_DEVICE_REBOOT: {
        const auto status =
            *static_cast<const ezb_bdb_comm_status_t*>(ezb_app_signal_get_params(app_signal));
        if (status == EZB_BDB_STATUS_SUCCESS) {
            const bool factory_new = ezb_bdb_is_factory_new();
            ESP_LOGI(TAG, "Device started%s", factory_new ? " (factory new)" : "");
            if (factory_new) {
                ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_NETWORK_STEERING);
            } else {
                announce_joined_network("Rejoining existing network");
            }
        } else {
            ESP_LOGW(TAG, "%s failed (0x%02x), retrying...", ezb_app_signal_to_string(signal_type),
                     status);
            esp_zb_scheduler_alarm(alarm_restart_commissioning, EZB_BDB_MODE_INITIALIZATION, 1000);
        }
    } break;

    case EZB_BDB_SIGNAL_STEERING: {
        const auto status =
            *static_cast<const ezb_bdb_comm_status_t*>(ezb_app_signal_get_params(app_signal));
        if (status == EZB_BDB_STATUS_SUCCESS) {
            announce_joined_network("Joined network");
        } else {
            ESP_LOGW(TAG, "Network steering failed (0x%02x), retrying in 1s...", status);
            esp_zb_scheduler_alarm(alarm_restart_commissioning, EZB_BDB_MODE_NETWORK_STEERING,
                                   1000);
        }
    } break;

    case EZB_ZDO_SIGNAL_LEAVE: {
        const auto* params =
            static_cast<const ezb_zdo_signal_leave_params_t*>(ezb_app_signal_get_params(app_signal));
        const auto leave_type = params ? params->leave_type : EZB_ZDO_LEAVE_TYPE_RESET;
        ESP_LOGW(TAG, "Left network (leave type: %s), returning to commissioning",
                 leave_type_name(leave_type));
        post(LeftNetwork);
        ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_NETWORK_STEERING);
    } break;

    case EZB_NWK_SIGNAL_NETWORK_STATUS: {
        const auto* params = static_cast<const ezb_nwk_signal_network_status_params_t*>(
            ezb_app_signal_get_params(app_signal));
        if (params != nullptr) {
            ESP_LOGW(TAG, "Network failure 0x%02x at addr 0x%04hx (unknown cmd 0x%02x)",
                     params->status, params->network_addr, params->unknown_command_id);
        }
    } break;

    case EZB_ZDO_SIGNAL_DEVICE_UNAVAILABLE: {
        const auto* params = static_cast<const ezb_zdo_signal_device_unavailable_params_t*>(
            ezb_app_signal_get_params(app_signal));
        if (params != nullptr) {
            ESP_LOGW(TAG, "Delivery failed: addr 0x%04hx (ieee 0x%016llx) unreachable",
                     params->short_addr, params->device_addr.u64);
        }
    } break;

    case EZB_NWK_SIGNAL_PERMIT_JOIN_STATUS: {
        const auto* params = static_cast<const ezb_nwk_signal_permit_join_status_params_t*>(
            ezb_app_signal_get_params(app_signal));
        if (params != nullptr) {
            ESP_LOGI(TAG, "Permit-join %s (%u s)", params->duration ? "open" : "closed",
                     params->duration);
        }
    } break;

    default:
        ESP_LOGI(TAG, "Zigbee signal: %s (0x%04x)", ezb_app_signal_to_string(signal_type),
                 signal_type);
        break;
    }
    return true;
}

// Dispatch a cover command through the application-registered handlers and
// return its status. If the relevant handler is unset, return Failure so the
// hub sees a real "no" rather than silent acceptance.
CommandStatus dispatch_cover_command(std::uint8_t cmd_id,
                                     const ezb_zcl_window_covering_movement_message_t* msg) noexcept
{
    switch (cmd_id) {
    case EZB_ZCL_CMD_WINDOW_COVERING_UP_OPEN_ID:
        return cover_handlers.open ? cover_handlers.open() : CommandStatus::Failure;
    case EZB_ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE_ID:
        return cover_handlers.close ? cover_handlers.close() : CommandStatus::Failure;
    case EZB_ZCL_CMD_WINDOW_COVERING_STOP_ID:
        return cover_handlers.stop ? cover_handlers.stop() : CommandStatus::Failure;
    case EZB_ZCL_CMD_WINDOW_COVERING_GO_TO_LIFT_PERCENTAGE_ID:
        return cover_handlers.go_to_percent
                   ? cover_handlers.go_to_percent(msg->in.payload.lift_percentage)
                   : CommandStatus::Failure;
    default:
        return CommandStatus::UnsupportedCommand;
    }
}

void handle_window_covering(ezb_zcl_window_covering_movement_message_t* msg) noexcept
{
    const std::uint8_t cmd_id = msg->in.header ? msg->in.header->cmd_id : 0xFF;
    const std::uint8_t pct = cmd_id == EZB_ZCL_CMD_WINDOW_COVERING_GO_TO_LIFT_PERCENTAGE_ID
                                 ? msg->in.payload.lift_percentage
                                 : 0;
    event_log::zigbee_cover_received(cmd_id, pct);

    // Map known commands to readable names just for the log line.
    const char* name = "?";
    switch (cmd_id) {
    case EZB_ZCL_CMD_WINDOW_COVERING_UP_OPEN_ID:               name = "UpOpen"; break;
    case EZB_ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE_ID:            name = "DownClose"; break;
    case EZB_ZCL_CMD_WINDOW_COVERING_STOP_ID:                  name = "Stop"; break;
    case EZB_ZCL_CMD_WINDOW_COVERING_GO_TO_LIFT_PERCENTAGE_ID: name = "GoToLiftPercentage"; break;
    }

    const auto status = dispatch_cover_command(cmd_id, msg);
    event_log::zigbee_cover_status(cmd_id, static_cast<int>(status), pct);
    ESP_LOGI(TAG, "Cover: %s%s%d%s → status 0x%02x",
             name,
             cmd_id == EZB_ZCL_CMD_WINDOW_COVERING_GO_TO_LIFT_PERCENTAGE_ID ? "(" : "",
             pct,
             cmd_id == EZB_ZCL_CMD_WINDOW_COVERING_GO_TO_LIFT_PERCENTAGE_ID ? "%)" : "",
             static_cast<unsigned>(status));

    msg->out.result = static_cast<ezb_zcl_status_t>(status);
}

void handle_set_attr(ezb_zcl_set_attr_value_message_t* msg) noexcept
{
    using enum Event;
    if (msg == nullptr) {
        return;
    }

    switch (msg->info.cluster_id) {
    case EZB_ZCL_CLUSTER_ID_IDENTIFY:
        // HA's "Identify" device action writes the IdentifyTime attribute.
        // The SDK auto-decrements it once per second; any nonzero value here
        // means "be identifiable". Map to a Blink effect; the LED component
        // owns the actual blink rhythm and duration.
        if (msg->in.attribute.id == EZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID) {
            const auto t = *static_cast<std::uint16_t*>(msg->in.attribute.data.value);
            ESP_LOGI(TAG, "Identify time = %u sec", t);
            const auto effect = t > 0 ? IdentifyEffect::Blink : IdentifyEffect::StopEffect;
            post(Identify, static_cast<std::uint8_t>(effect));
        }
        break;

    default:
        break;
    }
}

void handle_identify_effect(ezb_zcl_identify_effect_message_t* msg) noexcept
{
    if (msg == nullptr) {
        return;
    }
    const auto effect_id = msg->in.effect_id;
    ESP_LOGI(TAG, "Identify effect 0x%02x (variant 0x%02x)", effect_id, msg->in.effect_variant);
    post(Event::Identify, effect_id);
    msg->out.result = EZB_ZCL_STATUS_SUCCESS;
}

// Config cluster callbacks
//
// The stack invokes these for the manufacturer-specific config cluster, keyed
// by cluster_id at registration. check_value gates a write before it's stored;
// write_attr fires after a stored write so we can mirror it into persistent
// config; process_cmd handles the (attribute-less) command requests.

extern "C" ezb_zcl_status_t config_cluster_check_value(std::uint16_t attr_id, std::uint8_t,
                                                       void* value) noexcept
{
    const auto v = *static_cast<std::uint16_t*>(value);
    if (attr_id == ATTR_COVER_SPEED && (v < COVER_RPM_MIN || v > COVER_RPM_MAX)) {
        return EZB_ZCL_STATUS_INVALID_VALUE;
    }
    return EZB_ZCL_STATUS_SUCCESS;
}

// Persist a validated config-attribute write. Runs in the Zigbee task from
// inside the stack's write path, so it must not take the stack lock; config
// saves are plain NVS writes that don't touch the stack. The motion task picks
// the change up on its next tick via config::generation(), so speed and limit
// edits apply live without a reboot.
extern "C" void config_cluster_write_attr(std::uint8_t, std::uint16_t attr_id, void* new_value,
                                          std::uint16_t) noexcept
{
    const auto v   = *static_cast<std::uint16_t*>(new_value);
    auto       cfg = config::get();

    switch (attr_id) {
    case ATTR_COVER_SPEED:
        cfg.motion.cover_rpm     = v;
        config_attrs.cover_speed = v;
        break;
    case ATTR_SOFT_STOP_MM:
        cfg.motion.soft_stop_mm   = static_cast<float>(v);
        config_attrs.soft_stop_mm = v;
        break;
    case ATTR_HARD_STOP_MM:
        cfg.motion.hard_stop_mm   = static_cast<float>(v);
        config_attrs.hard_stop_mm = v;
        break;
    default:
        return;
    }

    if (auto r = config::save(cfg); !r) {
        ESP_LOGW(TAG, "config attr 0x%04x: save failed (err %d)", attr_id,
                 static_cast<int>(r.error()));
        return;
    }
    ESP_LOGI(TAG, "config attr 0x%04x = %u (persisted)", attr_id, v);
}

extern "C" ezb_zcl_status_t config_cluster_process_cmd(const ezb_zcl_cmd_hdr_t* hdr,
                                                       const std::uint8_t*, std::uint16_t) noexcept
{
    if (hdr == nullptr) {
        return EZB_ZCL_STATUS_UNSUP_CMD;
    }

    switch (hdr->cmd_id) {
    case CMD_REBOOT_DEBUG:
        event_log::zigbee_config_command(hdr->cmd_id, CLUSTER_CONFIG);
        ESP_LOGI(TAG, "Config cmd: reboot into debug mode");
        post(Event::EnterDebug);
        return EZB_ZCL_STATUS_SUCCESS;
    case CMD_CALIBRATE:
        event_log::zigbee_config_command(hdr->cmd_id, CLUSTER_CONFIG);
        ESP_LOGI(TAG, "Config cmd: calibrate (home to top)");
        post(Event::Calibrate);
        return EZB_ZCL_STATUS_SUCCESS;
    default:
        event_log::zigbee_config_command(hdr->cmd_id, CLUSTER_CONFIG, true);
        ESP_LOGW(TAG, "Config cmd: unsupported 0x%02x", hdr->cmd_id);
        return EZB_ZCL_STATUS_UNSUP_CMD;
    }
}

// A command sent with a manufacturer code arrives here rather than at the custom
// cluster's process_cmd_cb — the SDK routes manufacturer-specific commands to
// this core callback, and process_cmd_cb only sees non-manufacturer commands. So
// forward our config cluster's manufacturer commands to the same handler.
void handle_manuf_spec_cmd(ezb_zcl_manuf_spec_cmd_message_t* msg) noexcept
{
    if (msg == nullptr || msg->in.header == nullptr) {
        return;
    }
    const auto* hdr = msg->in.header;

    if (hdr->dst_ep != EP_WINDOW_COVERING || hdr->cluster_id != CLUSTER_CONFIG ||
        hdr->manuf_code != MANUF_CODE) {
        return;
    }

    msg->out.result = config_cluster_process_cmd(hdr, msg->in.payload, msg->in.payload_size);
}

extern "C" void zcl_action_handler(ezb_zcl_core_action_callback_id_t cb_id, void* msg)
{
    switch (cb_id) {
    case EZB_ZCL_CORE_SET_ATTR_VALUE_CB_ID:
        handle_set_attr(static_cast<ezb_zcl_set_attr_value_message_t*>(msg));
        break;
    case EZB_ZCL_CORE_MANUF_SPEC_CMD_CB_ID:
        handle_manuf_spec_cmd(static_cast<ezb_zcl_manuf_spec_cmd_message_t*>(msg));
        break;
    case EZB_ZCL_CORE_WINDOW_COVERING_MOVEMENT_CB_ID:
        handle_window_covering(static_cast<ezb_zcl_window_covering_movement_message_t*>(msg));
        break;
    case EZB_ZCL_CORE_IDENTIFY_EFFECT_CB_ID:
        handle_identify_effect(static_cast<ezb_zcl_identify_effect_message_t*>(msg));
        break;
    case EZB_ZCL_CORE_DEFAULT_RSP_CB_ID: {
        const auto* rsp = static_cast<ezb_zcl_cmd_default_rsp_message_t*>(msg);
        ESP_LOGD(TAG, "Default ZCL response: status=0x%02x", rsp->in.status_code);
    } break;
    default:
        ESP_LOGD(TAG, "Unhandled ZCL action 0x%04lx", static_cast<unsigned long>(cb_id));
        break;
    }
}

// Endpoint construction

// Build the Basic cluster's SWBuildID payload from the firmware version that
// ESP-IDF bakes in at build time (PROJECT_VER, derived from `git describe` —
// i.e. the short commit SHA, plus a "-dirty" suffix for uncommitted builds).
// ZHA/Z2M surface this as the device's firmware version in Home Assistant, so
// you can see which git revision a flashed unit is running.
//
// Zigbee character strings are length-prefixed: byte 0 holds the length. The
// SDK keeps the pointer we hand it, so the buffer must outlive registration —
// hence the function-local static.
const char* sw_build_id()
{
    static char buf[1 + sizeof(esp_app_desc_t::version)] = {};
    if (buf[0] == '\0') {
        const char* version = esp_app_get_description()->version;
        const auto  len      = std::min(std::strlen(version), sizeof(buf) - 1);
        buf[0]               = static_cast<char>(len);
        std::memcpy(&buf[1], version, len);
    }
    return buf;
}

// The configured device location as a length-prefixed ZCL character string, or
// nullptr when none is set — stamped as the Basic cluster LocationDescription
// so the unit's "Bedroom Right" label travels with it on the Zigbee side. ZCL
// caps LocationDescription at 16 chars, so a longer value is truncated. Built
// once — like sw_build_id(), the SDK keeps our pointer, so the buffer is a
// function-local static that outlives registration.
const char* location_desc()
{
    static char buf[1 + 16] = {};
    if (buf[0] == '\0') {
        const config::Config cfg      = config::get();
        const char*          location = cfg.device.location;
        const auto           len      = std::min(std::strlen(location), sizeof(buf) - 1);
        if (len == 0) {
            return nullptr;
        }
        buf[0] = static_cast<char>(len);
        std::memcpy(&buf[1], location, len);
    }
    return buf;
}

void stamp_identity(ezb_af_ep_desc_t ep)
{
    auto basic =
        ezb_af_endpoint_get_cluster_desc(ep, EZB_ZCL_CLUSTER_ID_BASIC, EZB_ZCL_CLUSTER_SERVER);
    if (basic == nullptr) {
        return;
    }
    ezb_zcl_basic_cluster_desc_add_attr(basic, EZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID,
                                        const_cast<char*>(MANUFACTURER_NAME));
    ezb_zcl_basic_cluster_desc_add_attr(basic, EZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,
                                        const_cast<char*>(MODEL_IDENTIFIER));
    ezb_zcl_basic_cluster_desc_add_attr(basic, EZB_ZCL_ATTR_BASIC_SW_BUILD_ID_ID, sw_build_id());

    if (const char* loc = location_desc(); loc != nullptr) {
        ezb_zcl_basic_cluster_desc_add_attr(basic, EZB_ZCL_ATTR_BASIC_LOCATION_DESCRIPTION_ID, loc);
    }
}

// The SDK's Window Covering cluster carries only its mandatory attributes
// (type, config status, mode). CurrentPositionLiftPercentage is optional, so add
// it explicitly — without it report_position()'s set_attr_value fails (status
// 0x01, attribute not found), the hub shows the position as unknown, and a cover
// with no position attribute won't accept a go-to-percentage.
void add_position_attribute(ezb_af_ep_desc_t ep)
{
    auto wc = ezb_af_endpoint_get_cluster_desc(ep, EZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
                                               EZB_ZCL_CLUSTER_SERVER);
    if (wc == nullptr) {
        ESP_LOGE(TAG, "window covering cluster desc missing; can't add position attr");
        return;
    }
    static std::uint8_t initial_pct = 0;
    ezb_zcl_window_covering_cluster_desc_add_attr(
        wc, EZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID, &initial_pct);
}

// Attach the manufacturer-specific config cluster to an existing endpoint:
// seed the attribute values from the persisted config, build the cluster
// descriptor, and register the handlers that gate/persist writes and process
// the reboot command. Best-effort — a failure here just means the config
// cluster is absent; the standard Window Covering control still works.
void add_config_cluster(ezb_af_ep_desc_t ep)
{
    const auto m = config::get().motion;
    config_attrs = {
        .cover_speed  = static_cast<std::uint16_t>(m.cover_rpm),
        .soft_stop_mm = static_cast<std::uint16_t>(m.soft_stop_mm),
        .hard_stop_mm = static_cast<std::uint16_t>(m.hard_stop_mm),
    };

    const ezb_zcl_custom_cluster_config_t cfg{
        .cluster_id  = CLUSTER_CONFIG,
        .init_func   = nullptr,
        .deinit_func = nullptr,
    };
    auto cluster = ezb_zcl_custom_create_cluster_desc(&cfg, EZB_ZCL_CLUSTER_SERVER);
    if (cluster == nullptr) {
        ESP_LOGE(TAG, "config cluster desc alloc failed");
        return;
    }

    const auto add = [&](std::uint16_t id, void* val) {
        ezb_zcl_custom_cluster_desc_add_manuf_attr(cluster, id, EZB_ZCL_ATTR_TYPE_UINT16,
                                                   EZB_ZCL_ATTR_ACCESS_READ_WRITE, MANUF_CODE, val);
    };
    add(ATTR_COVER_SPEED, &config_attrs.cover_speed);
    add(ATTR_SOFT_STOP_MM, &config_attrs.soft_stop_mm);
    add(ATTR_HARD_STOP_MM, &config_attrs.hard_stop_mm);

    ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(ep, cluster));

    static const ezb_zcl_custom_cluster_handlers_t handlers{
        .cluster_id     = CLUSTER_CONFIG,
        .cluster_role   = EZB_ZCL_CLUSTER_SERVER,
        .check_value_cb = config_cluster_check_value,
        .write_attr_cb  = config_cluster_write_attr,
        .cmd_disc_cb    = nullptr,
        .process_cmd_cb = config_cluster_process_cmd,
    };
    ESP_ERROR_CHECK(ezb_zcl_custom_cluster_handlers_register(&handlers));
}

ezb_af_ep_desc_t build_window_covering_endpoint()
{
    auto cfg = ezb_zha_window_covering_config_t EZB_ZHA_WINDOW_COVERING_CONFIG();
    cfg.window_covering_cfg.window_covering_type =
        EZB_ZCL_WINDOW_COVERING_WINDOW_COVERING_TYPE_ROLLERSHADE;
    cfg.window_covering_cfg.config_status = EZB_ZCL_WINDOW_COVERING_CONFIG_STATUS_OPERATIONAL |
                                            EZB_ZCL_WINDOW_COVERING_CONFIG_STATUS_ONLINE |
                                            EZB_ZCL_WINDOW_COVERING_CONFIG_STATUS_LIFT_CLOSED_LOOP |
                                            EZB_ZCL_WINDOW_COVERING_CONFIG_STATUS_LIFT_ENCODER_CONTROLLED;
    cfg.window_covering_cfg.mode          = 0;

    auto ep = ezb_zha_create_window_covering(EP_WINDOW_COVERING, &cfg);
    stamp_identity(ep);
    add_position_attribute(ep);
    add_config_cluster(ep);
    return ep;
}

esp_err_t register_endpoints()
{
    // Tag the node descriptor with the manufacturer code so the manufacturer-
    // specific attributes and command read back consistently on the hub.
    ezb_af_node_desc_set_manuf_code(MANUF_CODE);

    auto dev = ezb_af_create_device_desc();
    ESP_RETURN_ON_FALSE(dev != nullptr, ESP_ERR_NO_MEM, TAG, "device desc failed");

    auto wc = build_window_covering_endpoint();
    ESP_RETURN_ON_FALSE(wc != nullptr, ESP_ERR_NO_MEM, TAG, "window covering ep failed");
    ESP_ERROR_CHECK(ezb_af_device_add_endpoint_desc(dev, wc));

    ESP_ERROR_CHECK(ezb_af_device_desc_register(dev));
    ezb_zcl_core_action_handler_register(zcl_action_handler);
    return ESP_OK;
}

esp_err_t setup_commissioning()
{
    ezb_aps_secur_enable_distributed_security(false);
    ESP_ERROR_CHECK(ezb_bdb_set_primary_channel_set(PRIMARY_CHANNEL));
    ESP_ERROR_CHECK(ezb_bdb_set_secondary_channel_set(SECONDARY_CHANNELS));
    ESP_ERROR_CHECK(ezb_app_signal_add_handler(signal_handler));
    return ESP_OK;
}

// Zigbee mainloop task. Owns the SDK; never returns under normal operation.
void task_main(void*)
{
    const esp_zigbee_config_t config{
        .device_config{
            .device_type         = EZB_NWK_DEVICE_TYPE_ROUTER,
            .install_code_policy = false,
            .zczr_config         = {.max_children = 0},
        },
#if CONFIG_SOC_IEEE802154_SUPPORTED
        .platform_config{
            .storage_partition_name = STORAGE_PARTITION,
            .radio_config           = {.radio_mode = ESP_ZIGBEE_RADIO_MODE_NATIVE},
        },
#endif
    };

    ESP_ERROR_CHECK(esp_zigbee_init(&config));
    ESP_ERROR_CHECK(setup_commissioning());
    ESP_ERROR_CHECK(register_endpoints());
    ESP_ERROR_CHECK(esp_zigbee_start(false));

    esp_zigbee_launch_mainloop();

    esp_zigbee_deinit();
    vTaskDelete(nullptr);
}

} // namespace

void start()
{
    // The default NVS partition is brought up by config::init() in app_main;
    // here we only initialize the Zigbee stack's dedicated storage partition.
    ESP_ERROR_CHECK(nvs_flash_init_partition(STORAGE_PARTITION));
    xTaskCreate(task_main, "zb_main", 4096, nullptr, 5, nullptr);
}

void register_cover_handlers(const CoverHandlers& handlers)
{
    cover_handlers = handlers;
}

void report_position(std::uint8_t pct)
{
    if (pct > 100) {
        pct = 100;
    }
    StackLock lock;
    const auto status = ezb_zcl_set_attr_value(
        EP_WINDOW_COVERING, EZB_ZCL_CLUSTER_ID_WINDOW_COVERING, EZB_ZCL_CLUSTER_SERVER,
        EZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID,
        EZB_ZCL_STD_MANUF_CODE, &pct, /*check_access=*/false);
    if (status != EZB_ZCL_STATUS_SUCCESS) {
        event_log::zigbee_cover_status(0xFE, status, pct);
        ESP_LOGW(TAG, "report_position(%u): set_attr status 0x%02x", pct, status);
    }
}

} // namespace hvmrf01::zigbee
