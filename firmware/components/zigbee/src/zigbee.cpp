#include "hv-mrf-01/zigbee.hpp"

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
#include "ezbee/zha.h"
#include "ezbee/zcl/cluster/basic_desc.h"
#include "ezbee/zcl/cluster/identify.h"
#include "ezbee/zcl/cluster/identify_desc.h"
#include "ezbee/zcl/cluster/on_off_desc.h"
#include "ezbee/zcl/cluster/window_covering_desc.h"

namespace hvmrf01::zigbee {

ESP_EVENT_DEFINE_BASE(EVENTS);

namespace {

constexpr auto* TAG = "hv-mrf-01.zigbee";

// Zigbee character strings: byte 0 is the length, then the chars.
constexpr auto MANUFACTURER_NAME           = "\x0E"
                                             "Evan Purkhiser";
constexpr auto MODEL_IDENTIFIER            = "\x09"
                                             "HV-MRF-01";

constexpr std::uint8_t  EP_WINDOW_COVERING = 10;
constexpr std::uint8_t  EP_DEBUG_SWITCH    = 11;
constexpr std::uint32_t PRIMARY_CHANNEL    = 1U << 13;    // ch 13 first
constexpr std::uint32_t SECONDARY_CHANNELS = 0x07FFF800U; // ch 11..26
constexpr auto          STORAGE_PARTITION  = "zb_storage";

// Registered cover handlers. Set via register_cover_handlers(). Defaults to
// all-null; the dispatch path returns CommandStatus::Failure for any unset
// handler.
CoverHandlers cover_handlers{};

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
                ESP_LOGI(TAG, "Rejoining existing network (PAN 0x%04hx, ch %d)",
                         ezb_nwk_get_panid(), ezb_nwk_get_current_channel());
                post(JoinedNetwork);
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
            ezb_extpanid_t extpanid;
            ezb_nwk_get_extended_panid(&extpanid);
            ESP_LOGI(TAG, "Joined network: PAN 0x%04hx (ext 0x%llx), ch %d, addr 0x%04hx",
                     ezb_nwk_get_panid(), extpanid.u64, ezb_nwk_get_current_channel(),
                     ezb_nwk_get_short_address());
            post(JoinedNetwork);
        } else {
            ESP_LOGW(TAG, "Network steering failed (0x%02x), retrying in 1s...", status);
            esp_zb_scheduler_alarm(alarm_restart_commissioning, EZB_BDB_MODE_NETWORK_STEERING,
                                   1000);
        }
    } break;

    case EZB_ZDO_SIGNAL_LEAVE:
        ESP_LOGW(TAG, "Left network, returning to commissioning");
        post(LeftNetwork);
        ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_NETWORK_STEERING);
        break;

    default:
        ESP_LOGD(TAG, "Zigbee signal: %s (0x%02x)", ezb_app_signal_to_string(signal_type),
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
        return CommandStatus::Failure;
    }
}

void handle_window_covering(ezb_zcl_window_covering_movement_message_t* msg) noexcept
{
    const std::uint8_t cmd_id = msg->in.header ? msg->in.header->cmd_id : 0xFF;

    // Map known commands to readable names just for the log line.
    const char* name = "?";
    switch (cmd_id) {
    case EZB_ZCL_CMD_WINDOW_COVERING_UP_OPEN_ID:               name = "UpOpen"; break;
    case EZB_ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE_ID:            name = "DownClose"; break;
    case EZB_ZCL_CMD_WINDOW_COVERING_STOP_ID:                  name = "Stop"; break;
    case EZB_ZCL_CMD_WINDOW_COVERING_GO_TO_LIFT_PERCENTAGE_ID: name = "GoToLiftPercentage"; break;
    }

    const auto status = dispatch_cover_command(cmd_id, msg);
    ESP_LOGI(TAG, "Cover: %s%s%d%s → status 0x%02x",
             name,
             cmd_id == EZB_ZCL_CMD_WINDOW_COVERING_GO_TO_LIFT_PERCENTAGE_ID ? "(" : "",
             cmd_id == EZB_ZCL_CMD_WINDOW_COVERING_GO_TO_LIFT_PERCENTAGE_ID
                 ? msg->in.payload.lift_percentage
                 : 0,
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
    case EZB_ZCL_CLUSTER_ID_ON_OFF:
        // The "Debug Mode" switch endpoint. Turning it on is the untethered
        // entry point into WiFi debug mode: post the event and let app_main
        // arm the one-shot flag and reboot (keeps Zigbee free of that policy).
        // Gate on the specific endpoint and validate the payload so an On/Off
        // write to some other future endpoint can't reboot us out of Zigbee.
        if (msg->info.dst_ep == EP_DEBUG_SWITCH &&
            msg->in.attribute.id == EZB_ZCL_ATTR_ON_OFF_ON_OFF_ID &&
            msg->in.attribute.data.value != nullptr &&
            msg->in.attribute.data.size >= 1) {
            const bool on = *static_cast<bool*>(msg->in.attribute.data.value);
            ESP_LOGI(TAG, "Debug Mode switch = %s", on ? "on" : "off");
            if (on) {
                post(EnterDebug);
            }
        }
        break;

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

extern "C" void zcl_action_handler(ezb_zcl_core_action_callback_id_t cb_id, void* msg)
{
    switch (cb_id) {
    case EZB_ZCL_CORE_SET_ATTR_VALUE_CB_ID:
        handle_set_attr(static_cast<ezb_zcl_set_attr_value_message_t*>(msg));
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
}

ezb_af_ep_desc_t build_window_covering_endpoint()
{
    auto cfg = ezb_zha_window_covering_config_t EZB_ZHA_WINDOW_COVERING_CONFIG();
    cfg.window_covering_cfg.window_covering_type =
        EZB_ZCL_WINDOW_COVERING_WINDOW_COVERING_TYPE_ROLLERSHADE;
    cfg.window_covering_cfg.config_status = EZB_ZCL_WINDOW_COVERING_CONFIG_STATUS_OPERATIONAL |
                                            EZB_ZCL_WINDOW_COVERING_CONFIG_STATUS_ONLINE;
    cfg.window_covering_cfg.mode          = 0;

    auto ep = ezb_zha_create_window_covering(EP_WINDOW_COVERING, &cfg);
    stamp_identity(ep);
    return ep;
}

// A second endpoint exposing a single On/Off server cluster. ZHA surfaces it as
// a switch entity ("Debug Mode"); turning it on is the untethered trigger to
// reboot into WiFi debug mode. See handle_set_attr's ON_OFF case.
ezb_af_ep_desc_t build_debug_switch_endpoint()
{
    auto cfg = ezb_zha_mains_power_outlet_config_t EZB_ZHA_MAINS_POWER_OUTLET_CONFIG();
    return ezb_zha_create_mains_power_outlet(EP_DEBUG_SWITCH, &cfg);
}

esp_err_t register_endpoints()
{
    auto dev = ezb_af_create_device_desc();
    ESP_RETURN_ON_FALSE(dev != nullptr, ESP_ERR_NO_MEM, TAG, "device desc failed");

    auto wc = build_window_covering_endpoint();
    ESP_RETURN_ON_FALSE(wc != nullptr, ESP_ERR_NO_MEM, TAG, "window covering ep failed");
    ESP_ERROR_CHECK(ezb_af_device_add_endpoint_desc(dev, wc));

    auto dbg = build_debug_switch_endpoint();
    ESP_RETURN_ON_FALSE(dbg != nullptr, ESP_ERR_NO_MEM, TAG, "debug switch ep failed");
    ESP_ERROR_CHECK(ezb_af_device_add_endpoint_desc(dev, dbg));

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
        ESP_LOGW(TAG, "report_position(%u): set_attr status 0x%02x", pct, status);
    }
}

} // namespace hvmrf01::zigbee
