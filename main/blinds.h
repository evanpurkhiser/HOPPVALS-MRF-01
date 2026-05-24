#pragma once

// Zigbee channel mask. 0x07FFF800 covers channels 11..26 (the full 2.4 GHz
// Zigbee band). We try our primary channel first, then fall back.
#define ESP_ZIGBEE_PRIMARY_CHANNEL_MASK   (1U << 13)
#define ESP_ZIGBEE_SECONDARY_CHANNEL_MASK (0x07FFF800U)

#define BLINDS_ENDPOINT_ID 10

#define ESP_ZIGBEE_STORAGE_PARTITION_NAME "zb_storage"

#define BLINDS_MANUFACTURER_NAME "\x06" "EVANPK"
#define BLINDS_MODEL_IDENTIFIER  "\x06" "BLINDS"

#define BLINDS_ZR_CONFIG()                              \
    {                                                   \
        .device_type = EZB_NWK_DEVICE_TYPE_ROUTER,      \
        .install_code_policy = false,                   \
        .zczr_config = {                                \
            .max_children = 0,                          \
        },                                              \
    }

#if CONFIG_SOC_IEEE802154_SUPPORTED
#define BLINDS_PLATFORM_CONFIG()                                     \
    {                                                                \
        .storage_partition_name = ESP_ZIGBEE_STORAGE_PARTITION_NAME, \
        .radio_config = {                                            \
            .radio_mode = ESP_ZIGBEE_RADIO_MODE_NATIVE,              \
        },                                                           \
    }
#else
#error "This firmware targets an SoC with native 802.15.4 (ESP32-C6/H2)."
#endif

#define BLINDS_DEFAULT_CONFIG()                  \
    {                                            \
        .device_config = BLINDS_ZR_CONFIG(),     \
        .platform_config = BLINDS_PLATFORM_CONFIG(), \
    };
