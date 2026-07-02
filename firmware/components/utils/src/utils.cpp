#include "hv-mrf-01/utils.hpp"

#include <cstdint>
#include <cstdio>

#include "esp_log.h"
#include "esp_mac.h"

namespace hvmrf01::utils {

namespace {

constexpr auto* TAG = "hv-mrf-01.utils";

}  // namespace

const char* device_id()
{
    static char id[9] = {};
    if (id[0] != '\0') {
        return id;
    }

    std::uint8_t mac[8] = {};
    esp_read_mac(mac, ESP_MAC_IEEE802154);

    // ZHA prints the EUI-64 MSB-first; esp_read_mac fills the buffer in the
    // reverse order, so the low 4 bytes (the last 8 hex chars ZHA shows) are
    // mac[3..0]. If the boot log proves the orders are flipped, swap to
    // mac[4..7] here.
    std::snprintf(id, sizeof(id), "%02x%02x%02x%02x", mac[3], mac[2], mac[1], mac[0]);
    return id;
}

void log_identity()
{
    std::uint8_t mac[8] = {};
    esp_read_mac(mac, ESP_MAC_IEEE802154);

    ESP_LOGI(TAG, "EUI-64 raw     %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], mac[6], mac[7]);
    ESP_LOGI(TAG, "EUI-64 ZHA-fmt %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
             mac[7], mac[6], mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
    ESP_LOGI(TAG, "device_id      %s", device_id());
}

}  // namespace hvmrf01::utils
