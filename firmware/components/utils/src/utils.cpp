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

std::size_t slugify(const char* in, char* out, std::size_t cap)
{
    if (out == nullptr || cap == 0) {
        return 0;
    }

    std::size_t n = 0;
    bool pending_hyphen = false;

    for (const char* p = in; p != nullptr && *p != '\0' && n + 1 < cap; ++p) {
        const unsigned char c = static_cast<unsigned char>(*p);

        if (c >= 'A' && c <= 'Z') {
            if (pending_hyphen && n > 0) out[n++] = '-';
            pending_hyphen = false;
            if (n + 1 < cap) out[n++] = static_cast<char>(c - 'A' + 'a');
            continue;
        }
        if ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9')) {
            if (pending_hyphen && n > 0) out[n++] = '-';
            pending_hyphen = false;
            if (n + 1 < cap) out[n++] = static_cast<char>(c);
            continue;
        }

        // Any other byte is a separator: remember that a hyphen is owed, but
        // only emit it once a following kept character actually arrives, so
        // leading, trailing, and repeated separators never reach the output.
        pending_hyphen = true;
    }

    // A separator's hyphen is written just before its following character, so a
    // truncation that stops between the two (the char didn't fit `cap`) can
    // leave a dangling trailing hyphen. Drop it.
    if (n > 0 && out[n - 1] == '-') {
        n--;
    }

    out[n] = '\0';
    return n;
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
