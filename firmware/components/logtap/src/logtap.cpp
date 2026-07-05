#include "hv-mrf-01/logtap.hpp"

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"

namespace hvmrf01::logtap {

namespace {

constexpr std::size_t RING_SZ = 8192;
constexpr auto*       NS       = "logtap";
constexpr auto*       KEY      = "log";

char           ring[RING_SZ];
std::size_t    head    = 0;      // next write index
bool           wrapped = false;  // ring has cycled at least once
portMUX_TYPE   mux     = portMUX_INITIALIZER_UNLOCKED;
vprintf_like_t orig    = nullptr;

void append(const char* data, std::size_t n)
{
    taskENTER_CRITICAL(&mux);
    for (std::size_t i = 0; i < n; ++i) {
        ring[head++] = data[i];
        if (head == RING_SZ) {
            head    = 0;
            wrapped = true;
        }
    }
    taskEXIT_CRITICAL(&mux);
}

// Copy the ring in chronological order into out (up to cap bytes).
std::size_t linearize(char* out, std::size_t cap)
{
    taskENTER_CRITICAL(&mux);
    std::size_t copied = 0;
    if (!wrapped) {
        copied = std::min(head, cap);
        std::memcpy(out, ring, copied);
    } else {
        const std::size_t tail = RING_SZ - head;  // oldest bytes: [head, end)
        const std::size_t c1   = std::min(tail, cap);
        std::memcpy(out, ring + head, c1);
        const std::size_t c2 = std::min(head, cap - c1);  // then [0, head)
        std::memcpy(out + c1, ring, c2);
        copied = c1 + c2;
    }
    taskEXIT_CRITICAL(&mux);
    return copied;
}

int hook(const char* fmt, va_list ap)
{
    char    line[256];
    va_list ap2;
    va_copy(ap2, ap);
    const int n = vsnprintf(line, sizeof(line), fmt, ap2);
    va_end(ap2);
    if (n > 0) {
        append(line, std::min(static_cast<std::size_t>(n), sizeof(line) - 1));
    }
    return orig != nullptr ? orig(fmt, ap) : vprintf(fmt, ap);
}

}  // namespace

void install()
{
    orig = esp_log_set_vprintf(&hook);
}

void save()
{
    static char buf[RING_SZ];
    const std::size_t n = linearize(buf, sizeof(buf));

    nvs_handle_t h;
    if (nvs_open(NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_set_blob(h, KEY, buf, n);
    nvs_commit(h);
    nvs_close(h);
}

std::size_t copy_live(char* out, std::size_t cap)
{
    return linearize(out, cap);
}

std::size_t copy_persisted(char* out, std::size_t cap)
{
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READONLY, &h) != ESP_OK) {
        return 0;
    }
    std::size_t len = cap;
    if (nvs_get_blob(h, KEY, out, &len) != ESP_OK) {
        len = 0;
    }
    nvs_close(h);
    return len;
}

}  // namespace hvmrf01::logtap
