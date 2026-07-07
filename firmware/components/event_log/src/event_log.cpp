#include "hv-mrf-01/event_log.hpp"

#include <algorithm>

#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace hvmrf01::event_log {

namespace {

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

constexpr std::uint32_t MAGIC   = 0x48564C47;  // HVLG
constexpr std::uint16_t VERSION = 1;

struct Store
{
    std::uint32_t magic;
    std::uint16_t version;
    std::uint16_t record_size;
    std::uint32_t capacity;
    std::uint32_t next_seq;
    std::uint32_t head;
    std::uint32_t count;
    Record records[CAPACITY];
};

RTC_NOINIT_ATTR Store store;

void reset_locked() noexcept
{
    store.magic       = MAGIC;
    store.version     = VERSION;
    store.record_size = sizeof(Record);
    store.capacity    = CAPACITY;
    store.next_seq    = 0;
    store.head        = 0;
    store.count       = 0;
}

void init_locked() noexcept
{
    if (store.magic == MAGIC && store.version == VERSION &&
        store.record_size == sizeof(Record) && store.capacity == CAPACITY &&
        store.head < CAPACITY && store.count <= CAPACITY) {
        return;
    }

    reset_locked();
}

}  // namespace

void record(Event event, std::uint8_t detail, std::int32_t a,
            std::int32_t b, std::int32_t c) noexcept
{
    portENTER_CRITICAL(&mux);
    init_locked();
    const std::uint32_t tick = xTaskGetTickCount();
    store.records[store.head] = Record{
        .tick     = tick,
        .seq      = store.next_seq,
        .event    = event,
        .detail   = detail,
        .reserved = 0,
        .a        = a,
        .b        = b,
        .c        = c,
    };
    store.next_seq++;
    store.head = (store.head + 1) % CAPACITY;
    store.count = std::min<std::uint32_t>(store.count + 1, CAPACITY);
    portEXIT_CRITICAL(&mux);
}

std::size_t snapshot(Record* out, std::size_t max) noexcept
{
    if (out == nullptr || max == 0) {
        return 0;
    }

    portENTER_CRITICAL(&mux);
    init_locked();
    const std::size_t n = std::min<std::size_t>(store.count, max);
    const std::size_t start = (store.head + CAPACITY - n) % CAPACITY;
    for (std::size_t i = 0; i < n; ++i) {
        out[i] = store.records[(start + i) % CAPACITY];
    }
    portEXIT_CRITICAL(&mux);

    return n;
}

std::size_t size() noexcept
{
    portENTER_CRITICAL(&mux);
    init_locked();
    const std::size_t n = store.count;
    portEXIT_CRITICAL(&mux);

    return n;
}

Range range() noexcept
{
    portENTER_CRITICAL(&mux);
    init_locked();
    Range r{};
    if (store.count > 0) {
        const std::size_t oldest = (store.head + CAPACITY - store.count) % CAPACITY;
        r.first_seq = store.records[oldest].seq;
        r.count     = store.count;
    }
    portEXIT_CRITICAL(&mux);

    return r;
}

bool get(std::size_t offset_from_oldest, Record& out) noexcept
{
    portENTER_CRITICAL(&mux);
    init_locked();
    const bool ok = offset_from_oldest < store.count;
    if (ok) {
        const std::size_t start = (store.head + CAPACITY - store.count) % CAPACITY;
        out = store.records[(start + offset_from_oldest) % CAPACITY];
    }
    portEXIT_CRITICAL(&mux);

    return ok;
}

bool get_seq(std::uint32_t seq, Record& out) noexcept
{
    portENTER_CRITICAL(&mux);
    init_locked();
    bool ok = false;
    for (std::size_t i = 0; i < store.count; ++i) {
        if (store.records[i].seq != seq) {
            continue;
        }
        out = store.records[i];
        ok  = true;
        break;
    }
    portEXIT_CRITICAL(&mux);

    return ok;
}

void clear() noexcept
{
    portENTER_CRITICAL(&mux);
    reset_locked();
    portEXIT_CRITICAL(&mux);
}

const char* event_name(Event event) noexcept
{
    using enum Event;
    switch (event) {
    case ZigbeeCoverRx:     return "zigbee.cover.rx";
    case ZigbeeCoverStatus: return "zigbee.cover.status";
    case ZigbeeConfigCmd:   return "zigbee.config.cmd";
    case MotionOpen:        return "motion.open";
    case MotionClose:       return "motion.close";
    case MotionGoTo:        return "motion.goto";
    case MotionStop:        return "motion.stop";
    case MotionSetTarget:   return "motion.target";
    case MotionDriveStart:  return "motion.drive.start";
    case MotionReject:      return "motion.reject";
    case HomeBegin:         return "home.begin";
    case HomeStart:         return "home.start";
    case HomeSettled:       return "home.settled";
    case HomeDone:          return "home.done";
    case GoToBegin:         return "goto.begin";
    case GoToArrived:       return "goto.arrived";
    case GoToDone:          return "goto.done";
    case LimitStop:         return "limit.stop";
    case FaultSync:         return "fault.sync";
    case FaultStall:        return "fault.stall";
    }
    return "?";
}

}  // namespace hvmrf01::event_log
