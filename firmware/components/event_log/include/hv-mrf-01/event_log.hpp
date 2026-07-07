#pragma once

#include <cstddef>
#include <cstdint>

namespace hvmrf01::event_log {

enum class Event : std::uint8_t
{
    ZigbeeCoverRx,
    ZigbeeCoverStatus,
    ZigbeeConfigCmd,
    MotionOpen,
    MotionClose,
    MotionGoTo,
    MotionStop,
    MotionSetTarget,
    MotionDriveStart,
    MotionReject,
    HomeBegin,
    HomeStart,
    HomeSettled,
    HomeDone,
    GoToBegin,
    GoToArrived,
    GoToDone,
    LimitStop,
    FaultSync,
    FaultStall,
};

enum class MotionDirection : std::uint8_t { Raise, Lower, Stop };
enum class StopSource : std::uint8_t { Cover, Controller };
enum class Side : std::uint8_t { Left, Right };

enum class RejectReason : std::uint8_t
{
    Faulted             = 1,
    Homing              = 2,
    LowerUnhomed        = 3,
    GoToBeginFailed     = 4,
    GoToFaulted         = 5,
    GoToHoming          = 6,
    GoToUnhomed         = 7,
    GoToNotCalibrated   = 8,
};

enum class HomeBeginResult : std::uint8_t
{
    Claimed,
    AlreadyRunning,
    TaskCreateFailed,
};

struct Range
{
    std::uint32_t first_seq;
    std::uint32_t count;
};

struct Record
{
    std::uint32_t tick;
    std::uint32_t seq;
    Event         event;
    std::uint8_t  detail;
    std::uint16_t reserved;
    std::int32_t  a;
    std::int32_t  b;
    std::int32_t  c;
};

static_assert(sizeof(Record) == 24);

constexpr std::size_t CAPACITY = 128;

void record(Event event, std::uint8_t detail = 0, std::int32_t a = 0,
            std::int32_t b = 0, std::int32_t c = 0) noexcept;

inline std::int32_t mm10(float mm) noexcept
{
    return static_cast<std::int32_t>(mm * 10.0f);
}

inline void zigbee_cover_received(std::uint8_t command_id, std::uint8_t pct) noexcept
{
    record(Event::ZigbeeCoverRx, command_id, pct);
}

inline void zigbee_cover_status(std::uint8_t command_id, std::int32_t status,
                                std::uint8_t pct) noexcept
{
    record(Event::ZigbeeCoverStatus, command_id, status, pct);
}

inline void zigbee_config_command(std::uint8_t command_id, std::uint16_t cluster_id,
                                  bool unsupported = false) noexcept
{
    record(Event::ZigbeeConfigCmd, command_id, cluster_id, unsupported ? 1 : 0);
}

inline void motion_open(bool homed) noexcept
{
    record(Event::MotionOpen, 0, homed ? 1 : 0);
}

inline void motion_close(bool homed) noexcept
{
    record(Event::MotionClose, 0, homed ? 1 : 0);
}

inline void motion_go_to(std::uint8_t pct, bool homed) noexcept
{
    record(Event::MotionGoTo, 0, pct, homed ? 1 : 0);
}

inline void motion_stop(StopSource source, bool faulted = false, bool position_mode = false,
                        bool homing = false) noexcept
{
    record(Event::MotionStop, static_cast<std::uint8_t>(source),
           faulted ? 1 : 0, position_mode ? 1 : 0, homing ? 1 : 0);
}

inline void motion_target(int rpm, MotionDirection direction) noexcept
{
    record(Event::MotionSetTarget, 0, rpm, static_cast<std::int32_t>(direction));
}

inline void motion_drive_start(MotionDirection direction, int rpm,
                               std::int32_t count_l, std::int32_t count_r) noexcept
{
    record(Event::MotionDriveStart, static_cast<std::uint8_t>(direction), rpm, count_l, count_r);
}

inline void motion_reject(RejectReason reason, std::int32_t value = 0,
                          MotionDirection direction = MotionDirection::Stop) noexcept
{
    record(Event::MotionReject, static_cast<std::uint8_t>(reason), value,
           static_cast<std::int32_t>(direction));
}

inline void home_begin(HomeBeginResult result) noexcept
{
    record(Event::HomeBegin, static_cast<std::uint8_t>(result));
}

inline void home_start(int duty_pct, int settle_ms, int timeout_s) noexcept
{
    record(Event::HomeStart, 0, duty_pct, settle_ms, timeout_s);
}

inline void home_settled(Side side, std::int32_t count_before_reset) noexcept
{
    record(Event::HomeSettled, static_cast<std::uint8_t>(side), count_before_reset);
}

inline void home_done(bool success, bool left, bool right) noexcept
{
    record(Event::HomeDone, success ? 1 : 0, left ? 1 : 0, right ? 1 : 0);
}

inline void go_to_begin(std::int32_t target_counts, int rpm, float target_mm) noexcept
{
    record(Event::GoToBegin, 0, target_counts, rpm, mm10(target_mm));
}

inline void go_to_arrived(std::int32_t pos_counts, std::int32_t target_counts,
                          std::int32_t tolerance_counts) noexcept
{
    record(Event::GoToArrived, 0, pos_counts, target_counts, tolerance_counts);
}

inline void go_to_done(std::uint8_t status, float left_mm, float right_mm) noexcept
{
    record(Event::GoToDone, status, mm10(left_mm), mm10(right_mm));
}

inline void limit_stop(MotionDirection direction, std::int32_t pos_counts,
                       std::int32_t down_limit_counts) noexcept
{
    record(Event::LimitStop, static_cast<std::uint8_t>(direction),
           pos_counts, down_limit_counts);
}

inline void fault_sync(std::int32_t count_l, std::int32_t count_r, std::int32_t sync_err) noexcept
{
    record(Event::FaultSync, 0, count_l, count_r, sync_err);
}

inline void fault_stall(std::int32_t delta_l, std::int32_t delta_r, int rpm) noexcept
{
    record(Event::FaultStall, 0, delta_l, delta_r, rpm);
}

std::size_t snapshot(Record* out, std::size_t max) noexcept;
std::size_t size() noexcept;
Range range() noexcept;
bool get(std::size_t offset_from_oldest, Record& out) noexcept;
bool get_seq(std::uint32_t seq, Record& out) noexcept;
void clear() noexcept;
const char* event_name(Event event) noexcept;

}  // namespace hvmrf01::event_log
