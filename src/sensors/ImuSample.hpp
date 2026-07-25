#pragma once

#include <chrono>
#include <cstdint>
#include <optional>

namespace xreal::sensors
{

struct DeviceTimestamp
{
    std::uint64_t nanoseconds{};
};

struct RawImuVector3
{
    std::int32_t x{};
    std::int32_t y{};
    std::int32_t z{};
};

struct SiImuVector3
{
    double x{};
    double y{};
    double z{};
};

struct ImuSample
{
    DeviceTimestamp deviceTimestamp;
    RawImuVector3 gyroscopeRaw;
    RawImuVector3 accelerometerRaw;
    std::optional<SiImuVector3> gyroscopeRadiansPerSecond;
    std::optional<SiImuVector3> accelerometerMetersPerSecondSquared;
    std::uint8_t packetSequence{};
    std::chrono::steady_clock::time_point hostReceiveTimestamp;
};

} // namespace xreal::sensors
