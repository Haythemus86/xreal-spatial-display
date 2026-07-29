#pragma once

#include "sensors/ImuSample.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace xreal::sensors
{

inline constexpr double standardGravityMetersPerSecondSquared = 9.80665;

struct PhysicalVector3d
{
    double x{};
    double y{};
    double z{};

    [[nodiscard]] bool finite() const noexcept;
    [[nodiscard]] double norm() const noexcept;
};

enum class AccelerometerLogicalAxis
{
    x,
    y,
    z,
};

struct AccelerometerAxisMappingEntry
{
    AccelerometerLogicalAxis logicalAxis{AccelerometerLogicalAxis::x};
    int sign{1};
};

struct AccelerometerAxisMapping
{
    AccelerometerAxisMappingEntry sensorX{AccelerometerLogicalAxis::x, 1};
    AccelerometerAxisMappingEntry sensorY{AccelerometerLogicalAxis::y, 1};
    AccelerometerAxisMappingEntry sensorZ{AccelerometerLogicalAxis::z, 1};
    bool experimental{true};
    bool verified{};
    std::string source{"axis-aligned-ellipsoid-profile"};
    std::string notes{"Sensor-to-body mapping remains experimental."};
};

struct AccelerometerCalibrationAxis
{
    double offsetRaw{};
    double rawUnitsPerG{};
    bool valid{true};
};

struct AccelerometerCalibrationProfile
{
    std::uint32_t schemaVersion{1};
    AccelerometerCalibrationAxis x;
    AccelerometerCalibrationAxis y;
    AccelerometerCalibrationAxis z;
    std::string source;
    bool experimental{true};
    bool verified{};
    std::uint16_t vendorId{0x3318};
    std::uint16_t productId{0x0426};
    int interfaceNumber{2};
    AccelerometerAxisMapping axisMapping;
};

struct AccelerometerPhysicalSample
{
    RawImuVector3 raw;
    PhysicalVector3d correctedRaw;
    PhysicalVector3d accelerationG;
    PhysicalVector3d accelerationMetersPerSecondSquared;
    double normG{};
    bool valid{};
};

struct AccelerometerProfileLoadResult
{
    std::optional<AccelerometerCalibrationProfile> profile;
    std::string error;
};

[[nodiscard]] bool validateAccelerometerAxisMapping(
    const AccelerometerAxisMapping& mapping) noexcept;
[[nodiscard]] AccelerometerAxisMapping
makeExperimentalXrealAir2UltraAccelerometerAxisMapping();
[[nodiscard]] bool validateAccelerometerCalibrationProfile(
    const AccelerometerCalibrationProfile& profile) noexcept;
[[nodiscard]] AccelerometerPhysicalSample convertAccelerometerToPhysicalUnits(
    const RawImuVector3& raw,
    const AccelerometerCalibrationProfile& profile) noexcept;
[[nodiscard]] AccelerometerProfileLoadResult loadAccelerometerCalibrationProfileJson(
    std::string_view json,
    std::string source);

} // namespace xreal::sensors
