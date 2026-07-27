#include "sensors/AccelerometerPhysicalUnits.hpp"

#include <array>
#include <charconv>
#include <cmath>
#include <limits>

namespace xreal::sensors
{
namespace
{

[[nodiscard]] std::optional<std::string_view> objectAfter(
    std::string_view json,
    std::string_view key)
{
    const std::string quoted = "\"" + std::string(key) + "\"";
    std::size_t searchPosition{};
    while (searchPosition < json.size())
    {
        const std::size_t keyPosition = json.find(quoted, searchPosition);
        if (keyPosition == std::string_view::npos)
        {
            return std::nullopt;
        }
        std::size_t cursor = keyPosition + quoted.size();
        while (cursor < json.size()
               && (json[cursor] == ' ' || json[cursor] == '\t'
                   || json[cursor] == '\r' || json[cursor] == '\n'))
        {
            ++cursor;
        }
        if (cursor < json.size() && json[cursor] == ':')
        {
            ++cursor;
            while (cursor < json.size()
                   && (json[cursor] == ' ' || json[cursor] == '\t'
                       || json[cursor] == '\r' || json[cursor] == '\n'))
            {
                ++cursor;
            }
            if (cursor < json.size() && json[cursor] == '{')
            {
                const std::size_t end = json.find('}', cursor + 1U);
                if (end == std::string_view::npos)
                {
                    return std::nullopt;
                }
                return json.substr(cursor, end - cursor + 1U);
            }
        }
        searchPosition = keyPosition + quoted.size();
    }
    return std::nullopt;
}

[[nodiscard]] std::optional<double> numberAfter(std::string_view json, std::string_view key)
{
    const std::string quoted = "\"" + std::string(key) + "\"";
    const std::size_t keyPosition = json.find(quoted);
    if (keyPosition == std::string_view::npos)
    {
        return std::nullopt;
    }
    const std::size_t colon = json.find(':', keyPosition + quoted.size());
    if (colon == std::string_view::npos)
    {
        return std::nullopt;
    }
    const char* begin = json.data() + colon + 1U;
    const char* end = json.data() + json.size();
    while (begin != end && (*begin == ' ' || *begin == '\t' || *begin == '\r' || *begin == '\n'))
    {
        ++begin;
    }
    double value{};
    const auto result = std::from_chars(begin, end, value);
    if (result.ec != std::errc{} || !std::isfinite(value))
    {
        return std::nullopt;
    }
    return value;
}

[[nodiscard]] PhysicalVector3d mapped(
    const PhysicalVector3d& value,
    const AccelerometerAxisMapping& mapping) noexcept
{
    PhysicalVector3d result;
    const auto assign = [&](const AccelerometerAxisMappingEntry& entry, double component) {
        double* destination{};
        switch (entry.logicalAxis)
        {
        case AccelerometerLogicalAxis::x: destination = &result.x; break;
        case AccelerometerLogicalAxis::y: destination = &result.y; break;
        case AccelerometerLogicalAxis::z: destination = &result.z; break;
        }
        *destination = component * static_cast<double>(entry.sign);
    };
    assign(mapping.sensorX, value.x);
    assign(mapping.sensorY, value.y);
    assign(mapping.sensorZ, value.z);
    return result;
}

} // namespace

bool PhysicalVector3d::finite() const noexcept
{
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

double PhysicalVector3d::norm() const noexcept
{
    return std::sqrt(x * x + y * y + z * z);
}

bool validateAccelerometerAxisMapping(const AccelerometerAxisMapping& mapping) noexcept
{
    const std::array entries{mapping.sensorX, mapping.sensorY, mapping.sensorZ};
    std::array<bool, 3> targets{};
    for (const auto& entry : entries)
    {
        if (entry.sign != -1 && entry.sign != 1)
        {
            return false;
        }
        const std::size_t target = static_cast<std::size_t>(entry.logicalAxis);
        if (target >= targets.size() || targets[target])
        {
            return false;
        }
        targets[target] = true;
    }
    return true;
}

bool validateAccelerometerCalibrationProfile(
    const AccelerometerCalibrationProfile& profile) noexcept
{
    if (profile.schemaVersion != 1U || !validateAccelerometerAxisMapping(profile.axisMapping))
    {
        return false;
    }
    for (const auto* axis : {&profile.x, &profile.y, &profile.z})
    {
        if (!axis->valid || !std::isfinite(axis->offsetRaw)
            || !std::isfinite(axis->rawUnitsPerG) || axis->rawUnitsPerG <= 0.0)
        {
            return false;
        }
    }
    return true;
}

AccelerometerPhysicalSample convertAccelerometerToPhysicalUnits(
    const RawImuVector3& raw,
    const AccelerometerCalibrationProfile& profile) noexcept
{
    AccelerometerPhysicalSample result;
    result.raw = raw;
    if (!validateAccelerometerCalibrationProfile(profile))
    {
        return result;
    }
    const PhysicalVector3d correctedSensor{
        static_cast<double>(raw.x) - profile.x.offsetRaw,
        static_cast<double>(raw.y) - profile.y.offsetRaw,
        static_cast<double>(raw.z) - profile.z.offsetRaw,
    };
    const PhysicalVector3d accelerationSensorG{
        correctedSensor.x / profile.x.rawUnitsPerG,
        correctedSensor.y / profile.y.rawUnitsPerG,
        correctedSensor.z / profile.z.rawUnitsPerG,
    };
    result.correctedRaw = mapped(correctedSensor, profile.axisMapping);
    result.accelerationG = mapped(accelerationSensorG, profile.axisMapping);
    result.accelerationMetersPerSecondSquared = {
        result.accelerationG.x * standardGravityMetersPerSecondSquared,
        result.accelerationG.y * standardGravityMetersPerSecondSquared,
        result.accelerationG.z * standardGravityMetersPerSecondSquared,
    };
    result.normG = result.accelerationG.norm();
    result.valid = result.correctedRaw.finite() && result.accelerationG.finite()
        && result.accelerationMetersPerSecondSquared.finite() && std::isfinite(result.normG);
    return result;
}

AccelerometerProfileLoadResult loadAccelerometerCalibrationProfileJson(
    std::string_view json,
    std::string source)
{
    const auto schema = numberAfter(json, "schema_version");
    const auto offsets = objectAfter(json, "offset_raw");
    const auto scales = objectAfter(json, "raw_units_per_g");
    if (!schema.has_value() || *schema != 1.0 || !offsets.has_value() || !scales.has_value())
    {
        return {std::nullopt, "Missing or unsupported accelerometer calibration profile fields."};
    }
    const auto offsetX = numberAfter(*offsets, "x");
    const auto offsetY = numberAfter(*offsets, "y");
    const auto offsetZ = numberAfter(*offsets, "z");
    const auto scaleX = numberAfter(*scales, "x");
    const auto scaleY = numberAfter(*scales, "y");
    const auto scaleZ = numberAfter(*scales, "z");
    if (!offsetX || !offsetY || !offsetZ || !scaleX || !scaleY || !scaleZ)
    {
        return {std::nullopt, "Invalid accelerometer offset or raw-units-per-g field."};
    }
    AccelerometerCalibrationProfile profile;
    profile.x = {*offsetX, *scaleX, true};
    profile.y = {*offsetY, *scaleY, true};
    profile.z = {*offsetZ, *scaleZ, true};
    profile.source = std::move(source);
    if (!validateAccelerometerCalibrationProfile(profile))
    {
        return {std::nullopt, "Accelerometer calibration profile is not usable."};
    }
    return {profile, {}};
}

} // namespace xreal::sensors
