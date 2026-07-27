#include "sensors/GyroscopePhysicalUnits.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <iomanip>
#include <limits>
#include <numbers>
#include <sstream>
#include <vector>

namespace xreal::sensors
{
namespace
{

[[nodiscard]] GyroscopeScaleAxisProfile axisProfile(double rawPerDegree)
{
    GyroscopeScaleAxisProfile axis;
    axis.rawUnitsPerDegreePerSecond = rawPerDegree;
    axis.rawUnitsPerRadianPerSecond = rawPerDegree * 180.0 / std::numbers::pi;
    axis.degreesPerSecondPerRawUnit = 1.0 / rawPerDegree;
    axis.radiansPerSecondPerRawUnit = 1.0 / axis.rawUnitsPerRadianPerSecond;
    return axis;
}

[[nodiscard]] bool finitePositive(double value) noexcept
{
    return std::isfinite(value) && value > 0.0;
}

[[nodiscard]] GyroscopeScaleValidationResult validateAxis(
    const GyroscopeScaleAxisProfile& axis,
    std::string_view name) noexcept
{
    if (!axis.enabled)
    {
        return {true, {}, {}};
    }
    if (!axis.valid)
    {
        return {false, GyroscopeScaleValidationReason::contradictoryValidity,
                std::string(name) + " is enabled but marked invalid."};
    }
    if (!finitePositive(axis.rawUnitsPerDegreePerSecond)
        || !finitePositive(axis.rawUnitsPerRadianPerSecond))
    {
        return {false, GyroscopeScaleValidationReason::invalidScale,
                std::string(name) + " contains a non-finite or non-positive scale."};
    }
    if (!finitePositive(axis.degreesPerSecondPerRawUnit)
        || !finitePositive(axis.radiansPerSecondPerRawUnit))
    {
        return {false, GyroscopeScaleValidationReason::invalidInverseScale,
                std::string(name) + " contains an invalid inverse scale."};
    }
    const double expectedRawPerRadian = axis.rawUnitsPerDegreePerSecond
        * 180.0 / std::numbers::pi;
    const double expectedDegreesInverse = 1.0 / axis.rawUnitsPerDegreePerSecond;
    const double expectedRadiansInverse = 1.0 / expectedRawPerRadian;
    const auto differs = [](double actual, double expected) {
        return std::abs(actual - expected) > std::abs(expected) * 1.0e-12;
    };
    if (differs(axis.rawUnitsPerRadianPerSecond, expectedRawPerRadian)
        || differs(axis.degreesPerSecondPerRawUnit, expectedDegreesInverse)
        || differs(axis.radiansPerSecondPerRawUnit, expectedRadiansInverse))
    {
        return {false, GyroscopeScaleValidationReason::invalidInverseScale,
                std::string(name) + " scale and inverse fields are inconsistent."};
    }
    return {true, {}, {}};
}

[[nodiscard]] std::optional<std::string_view> valueAfter(
    std::string_view json,
    std::string_view key,
    std::size_t occurrence = 0U)
{
    std::size_t position{};
    for (std::size_t index = 0; index <= occurrence; ++index)
    {
        position = json.find(key, position);
        if (position == std::string_view::npos)
        {
            return std::nullopt;
        }
        position += key.size();
    }
    while (position < json.size() && (json[position] == ' ' || json[position] == '\t'
           || json[position] == '\r' || json[position] == '\n' || json[position] == ':'))
    {
        ++position;
    }
    return json.substr(position);
}

[[nodiscard]] std::optional<double> number(std::string_view json, std::string_view key,
                                           std::size_t occurrence = 0U)
{
    const auto source = valueAfter(json, key, occurrence);
    if (!source.has_value())
    {
        return std::nullopt;
    }
    double result{};
    const auto parsed = std::from_chars(source->data(), source->data() + source->size(), result);
    if (parsed.ec != std::errc{} || !std::isfinite(result))
    {
        return std::nullopt;
    }
    return result;
}

[[nodiscard]] std::optional<bool> boolean(std::string_view json, std::string_view key,
                                          std::size_t occurrence = 0U)
{
    const auto source = valueAfter(json, key, occurrence);
    if (!source.has_value())
    {
        return std::nullopt;
    }
    if (source->starts_with("true"))
    {
        return true;
    }
    if (source->starts_with("false"))
    {
        return false;
    }
    return std::nullopt;
}

[[nodiscard]] std::optional<std::string> stringValue(
    std::string_view json,
    std::string_view key)
{
    const auto source = valueAfter(json, key);
    if (!source.has_value() || source->empty() || source->front() != '"')
    {
        return std::nullopt;
    }
    const std::size_t end = source->find('"', 1U);
    if (end == std::string_view::npos)
    {
        return std::nullopt;
    }
    return std::string(source->substr(1U, end - 1U));
}

[[nodiscard]] GyroscopePhysicalAxisValue convertAxis(
    double corrected,
    const GyroscopeScaleAxisProfile& scale) noexcept
{
    if (!scale.enabled || !scale.valid || !finitePositive(scale.rawUnitsPerDegreePerSecond)
        || !finitePositive(scale.rawUnitsPerRadianPerSecond))
    {
        return {};
    }
    return {true, corrected / scale.rawUnitsPerDegreePerSecond,
            corrected / scale.rawUnitsPerRadianPerSecond};
}

} // namespace

GyroscopeScaleProfile makeExperimentalGyroscopeScaleProfile(double rawUnitsPerDegreePerSecond)
{
    GyroscopeScaleProfile profile;
    profile.x = axisProfile(rawUnitsPerDegreePerSecond);
    profile.y = axisProfile(rawUnitsPerDegreePerSecond);
    profile.z = axisProfile(rawUnitsPerDegreePerSecond);
    profile.source = "experimental-manual-calibration";
    profile.experimental = true;
    profile.verified = false;
    profile.axesShareSameScale = true;
    profile.axesShareSameScaleVerified = false;
    profile.notes = "Candidate scale selected from manual measurements and Q12 comparison; equal axis scales are unverified.";
    profile.device = GyroscopeScaleDevice{0x3318, 0x0426, 2, "XREAL Air 2 Ultra"};
    return profile;
}

GyroscopeScaleValidationResult validateGyroscopeScaleProfile(
    const GyroscopeScaleProfile& profile) noexcept
{
    if (profile.schemaVersion != 1U)
    {
        return {false, GyroscopeScaleValidationReason::unsupportedSchema,
                "Only gyroscope scale profile schema version 1 is supported."};
    }
    for (const auto pair : {std::pair{&profile.x, "X"}, std::pair{&profile.y, "Y"},
                            std::pair{&profile.z, "Z"}})
    {
        const auto validation = validateAxis(*pair.first, pair.second);
        if (!validation.valid)
        {
            return validation;
        }
    }
    if (!validateGyroscopeAxisMapping(profile.axisMapping))
    {
        return {false, GyroscopeScaleValidationReason::invalidAxisMapping,
                "Axis mapping must be a permutation with signs +1 or -1."};
    }
    return {true, {}, {}};
}

GyroscopePhysicalSample convertGyroscopeToPhysicalUnits(
    const RawImuVector3& raw,
    const GyroscopeBias& bias,
    const GyroscopeScaleProfile& profile) noexcept
{
    GyroscopePhysicalSample result;
    result.raw = raw;
    result.biasCorrectedRaw = applyGyroscopeBias(raw, bias);
    result.scaleSource = profile.source;
    result.scaleExperimental = profile.experimental;
    result.scaleVerified = profile.verified;
    if (!validateGyroscopeScaleProfile(profile).valid)
    {
        return result;
    }
    result.x = convertAxis(result.biasCorrectedRaw.x, profile.x);
    result.y = convertAxis(result.biasCorrectedRaw.y, profile.y);
    result.z = convertAxis(result.biasCorrectedRaw.z, profile.z);
    return result;
}

CorrectedGyroscopeRaw applyGyroscopeAxisMapping(
    const CorrectedGyroscopeRaw& sensorValues,
    const GyroscopeAxisMapping& mapping) noexcept
{
    CorrectedGyroscopeRaw result;
    const auto assign = [&](const GyroscopeAxisMappingEntry& entry, double value) {
        const double mapped = value * static_cast<double>(entry.sign);
        switch (entry.logicalAxis)
        {
        case GyroscopeLogicalAxis::sensorX: result.x = mapped; break;
        case GyroscopeLogicalAxis::sensorY: result.y = mapped; break;
        case GyroscopeLogicalAxis::sensorZ: result.z = mapped; break;
        }
    };
    assign(mapping.sensorX, sensorValues.x);
    assign(mapping.sensorY, sensorValues.y);
    assign(mapping.sensorZ, sensorValues.z);
    return result;
}

[[nodiscard]] std::string_view logicalAxisName(GyroscopeLogicalAxis axis) noexcept
{
    switch (axis)
    {
    case GyroscopeLogicalAxis::sensorX: return "x";
    case GyroscopeLogicalAxis::sensorY: return "y";
    case GyroscopeLogicalAxis::sensorZ: return "z";
    }
    return "invalid";
}

[[nodiscard]] std::optional<GyroscopeLogicalAxis> parseLogicalAxis(std::string_view value) noexcept
{
    if (value == "x")
    {
        return GyroscopeLogicalAxis::sensorX;
    }
    if (value == "y")
    {
        return GyroscopeLogicalAxis::sensorY;
    }
    if (value == "z")
    {
        return GyroscopeLogicalAxis::sensorZ;
    }
    return std::nullopt;
}

bool validateGyroscopeAxisMapping(const GyroscopeAxisMapping& mapping) noexcept
{
    const std::array entries{mapping.sensorX, mapping.sensorY, mapping.sensorZ};
    std::array<bool, 3> targets{};
    for (const auto& entry : entries)
    {
        if (entry.sign != -1 && entry.sign != 1)
        {
            return false;
        }
        const std::size_t index = static_cast<std::size_t>(entry.logicalAxis);
        if (index >= targets.size() || targets[index])
        {
            return false;
        }
        targets[index] = true;
    }
    return true;
}

GyroscopeScaleComparisonReport compareGyroscopeScales(
    double selected,
    double comparison,
    double duration)
{
    GyroscopeScaleComparisonReport result;
    result.selectedRawUnitsPerDegreePerSecond = selected;
    result.comparisonRawUnitsPerDegreePerSecond = comparison;
    result.absoluteScaleDifference = std::abs(selected - comparison);
    result.relativeScaleDifferencePercent = result.absoluteScaleDifference / comparison * 100.0;
    constexpr std::array<double, 4> rawValues{4090.0, 40960.0, 100000.0, 200000.0};
    for (std::size_t index = 0; index < rawValues.size(); ++index)
    {
        auto& sample = result.representativeSamples[index];
        sample.correctedRaw = rawValues[index];
        sample.selectedDegreesPerSecond = rawValues[index] / selected;
        sample.comparisonDegreesPerSecond = rawValues[index] / comparison;
        sample.degreesPerSecondDifference = sample.selectedDegreesPerSecond
            - sample.comparisonDegreesPerSecond;
        sample.selectedIntegratedDegrees = sample.selectedDegreesPerSecond * duration;
        sample.comparisonIntegratedDegrees = sample.comparisonDegreesPerSecond * duration;
        sample.integratedDegreesDifference = sample.selectedIntegratedDegrees
            - sample.comparisonIntegratedDegrees;
    }
    return result;
}

GyroscopeAngleComparison compareGyroscopeIntegratedAngle(
    double integratedRaw,
    double expectedDegrees,
    double selected,
    double comparison) noexcept
{
    GyroscopeAngleComparison result;
    result.integratedRaw = integratedRaw;
    result.expectedDegrees = expectedDegrees;
    result.selectedDegrees = integratedRaw / selected;
    result.selectedRadians = result.selectedDegrees * std::numbers::pi / 180.0;
    result.selectedSignedErrorDegrees = result.selectedDegrees - expectedDegrees;
    result.selectedAbsoluteErrorDegrees = std::abs(result.selectedSignedErrorDegrees);
    result.selectedPercentageError = expectedDegrees != 0.0
        ? result.selectedAbsoluteErrorDegrees / std::abs(expectedDegrees) * 100.0 : 0.0;
    result.comparisonDegrees = integratedRaw / comparison;
    result.comparisonSignedErrorDegrees = result.comparisonDegrees - expectedDegrees;
    result.comparisonAbsoluteErrorDegrees = std::abs(result.comparisonSignedErrorDegrees);
    result.relativeResultDifferencePercent = result.selectedDegrees != 0.0
        ? std::abs(result.selectedDegrees - result.comparisonDegrees)
            / std::abs(result.selectedDegrees) * 100.0 : 0.0;
    result.selectedFitsBetter = result.selectedAbsoluteErrorDegrees
        < result.comparisonAbsoluteErrorDegrees;
    return result;
}

GyroscopeAngleErrorStatistics calculateGyroscopeAngleErrorStatistics(
    std::span<const GyroscopeAngleComparison> comparisons)
{
    GyroscopeAngleErrorStatistics result;
    if (comparisons.empty())
    {
        return result;
    }
    std::vector<double> absoluteErrors;
    absoluteErrors.reserve(comparisons.size());
    double selectedSum{};
    double comparisonSum{};
    double squared{};
    double positiveSum{};
    double negativeSum{};
    std::size_t positiveCount{};
    std::size_t negativeCount{};
    for (const auto& comparison : comparisons)
    {
        absoluteErrors.push_back(comparison.selectedAbsoluteErrorDegrees);
        selectedSum += comparison.selectedAbsoluteErrorDegrees;
        comparisonSum += comparison.comparisonAbsoluteErrorDegrees;
        squared += comparison.selectedSignedErrorDegrees * comparison.selectedSignedErrorDegrees;
        if (comparison.expectedDegrees >= 0.0)
        {
            positiveSum += comparison.selectedSignedErrorDegrees;
            ++positiveCount;
        }
        else
        {
            negativeSum += comparison.selectedSignedErrorDegrees;
            ++negativeCount;
        }
    }
    std::sort(absoluteErrors.begin(), absoluteErrors.end());
    const std::size_t middle = absoluteErrors.size() / 2U;
    result.meanAbsoluteErrorDegrees = selectedSum / comparisons.size();
    result.medianAbsoluteErrorDegrees = absoluteErrors.size() % 2U == 0U
        ? (absoluteErrors[middle - 1U] + absoluteErrors[middle]) * 0.5
        : absoluteErrors[middle];
    result.rootMeanSquareErrorDegrees = std::sqrt(squared / comparisons.size());
    result.positiveMeanSignedErrorDegrees = positiveCount == 0U
        ? 0.0 : positiveSum / positiveCount;
    result.negativeMeanSignedErrorDegrees = negativeCount == 0U
        ? 0.0 : negativeSum / negativeCount;
    result.selectedFitsBetter = selectedSum <= comparisonSum;
    return result;
}

std::string serializeGyroscopeScaleProfileJson(const GyroscopeScaleProfile& profile)
{
    std::ostringstream output;
    output << std::setprecision(std::numeric_limits<double>::max_digits10)
           << "{\n  \"schema_version\":" << profile.schemaVersion;
    if (profile.device.has_value())
    {
        output << ",\n  \"device\":{\"vid\":\"0x" << std::hex << std::uppercase
               << profile.device->vendorId << "\",\"pid\":\"0x" << profile.device->productId
               << std::dec << "\",\"interface_number\":" << profile.device->interfaceNumber
               << ",\"product\":\"" << profile.device->productName << "\"}";
    }
    output << ",\n  \"calibration\":{\"type\":\"gyroscope-scale\",\"source\":\""
           << profile.source << "\",\"experimental\":" << (profile.experimental ? "true" : "false")
           << ",\"verified\":" << (profile.verified ? "true" : "false")
           << ",\"axes_share_same_scale\":" << (profile.axesShareSameScale ? "true" : "false")
           << ",\"axes_share_same_scale_verified\":"
           << (profile.axesShareSameScaleVerified ? "true" : "false")
           << ",\"validity\":{\"x\":" << (profile.x.valid ? "true" : "false")
           << ",\"y\":" << (profile.y.valid ? "true" : "false")
           << ",\"z\":" << (profile.z.valid ? "true" : "false")
           << "},\"scale\":{\"raw_units_per_degree_per_second\":{\"x\":"
           << profile.x.rawUnitsPerDegreePerSecond << ",\"y\":"
           << profile.y.rawUnitsPerDegreePerSecond << ",\"z\":"
           << profile.z.rawUnitsPerDegreePerSecond
           << "},\"raw_units_per_radian_per_second\":{\"x\":"
           << profile.x.rawUnitsPerRadianPerSecond << ",\"y\":"
           << profile.y.rawUnitsPerRadianPerSecond << ",\"z\":"
           << profile.z.rawUnitsPerRadianPerSecond
           << "}},\"axis_mapping\":{\"source\":\"" << profile.axisMapping.source
           << "\",\"notes\":\"" << profile.axisMapping.notes << "\",\"experimental\":"
           << (profile.axisMapping.experimental ? "true" : "false")
           << ",\"verified\":" << (profile.axisMapping.verified ? "true" : "false")
           << ",\"sensor_x_target\":\""
           << logicalAxisName(profile.axisMapping.sensorX.logicalAxis) << '"'
           << ",\"sensor_x_sign\":" << profile.axisMapping.sensorX.sign
           << ",\"sensor_y_target\":\""
           << logicalAxisName(profile.axisMapping.sensorY.logicalAxis) << '"'
           << ",\"sensor_y_sign\":" << profile.axisMapping.sensorY.sign
           << ",\"sensor_z_target\":\""
           << logicalAxisName(profile.axisMapping.sensorZ.logicalAxis) << '"'
           << ",\"sensor_z_sign\":" << profile.axisMapping.sensorZ.sign
           << "},\"notes\":\"" << profile.notes << "\"}\n}\n";
    return output.str();
}

GyroscopeScaleProfileLoadResult loadGyroscopeScaleProfileJson(std::string_view json)
{
    const auto schema = number(json, "\"schema_version\"");
    const auto source = stringValue(json, "\"source\"");
    const auto experimental = boolean(json, "\"experimental\"");
    const auto verified = boolean(json, "\"verified\"");
    const std::size_t scalePosition = json.find("\"raw_units_per_degree_per_second\"");
    const std::string_view scaleJson = scalePosition == std::string_view::npos
        ? std::string_view{} : json.substr(scalePosition);
    const auto x = number(scaleJson, "\"x\"");
    const auto y = number(scaleJson, "\"y\"");
    const auto z = number(scaleJson, "\"z\"");
    if (!schema.has_value() || !source.has_value() || !experimental.has_value()
        || !verified.has_value() || !x.has_value() || !y.has_value() || !z.has_value())
    {
        return {std::nullopt, "Missing or invalid required gyroscope scale profile field."};
    }
    if (*schema != 1.0)
    {
        return {std::nullopt, "Unsupported gyroscope scale profile schema version."};
    }
    GyroscopeScaleProfile profile;
    profile.x = axisProfile(*x);
    profile.y = axisProfile(*y);
    profile.z = axisProfile(*z);
    profile.source = *source;
    profile.experimental = *experimental;
    profile.verified = *verified;
    profile.axesShareSameScale = std::abs(*x - *y) < 1.0e-12 && std::abs(*x - *z) < 1.0e-12;
    profile.axesShareSameScaleVerified = false;
    profile.notes = stringValue(json, "\"notes\"").value_or("");
    const std::size_t validityPosition = json.find("\"validity\"");
    if (validityPosition != std::string_view::npos)
    {
        const std::string_view validityJson = json.substr(validityPosition, scalePosition - validityPosition);
        profile.x.valid = boolean(validityJson, "\"x\"").value_or(false);
        profile.y.valid = boolean(validityJson, "\"y\"").value_or(false);
        profile.z.valid = boolean(validityJson, "\"z\"").value_or(false);
    }
    const std::size_t mappingPosition = json.find("\"axis_mapping\"");
    if (mappingPosition != std::string_view::npos)
    {
        const std::string_view mappingJson = json.substr(mappingPosition);
        profile.axisMapping.source = stringValue(mappingJson, "\"source\"")
            .value_or(profile.axisMapping.source);
        profile.axisMapping.notes = stringValue(mappingJson, "\"notes\"")
            .value_or(profile.axisMapping.notes);
        profile.axisMapping.experimental = boolean(mappingJson, "\"experimental\"")
            .value_or(profile.axisMapping.experimental);
        profile.axisMapping.verified = boolean(mappingJson, "\"verified\"")
            .value_or(profile.axisMapping.verified);
        const auto loadMappingEntry = [&](std::string_view targetKey,
                                          std::string_view signKey,
                                          GyroscopeAxisMappingEntry& entry) -> bool {
            const auto targetName = stringValue(mappingJson, targetKey);
            if (targetName.has_value())
            {
                const auto target = parseLogicalAxis(*targetName);
                if (!target.has_value())
                {
                    return false;
                }
                entry.logicalAxis = *target;
            }
            const auto sign = number(mappingJson, signKey);
            if (sign.has_value())
            {
                if (*sign != -1.0 && *sign != 1.0)
                {
                    return false;
                }
                entry.sign = static_cast<int>(*sign);
            }
            return true;
        };
        if (!loadMappingEntry("\"sensor_x_target\"", "\"sensor_x_sign\"", profile.axisMapping.sensorX)
            || !loadMappingEntry("\"sensor_y_target\"", "\"sensor_y_sign\"", profile.axisMapping.sensorY)
            || !loadMappingEntry("\"sensor_z_target\"", "\"sensor_z_sign\"", profile.axisMapping.sensorZ))
        {
            return {std::nullopt, "Invalid gyroscope axis mapping target."};
        }
    }
    const auto validation = validateGyroscopeScaleProfile(profile);
    if (!validation.valid)
    {
        return {std::nullopt, validation.explanation};
    }
    return {profile, {}};
}

std::string gyroscopeScaleValidationReasonText(GyroscopeScaleValidationReason reason)
{
    switch (reason)
    {
    case GyroscopeScaleValidationReason::none: return "none";
    case GyroscopeScaleValidationReason::unsupportedSchema: return "unsupported schema";
    case GyroscopeScaleValidationReason::invalidScale: return "invalid scale";
    case GyroscopeScaleValidationReason::invalidInverseScale: return "invalid inverse scale";
    case GyroscopeScaleValidationReason::contradictoryValidity: return "contradictory validity";
    case GyroscopeScaleValidationReason::invalidAxisMapping: return "invalid axis mapping";
    }
    return "unknown";
}

std::optional<std::string> gyroscopeScaleProvenanceWarning(
    const GyroscopeScaleProfile& profile)
{
    if (profile.experimental || !profile.verified)
    {
        return "Physical gyroscope units use an experimental, unverified scale.";
    }
    return std::nullopt;
}

} // namespace xreal::sensors
