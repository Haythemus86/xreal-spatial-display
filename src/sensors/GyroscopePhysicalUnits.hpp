#pragma once

#include "sensors/GyroscopeBiasCalibration.hpp"
#include "sensors/GyroscopeScaleCalibration.hpp"
#include "sensors/ImuSample.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>

namespace xreal::sensors
{

struct GyroscopeScaleAxisProfile
{
    bool enabled{true};
    bool valid{true};
    double rawUnitsPerDegreePerSecond{};
    double rawUnitsPerRadianPerSecond{};
    double degreesPerSecondPerRawUnit{};
    double radiansPerSecondPerRawUnit{};
};

struct GyroscopeScaleDevice
{
    std::uint16_t vendorId{};
    std::uint16_t productId{};
    int interfaceNumber{};
    std::string productName;
};

enum class GyroscopeLogicalAxis
{
    sensorX,
    sensorY,
    sensorZ,
};

struct GyroscopeAxisMappingEntry
{
    GyroscopeLogicalAxis logicalAxis{GyroscopeLogicalAxis::sensorX};
    int sign{1};
};

struct GyroscopeAxisMapping
{
    GyroscopeAxisMappingEntry sensorX{GyroscopeLogicalAxis::sensorX, 1};
    GyroscopeAxisMappingEntry sensorY{GyroscopeLogicalAxis::sensorY, 1};
    GyroscopeAxisMappingEntry sensorZ{GyroscopeLogicalAxis::sensorZ, 1};
    bool experimental{true};
    bool verified{};
    std::string source{"provisional-flat-chair-observation"};
    std::string notes{"Sensor Z was dominant during a flat chair rotation; yaw correspondence remains provisional."};
};

struct GyroscopeScaleProfile
{
    std::uint32_t schemaVersion{1};
    GyroscopeScaleAxisProfile x;
    GyroscopeScaleAxisProfile y;
    GyroscopeScaleAxisProfile z;
    std::string source;
    bool experimental{true};
    bool verified{};
    bool axesShareSameScale{true};
    bool axesShareSameScaleVerified{};
    std::string notes;
    std::optional<GyroscopeScaleDevice> device;
    GyroscopeAxisMapping axisMapping;
};

enum class GyroscopeScaleValidationReason
{
    none,
    unsupportedSchema,
    invalidScale,
    invalidInverseScale,
    contradictoryValidity,
    invalidAxisMapping,
};

struct GyroscopeScaleValidationResult
{
    bool valid{};
    GyroscopeScaleValidationReason reason{GyroscopeScaleValidationReason::none};
    std::string explanation;
};

struct GyroscopePhysicalAxisValue
{
    bool valid{};
    double degreesPerSecond{};
    double radiansPerSecond{};
};

struct GyroscopePhysicalSample
{
    RawImuVector3 raw;
    CorrectedGyroscopeRaw biasCorrectedRaw;
    GyroscopePhysicalAxisValue x;
    GyroscopePhysicalAxisValue y;
    GyroscopePhysicalAxisValue z;
    std::string scaleSource;
    bool scaleExperimental{};
    bool scaleVerified{};
};

struct GyroscopeScaleComparisonSample
{
    double correctedRaw{};
    double selectedDegreesPerSecond{};
    double comparisonDegreesPerSecond{};
    double degreesPerSecondDifference{};
    double selectedIntegratedDegrees{};
    double comparisonIntegratedDegrees{};
    double integratedDegreesDifference{};
};

struct GyroscopeScaleComparisonReport
{
    double selectedRawUnitsPerDegreePerSecond{};
    double comparisonRawUnitsPerDegreePerSecond{};
    double absoluteScaleDifference{};
    double relativeScaleDifferencePercent{};
    std::array<GyroscopeScaleComparisonSample, 4> representativeSamples{};
    bool documentaryProof{};
};

struct GyroscopeAngleComparison
{
    double integratedRaw{};
    double expectedDegrees{};
    double selectedDegrees{};
    double selectedRadians{};
    double selectedSignedErrorDegrees{};
    double selectedAbsoluteErrorDegrees{};
    double selectedPercentageError{};
    double comparisonDegrees{};
    double comparisonSignedErrorDegrees{};
    double comparisonAbsoluteErrorDegrees{};
    double relativeResultDifferencePercent{};
    bool selectedFitsBetter{};
};

struct GyroscopeAngleErrorStatistics
{
    double meanAbsoluteErrorDegrees{};
    double medianAbsoluteErrorDegrees{};
    double rootMeanSquareErrorDegrees{};
    double positiveMeanSignedErrorDegrees{};
    double negativeMeanSignedErrorDegrees{};
    bool selectedFitsBetter{};
};

struct GyroscopeScaleProfileLoadResult
{
    std::optional<GyroscopeScaleProfile> profile;
    std::string error;
};

[[nodiscard]] GyroscopeScaleProfile makeExperimentalGyroscopeScaleProfile(
    double rawUnitsPerDegreePerSecond);
[[nodiscard]] GyroscopeAxisMapping makeExperimentalXrealAir2UltraGyroscopeAxisMapping();
[[nodiscard]] GyroscopeScaleValidationResult validateGyroscopeScaleProfile(
    const GyroscopeScaleProfile& profile) noexcept;
[[nodiscard]] GyroscopePhysicalSample convertGyroscopeToPhysicalUnits(
    const RawImuVector3& raw,
    const GyroscopeBias& bias,
    const GyroscopeScaleProfile& profile) noexcept;
[[nodiscard]] CorrectedGyroscopeRaw applyGyroscopeAxisMapping(
    const CorrectedGyroscopeRaw& sensorValues,
    const GyroscopeAxisMapping& mapping) noexcept;
[[nodiscard]] bool validateGyroscopeAxisMapping(
    const GyroscopeAxisMapping& mapping) noexcept;
[[nodiscard]] GyroscopeScaleComparisonReport compareGyroscopeScales(
    double selectedRawUnitsPerDegreePerSecond,
    double comparisonRawUnitsPerDegreePerSecond,
    double representativeDurationSeconds = 1.0);
[[nodiscard]] GyroscopeAngleComparison compareGyroscopeIntegratedAngle(
    double integratedRaw,
    double expectedDegrees,
    double selectedRawUnitsPerDegreePerSecond,
    double comparisonRawUnitsPerDegreePerSecond) noexcept;
[[nodiscard]] GyroscopeAngleErrorStatistics calculateGyroscopeAngleErrorStatistics(
    std::span<const GyroscopeAngleComparison> comparisons);
[[nodiscard]] std::string serializeGyroscopeScaleProfileJson(
    const GyroscopeScaleProfile& profile);
[[nodiscard]] GyroscopeScaleProfileLoadResult loadGyroscopeScaleProfileJson(
    std::string_view json);
[[nodiscard]] std::string gyroscopeScaleValidationReasonText(
    GyroscopeScaleValidationReason reason);
[[nodiscard]] std::optional<std::string> gyroscopeScaleProvenanceWarning(
    const GyroscopeScaleProfile& profile);

} // namespace xreal::sensors
