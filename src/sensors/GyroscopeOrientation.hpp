#pragma once

#include "sensors/GyroscopePhysicalUnits.hpp"

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

namespace xreal::sensors
{

struct Quaternion
{
    double w{1.0};
    double x{};
    double y{};
    double z{};

    [[nodiscard]] static constexpr Quaternion identity() noexcept { return {}; }
    [[nodiscard]] double norm() const noexcept;
    [[nodiscard]] bool finite() const noexcept;
    [[nodiscard]] std::optional<Quaternion> normalized() const noexcept;
    [[nodiscard]] Quaternion conjugate() const noexcept;
    [[nodiscard]] std::optional<Quaternion> inverseNormalized() const noexcept;
};

[[nodiscard]] Quaternion operator*(const Quaternion& left, const Quaternion& right) noexcept;

struct AngularVelocityRadians
{
    double xRadiansPerSecond{};
    double yRadiansPerSecond{};
    double zRadiansPerSecond{};

    [[nodiscard]] bool finite() const noexcept;
};

struct EulerAnglesDiagnostic
{
    double yawRadians{};
    double pitchRadians{};
    double rollRadians{};
    double yawDegrees{};
    double pitchDegrees{};
    double rollDegrees{};
};

enum class OrientationSampleStatus
{
    applied,
    skipped,
    rejected,
};

enum class OrientationRejectionReason
{
    none,
    firstTimestamp,
    duplicateTimestamp,
    decreasingTimestamp,
    excessiveTimestampDelta,
    invalidAngularVelocity,
    invalidAxisMapping,
    invalidOrientation,
};

struct OrientationIntegratorConfig
{
    std::chrono::nanoseconds maximumDeviceTimestampDelta{std::chrono::milliseconds(20)};
    double smallAngleThresholdRadians{1.0e-8};
    GyroscopeAxisMapping axisMapping;
};

struct OrientationState
{
    Quaternion orientation;
    std::optional<std::uint64_t> lastValidDeviceTimestamp;
    bool initialized{};
    bool valid{true};
    std::uint64_t appliedSampleCount{};
    std::uint64_t skippedSampleCount{};
    std::uint64_t rejectedSampleCount{};
    std::chrono::nanoseconds lastDeltaTime{};
    std::chrono::nanoseconds accumulatedIntegrationDuration{};
    OrientationRejectionReason lastRejectionReason{OrientationRejectionReason::none};
};

struct OrientationIntegratorResult
{
    OrientationSampleStatus status{OrientationSampleStatus::skipped};
    OrientationRejectionReason reason{OrientationRejectionReason::none};
    Quaternion absoluteOrientation;
    Quaternion relativeOrientation;
    std::chrono::nanoseconds deltaTime{};
};

class GyroscopeOrientationIntegrator
{
public:
    explicit GyroscopeOrientationIntegrator(OrientationIntegratorConfig configuration = {});

    [[nodiscard]] OrientationIntegratorResult update(
        const AngularVelocityRadians& sensorAngularVelocity,
        std::uint64_t deviceTimestamp) noexcept;
    void reset() noexcept;
    void clearTimestamp() noexcept;
    [[nodiscard]] bool setOrientation(const Quaternion& orientation) noexcept;
    [[nodiscard]] bool recenter() noexcept;
    void clearRecenter() noexcept;
    [[nodiscard]] Quaternion orientation() const noexcept;
    [[nodiscard]] Quaternion relativeOrientation() const noexcept;
    [[nodiscard]] const OrientationState& state() const noexcept;
    [[nodiscard]] const OrientationIntegratorConfig& configuration() const noexcept;

private:
    [[nodiscard]] OrientationIntegratorResult result(
        OrientationSampleStatus status,
        OrientationRejectionReason reason,
        std::chrono::nanoseconds delta) const noexcept;

    OrientationIntegratorConfig configuration_;
    OrientationState state_;
    Quaternion recenterReference_;
};

[[nodiscard]] AngularVelocityRadians mapAngularVelocity(
    const AngularVelocityRadians& sensorAngularVelocity,
    const GyroscopeAxisMapping& mapping) noexcept;
[[nodiscard]] EulerAnglesDiagnostic quaternionToEulerDiagnostic(
    const Quaternion& orientation) noexcept;
[[nodiscard]] std::string serializeGyroscopeOrientationJson(
    const GyroscopeOrientationIntegrator& integrator,
    const GyroscopeScaleProfile& scaleProfile,
    bool includeEuler);
[[nodiscard]] std::string orientationRejectionReasonText(OrientationRejectionReason reason);

} // namespace xreal::sensors
