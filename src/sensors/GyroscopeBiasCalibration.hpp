#pragma once

#include "sensors/ImuCalibration.hpp"
#include "sensors/ImuSample.hpp"

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

namespace xreal::sensors
{

struct GyroscopeBias
{
    double x{};
    double y{};
    double z{};
};

struct CorrectedGyroscopeRaw
{
    double x{};
    double y{};
    double z{};
};

struct PacketRateRange
{
    double minimumPacketsPerSecond{};
    double maximumPacketsPerSecond{};
};

struct GyroscopeBiasCalibrationConfig
{
    std::chrono::nanoseconds calibrationDuration = std::chrono::seconds(2);
    std::uint64_t minimumSampleCount = 1500;
    double maximumStandardDeviationRaw = 750.0;
    double maximumRangeRaw = 5000.0;
    std::optional<PacketRateRange> acceptablePacketRate = PacketRateRange{800.0, 1200.0};
    std::uint64_t warmupSampleCount = 100;
    std::optional<std::chrono::nanoseconds> warmupDuration;
};

enum class GyroscopeBiasCalibrationRejectionReason
{
    none,
    insufficientSamples,
    insufficientDuration,
    invalidDeviceTimestamp,
    excessiveStandardDeviation,
    excessiveRange,
    packetRateOutOfRange,
};

struct GyroscopeBiasCalibrationStatistics
{
    VectorStatistics gyroscopeRaw;
    RawVector3d rangeRaw;
    ScalarStatistics deviceTimestampDeltaNanoseconds;
    std::uint64_t sampleCount{};
    std::chrono::nanoseconds captureDuration{};
    double packetRate{};
};

struct GyroscopeBiasCalibrationResult
{
    bool accepted{};
    GyroscopeBiasCalibrationRejectionReason rejectionReason{
        GyroscopeBiasCalibrationRejectionReason::none};
    std::chrono::nanoseconds requestedDuration{};
    GyroscopeBiasCalibrationConfig configuration;
    GyroscopeBiasCalibrationStatistics statistics;
    std::optional<GyroscopeBias> biasRaw;
};

struct GyroscopeBiasCalibrationDevice
{
    std::uint16_t vendorId{};
    std::uint16_t productId{};
    int interfaceNumber{};
    std::string productName;
};

class GyroscopeBiasCalibrator
{
public:
    explicit GyroscopeBiasCalibrator(GyroscopeBiasCalibrationConfig configuration);

    void consume(const ImuSample& sample) noexcept;
    [[nodiscard]] bool isComplete() const noexcept;
    [[nodiscard]] std::optional<GyroscopeBiasCalibrationResult> result() const noexcept;
    [[nodiscard]] GyroscopeBiasCalibrationResult finish() noexcept;

private:
    void reject(GyroscopeBiasCalibrationRejectionReason reason) noexcept;
    void evaluateCompletedCapture() noexcept;
    [[nodiscard]] GyroscopeBiasCalibrationStatistics statistics() const noexcept;

    GyroscopeBiasCalibrationConfig configuration_;
    RunningStatistics x_;
    RunningStatistics y_;
    RunningStatistics z_;
    RunningStatistics deviceTimestampDeltas_;
    std::uint64_t observedSampleCount_{};
    std::uint64_t captureSampleCount_{};
    std::uint64_t accumulatedWarmupNanoseconds_{};
    std::uint64_t accumulatedCaptureNanoseconds_{};
    std::optional<std::uint64_t> previousDeviceTimestamp_;
    std::optional<GyroscopeBiasCalibrationResult> result_;
};

[[nodiscard]] CorrectedGyroscopeRaw applyGyroscopeBias(
    const RawImuVector3& raw,
    const GyroscopeBias& bias) noexcept;

[[nodiscard]] std::string gyroscopeBiasCalibrationRejectionReasonText(
    GyroscopeBiasCalibrationRejectionReason reason);

[[nodiscard]] std::string serializeGyroscopeBiasCalibrationJson(
    const GyroscopeBiasCalibrationDevice& device,
    const GyroscopeBiasCalibrationResult& result);

} // namespace xreal::sensors
