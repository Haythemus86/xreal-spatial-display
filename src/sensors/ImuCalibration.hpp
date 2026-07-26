#pragma once

#include "sensors/ImuSample.hpp"

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace xreal::sensors
{

struct ScalarStatistics
{
    std::uint64_t sampleCount{};
    double mean{};
    double minimum{};
    double maximum{};
    double standardDeviation{};
};

class RunningStatistics
{
public:
    void consume(double value) noexcept;
    [[nodiscard]] ScalarStatistics result() const noexcept;

private:
    std::uint64_t sampleCount_{};
    double mean_{};
    double sumSquaredDifferences_{};
    double minimum_{};
    double maximum_{};
};

struct RawVector3d
{
    double x{};
    double y{};
    double z{};
};

struct VectorStatistics
{
    ScalarStatistics x;
    ScalarStatistics y;
    ScalarStatistics z;
};

struct TimestampDeltaStatistics
{
    ScalarStatistics deviceNanoseconds;
    ScalarStatistics hostNanoseconds;
};

struct GyroBiasEstimate
{
    bool accepted{};
    RawVector3d mean;
    RawVector3d standardDeviation;
    std::uint64_t sampleCount{};
    double movementThresholdRaw{};
};

class GyroBiasEstimator
{
public:
    explicit GyroBiasEstimator(double movementThresholdRaw);

    void consume(const RawImuVector3& gyroscopeRaw) noexcept;
    [[nodiscard]] std::optional<GyroBiasEstimate> result() const noexcept;

private:
    double movementThresholdRaw_{};
    RunningStatistics x_;
    RunningStatistics y_;
    RunningStatistics z_;
};

struct StationaryCalibrationResult
{
    bool accepted{};
    RawVector3d gyroBias;
    RawVector3d gyroStandardDeviation;
    RawVector3d accelMean;
    double accelVectorMagnitudeRaw{};
    std::uint64_t sampleCount{};
    std::chrono::nanoseconds captureDuration{};
};

struct ImuCalibrationStatistics
{
    VectorStatistics gyroscopeRaw;
    VectorStatistics accelerometerRaw;
    TimestampDeltaStatistics timestampDeltas;
    std::uint64_t sampleCount{};
    std::chrono::nanoseconds captureDuration{};
};

class ImuCalibrationAccumulator
{
public:
    explicit ImuCalibrationAccumulator(double gyroMovementThresholdRaw);

    void consume(const ImuSample& sample) noexcept;
    [[nodiscard]] ImuCalibrationStatistics statistics() const noexcept;
    [[nodiscard]] std::optional<StationaryCalibrationResult> stationaryResult() const noexcept;

private:
    RunningStatistics gyroX_;
    RunningStatistics gyroY_;
    RunningStatistics gyroZ_;
    RunningStatistics accelX_;
    RunningStatistics accelY_;
    RunningStatistics accelZ_;
    RunningStatistics deviceTimestampDeltas_;
    RunningStatistics hostTimestampDeltas_;
    GyroBiasEstimator gyroBiasEstimator_;
    std::optional<std::uint64_t> previousDeviceTimestamp_;
    std::optional<std::chrono::steady_clock::time_point> previousHostTimestamp_;
    std::optional<std::chrono::steady_clock::time_point> firstHostTimestamp_;
    std::optional<std::chrono::steady_clock::time_point> lastHostTimestamp_;
};

struct ImuCalibrationReport
{
    std::string name;
    ImuCalibrationStatistics statistics;
    StationaryCalibrationResult stationary;
    double packetRate{};
    std::uint64_t sequenceGaps{};
    std::uint64_t outOfSequenceEvents{};
    std::uint64_t malformedPackets{};
    std::uint64_t queueDrops{};
};

[[nodiscard]] std::string serializeCalibrationReportJson(const ImuCalibrationReport& report);

} // namespace xreal::sensors
