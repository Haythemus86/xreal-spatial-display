#include "diagnostics/ImuDiagnosticOptions.hpp"
#include "sensors/XrealDevice.hpp"
#include "sensors/GyroscopeBiasCalibration.hpp"
#include "sensors/GyroscopePhysicalUnits.hpp"
#include "sensors/GyroscopeOrientation.hpp"
#include "sensors/AccelerometerPhysicalUnits.hpp"
#include "sensors/OrientationFusion.hpp"
#include "sensors/OrientationComparison.hpp"
#include "sensors/LiveOrientationPrediction.hpp"
#include "sensors/DeviceTimestampDelta.hpp"
#include "sensors/ImuCalibration.hpp"
#include "sensors/XrealImuStream.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <memory>
#include <mutex>
#include <numbers>
#include <optional>
#include <span>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace
{

template <typename Value, std::size_t Capacity>
class SampleQueue
{
public:
    [[nodiscard]] bool tryPush(const Value& value) noexcept
    {
        const std::size_t write = writeIndex_.load(std::memory_order_relaxed);
        const std::size_t next = (write + 1U) % Capacity;
        if (next == readIndex_.load(std::memory_order_acquire))
        {
            return false;
        }

        values_[write] = value;
        writeIndex_.store(next, std::memory_order_release);
        return true;
    }

    [[nodiscard]] bool tryPop(Value& value) noexcept
    {
        const std::size_t read = readIndex_.load(std::memory_order_relaxed);
        if (read == writeIndex_.load(std::memory_order_acquire))
        {
            return false;
        }

        value = values_[read];
        readIndex_.store((read + 1U) % Capacity, std::memory_order_release);
        return true;
    }

private:
    static_assert(Capacity > 1);
    std::array<Value, Capacity> values_{};
    std::atomic_size_t readIndex_{};
    std::atomic_size_t writeIndex_{};
};

void printUsage()
{
    std::cout << xreal::diagnostics::imuDiagnosticUsage();
}

std::wstring maskedSerial(const std::wstring& serial)
{
    if (serial.empty())
    {
        return L"unavailable";
    }

    return L"********";
}

void writeCsvHeader(std::ofstream& output)
{
    output << "host_timestamp_ns,device_timestamp,sequence,gyro_raw_x,gyro_raw_y,gyro_raw_z,"
              "accel_raw_x,accel_raw_y,accel_raw_z,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z\n";
}

void writeCsvSample(std::ofstream& output, const xreal::sensors::ImuSample& sample)
{
    const auto hostNanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(
        sample.hostReceiveTimestamp.time_since_epoch()).count();

    output << hostNanoseconds << ','
           << sample.deviceTimestamp.nanoseconds << ','
           << static_cast<unsigned int>(sample.packetSequence) << ','
           << sample.gyroscopeRaw.x << ',' << sample.gyroscopeRaw.y << ',' << sample.gyroscopeRaw.z << ','
           << sample.accelerometerRaw.x << ',' << sample.accelerometerRaw.y << ','
           << sample.accelerometerRaw.z;

    if (sample.gyroscopeRadiansPerSecond.has_value())
    {
        const auto& gyro = *sample.gyroscopeRadiansPerSecond;
        output << ',' << gyro.x << ',' << gyro.y << ',' << gyro.z;
    }
    else
    {
        output << ",,,";
    }

    if (sample.accelerometerMetersPerSecondSquared.has_value())
    {
        const auto& acceleration = *sample.accelerometerMetersPerSecondSquared;
        output << ',' << acceleration.x << ',' << acceleration.y << ',' << acceleration.z;
    }
    else
    {
        output << ",,,";
    }

    output << '\n';
}

void printSample(
    const xreal::sensors::ImuSample& sample,
    std::optional<std::uint64_t> deviceTimestampDelta,
    std::optional<std::int64_t> hostTimestampDelta,
    const std::optional<xreal::sensors::GyroscopeBias>& gyroscopeBias,
    const std::optional<xreal::sensors::GyroscopeScaleProfile>& gyroscopeScale,
    bool printDegrees,
    bool printRadians)
{
    std::cout << "seq=" << static_cast<unsigned int>(sample.packetSequence)
              << " device_ns=" << sample.deviceTimestamp.nanoseconds
              << " gyro_raw=[" << sample.gyroscopeRaw.x << ", " << sample.gyroscopeRaw.y
              << ", " << sample.gyroscopeRaw.z << ']'
              << " accel_raw=[" << sample.accelerometerRaw.x << ", " << sample.accelerometerRaw.y
              << ", " << sample.accelerometerRaw.z << ']';

    if (gyroscopeBias.has_value())
    {
        const auto corrected = xreal::sensors::applyGyroscopeBias(
            sample.gyroscopeRaw,
            *gyroscopeBias);
        std::cout << " gyro_corrected_raw_units=[" << corrected.x << ", "
                  << corrected.y << ", " << corrected.z << ']';
        if (gyroscopeScale.has_value() && (printDegrees || printRadians))
        {
            const auto physical = xreal::sensors::convertGyroscopeToPhysicalUnits(
                sample.gyroscopeRaw, *gyroscopeBias, *gyroscopeScale);
            if (printDegrees && physical.x.valid && physical.y.valid && physical.z.valid)
            {
                std::cout << " gyro_degrees_per_second=[" << physical.x.degreesPerSecond
                          << ", " << physical.y.degreesPerSecond << ", "
                          << physical.z.degreesPerSecond << ']';
            }
            if (printRadians && physical.x.valid && physical.y.valid && physical.z.valid)
            {
                std::cout << " gyro_radians_per_second=[" << physical.x.radiansPerSecond
                          << ", " << physical.y.radiansPerSecond << ", "
                          << physical.z.radiansPerSecond << ']';
            }
        }
    }

    if (deviceTimestampDelta.has_value() && hostTimestampDelta.has_value())
    {
        std::cout << " delta_device_ns=" << *deviceTimestampDelta
                  << " delta_host_ns=" << *hostTimestampDelta;
    }

    if (sample.gyroscopeRadiansPerSecond.has_value()
        && sample.accelerometerMetersPerSecondSquared.has_value())
    {
        const auto& gyro = *sample.gyroscopeRadiansPerSecond;
        const auto& acceleration = *sample.accelerometerMetersPerSecondSquared;
        std::cout << " gyro_rad_s=[" << gyro.x << ", " << gyro.y << ", " << gyro.z << ']'
                  << " accel_m_s2=[" << acceleration.x << ", " << acceleration.y << ", "
                  << acceleration.z << ']';
    }

    std::cout << '\n';
}

void printScalarStatistics(std::string_view label, const xreal::sensors::ScalarStatistics& statistics)
{
    std::cout << "  " << label
              << ": mean=" << statistics.mean
              << " min=" << statistics.minimum
              << " max=" << statistics.maximum
              << " stddev=" << statistics.standardDeviation
              << " count=" << statistics.sampleCount << '\n';
}

void printVectorStatistics(
    std::string_view label,
    const xreal::sensors::VectorStatistics& statistics)
{
    printScalarStatistics(std::string(label) + "_x", statistics.x);
    printScalarStatistics(std::string(label) + "_y", statistics.y);
    printScalarStatistics(std::string(label) + "_z", statistics.z);
}

constexpr double stationaryGyroMovementThresholdRaw = 5000.0;

[[nodiscard]] std::string narrowAscii(std::wstring_view value)
{
    std::string result;
    result.reserve(value.size());
    for (const wchar_t character : value)
    {
        result.push_back(character >= 0 && character <= 0x7F
            ? static_cast<char>(character)
            : '?');
    }
    return result;
}

void printOrientation(
    const xreal::sensors::GyroscopeOrientationIntegrator& integrator,
    xreal::diagnostics::OrientationOutputMode outputMode)
{
    const auto orientation = integrator.relativeOrientation();
    std::cout << "gyro_orientation";
    if (outputMode != xreal::diagnostics::OrientationOutputMode::euler)
    {
        std::cout << " gyro_relative_wxyz=[" << orientation.w << ", " << orientation.x
                  << ", " << orientation.y << ", " << orientation.z << ']';
    }
    if (outputMode != xreal::diagnostics::OrientationOutputMode::quaternion)
    {
        const auto euler = xreal::sensors::quaternionToEulerDiagnostic(orientation);
        std::cout << " gyro_relative_euler_zyx_degrees=[yaw=" << euler.yawDegrees
                  << ", pitch=" << euler.pitchDegrees
                  << ", roll=" << euler.rollDegrees << ']';
    }
    const auto& state = integrator.state();
    std::cout << " valid=" << (state.valid ? "yes" : "no")
              << " delta_ms="
              << std::chrono::duration<double, std::milli>(state.lastDeltaTime).count()
              << " applied=" << state.appliedSampleCount
              << " skipped=" << state.skippedSampleCount
              << " rejected=" << state.rejectedSampleCount << '\n';
}

void printFusion(
    const xreal::sensors::OrientationFusionFilter& filter,
    const xreal::sensors::OrientationFusionResult& result,
    const xreal::sensors::AccelerometerPhysicalSample& acceleration,
    xreal::diagnostics::OrientationOutputMode outputMode,
    bool printPhysical,
    bool printDiagnostics)
{
    const auto fused = filter.relativeOrientation();
    std::cout << "fusion";
    if (outputMode != xreal::diagnostics::OrientationOutputMode::euler)
    {
        std::cout << " fused_relative_wxyz=[" << fused.w << ", " << fused.x << ", "
                  << fused.y << ", " << fused.z << ']';
    }
    if (outputMode != xreal::diagnostics::OrientationOutputMode::quaternion)
    {
        const auto euler = xreal::sensors::quaternionToEulerDiagnostic(fused);
        std::cout << " fused_relative_euler_zyx_degrees=[yaw=" << euler.yawDegrees
                  << ", pitch=" << euler.pitchDegrees << ", roll=" << euler.rollDegrees << ']';
    }
    if (printPhysical)
    {
        std::cout << " accel_raw=[" << acceleration.raw.x << ", " << acceleration.raw.y
                  << ", " << acceleration.raw.z << ']'
                  << " accel_g=[" << acceleration.accelerationG.x << ", "
                  << acceleration.accelerationG.y << ", " << acceleration.accelerationG.z << ']'
                  << " accel_m_s2=[" << acceleration.accelerationMetersPerSecondSquared.x
                  << ", " << acceleration.accelerationMetersPerSecondSquared.y << ", "
                  << acceleration.accelerationMetersPerSecondSquared.z << ']'
                  << " accel_norm_g=" << acceleration.normG;
    }
    if (printDiagnostics)
    {
        constexpr double radiansToDegrees = 180.0 / std::numbers::pi;
        const auto& predicted = result.gyroscopePrediction;
        std::cout << " gyro_prediction_absolute_wxyz=[" << predicted.w << ", " << predicted.x
                  << ", " << predicted.y << ", " << predicted.z << ']'
                  << " confidence=" << result.gravity.confidence.correctionConfidence
                  << " correction_degrees="
                  << result.appliedCorrectionAngleRadians * radiansToDegrees
                  << " correction_status="
                  << (result.correctionStatus == xreal::sensors::FusionCorrectionStatus::applied
                          ? "applied" : result.correctionStatus
                              == xreal::sensors::FusionCorrectionStatus::rejected ? "rejected" : "skipped")
                  << " reason=" << xreal::sensors::fusionReasonText(result.reason);
    }
    const auto& state = filter.state();
    std::cout << " gyro_applied=" << state.appliedGyroscopeSampleCount
              << " accel_applied=" << state.appliedAccelerometerCorrectionCount
              << " accel_skipped=" << state.skippedAccelerometerCorrectionCount
              << " rejected=" << state.rejectedSampleCount << '\n';
}

void printQuaternion(std::string_view label, const xreal::sensors::Quaternion& value)
{
    std::cout << ' ' << label << "=[" << value.w << ", " << value.x << ", "
              << value.y << ", " << value.z << ']';
}

void printEuler(std::string_view label, const xreal::sensors::EulerAnglesDiagnostic& value)
{
    std::cout << ' ' << label << "=[yaw=" << value.yawDegrees
              << ", pitch=" << value.pitchDegrees << ", roll=" << value.rollDegrees << ']';
}

void printComparison(
    const xreal::sensors::OrientationComparisonRecord& record,
    xreal::diagnostics::OrientationOutputMode outputMode)
{
    const auto& orientations = record.orientations;
    std::cout << "comparison phase=" << xreal::sensors::experimentPhaseText(record.phase)
              << " stationary=" << (record.stationary.stationary ? "yes" : "no")
              << " stationary_reason="
              << xreal::sensors::stationaryReasonText(record.stationary.reason);
    if (outputMode != xreal::diagnostics::OrientationOutputMode::euler)
    {
        printQuaternion("gyro_absolute_wxyz", orientations.gyroAbsolute.value);
        printQuaternion("gyro_relative_wxyz", orientations.gyroRelative.value);
        printQuaternion("fused_absolute_wxyz", orientations.fusedAbsolute.value);
        printQuaternion("fused_relative_wxyz", orientations.fusedRelative.value);
        printQuaternion("gyro_recenter_reference_wxyz", orientations.gyroRecenterReference.value);
        printQuaternion("fused_recenter_reference_wxyz", orientations.fusedRecenterReference.value);
    }
    if (outputMode != xreal::diagnostics::OrientationOutputMode::quaternion)
    {
        printEuler("gyro_absolute_euler_zyx_degrees", orientations.gyroAbsoluteEuler);
        printEuler("gyro_relative_euler_zyx_degrees", orientations.gyroRelativeEuler);
        printEuler("fused_absolute_euler_zyx_degrees", orientations.fusedAbsoluteEuler);
        printEuler("fused_relative_euler_zyx_degrees", orientations.fusedRelativeEuler);
    }
    if (record.absoluteDifference.has_value() && record.relativeDifference.has_value())
    {
        std::cout << " gyro_vs_fused_absolute_error_degrees="
                  << record.absoluteDifference->total.degrees
                  << " gyro_vs_fused_relative_error_degrees="
                  << record.relativeDifference->total.degrees
                  << " tilt_error_degrees=" << record.absoluteDifference->tilt.degrees
                  << " yaw_difference_degrees=" << record.absoluteDifference->yawDegrees
                  << " pitch_difference_degrees=" << record.absoluteDifference->pitchDegrees
                  << " roll_difference_degrees=" << record.absoluteDifference->rollDegrees;
    }
    std::cout << " accelerometer_confidence=" << record.accelerometerConfidence
              << " correction_degrees=" << record.correctionAngleDegrees
              << " correction_status="
              << (record.correctionStatus == xreal::sensors::FusionCorrectionStatus::applied
                      ? "applied" : record.correctionStatus
                          == xreal::sensors::FusionCorrectionStatus::rejected ? "rejected" : "skipped")
              << " correction_reason="
              << xreal::sensors::fusionReasonText(record.correctionReason) << '\n';
}

void printPrediction(
    const xreal::sensors::OrientationPredictionRecord& record,
    xreal::diagnostics::OrientationOutputMode outputMode)
{
    const auto& prediction = record.prediction;
    std::cout << "prediction prediction_mode="
              << xreal::sensors::predictionModeText(record.mode)
              << " prediction_requested_horizon_ms=" << prediction.horizon.requested.count() * 1000.0
              << " prediction_applied_horizon_ms=" << prediction.horizon.applied.count() * 1000.0
              << " prediction_valid="
              << (prediction.validity == xreal::sensors::PredictionValidity::valid ? "yes" : "no")
              << " angular_velocity_raw_rad_s=["
              << prediction.angularVelocity.raw.xRadiansPerSecond << ','
              << prediction.angularVelocity.raw.yRadiansPerSecond << ','
              << prediction.angularVelocity.raw.zRadiansPerSecond << ']'
              << " angular_velocity_filtered_rad_s=["
              << prediction.angularVelocity.filtered.xRadiansPerSecond << ','
              << prediction.angularVelocity.filtered.yRadiansPerSecond << ','
              << prediction.angularVelocity.filtered.zRadiansPerSecond << ']';
    if (prediction.angularAcceleration.available)
    {
        std::cout << " angular_acceleration_raw_rad_s2=["
                  << prediction.angularAcceleration.raw.xRadiansPerSecondSquared << ','
                  << prediction.angularAcceleration.raw.yRadiansPerSecondSquared << ','
                  << prediction.angularAcceleration.raw.zRadiansPerSecondSquared << ']'
                  << " angular_acceleration_filtered_rad_s2=["
                  << prediction.angularAcceleration.filtered.xRadiansPerSecondSquared << ','
                  << prediction.angularAcceleration.filtered.yRadiansPerSecondSquared << ','
                  << prediction.angularAcceleration.filtered.zRadiansPerSecondSquared << ']';
    }
    else
    {
        std::cout << " angular_acceleration_raw_rad_s2=unavailable"
                     " angular_acceleration_filtered_rad_s2=unavailable";
    }
    std::cout << " angular_acceleration_used="
              << (prediction.angularAcceleration.used ? "yes" : "no");
    if (outputMode != xreal::diagnostics::OrientationOutputMode::euler)
    {
        printQuaternion("fused_measured_absolute_wxyz", record.measuredAbsolute.value);
        printQuaternion("fused_predicted_absolute_wxyz", prediction.absolute.value);
        printQuaternion("fused_measured_relative_wxyz", record.measuredRelative.value);
        printQuaternion("fused_predicted_relative_wxyz", prediction.relative.value);
    }
    if (outputMode != xreal::diagnostics::OrientationOutputMode::quaternion)
    {
        printEuler("fused_measured_absolute_euler_zyx_degrees",
            xreal::sensors::quaternionToEulerDiagnostic(record.measuredAbsolute.value));
        printEuler("fused_predicted_absolute_euler_zyx_degrees",
            xreal::sensors::quaternionToEulerDiagnostic(prediction.absolute.value));
        printEuler("fused_measured_relative_euler_zyx_degrees",
            xreal::sensors::quaternionToEulerDiagnostic(record.measuredRelative.value));
        printEuler("fused_predicted_relative_euler_zyx_degrees",
            xreal::sensors::quaternionToEulerDiagnostic(prediction.relative.value));
    }
    std::cout << " predicted_delta_angle_degrees="
              << prediction.diagnostics.predictedAngleRadians * (180.0 / std::numbers::pi)
              << " prediction_status="
              << xreal::sensors::predictionValidityText(prediction.validity)
              << " prediction_reason="
              << xreal::sensors::predictionRejectionReasonText(prediction.rejectionReason)
              << " prediction_fallback="
              << xreal::sensors::predictionFallbackReasonText(
                     prediction.diagnostics.fallbackReason)
              << " prediction_clamped="
              << (prediction.horizon.clamped || prediction.diagnostics.speedClamped
                     || prediction.diagnostics.accelerationClamped
                     || prediction.diagnostics.angleClamped ? "yes" : "no");
    if (record.delayedEvaluation.has_value())
    {
        const auto& evaluation = *record.delayedEvaluation;
        std::cout << " evaluated_target_device_timestamp_ns="
                  << evaluation.targetDeviceTimestampNanoseconds
                  << " target_timestamp_error_ns=" << evaluation.targetTimestampErrorNanoseconds
                  << " unpredicted_error_degrees=" << evaluation.baselineTotalErrorDegrees
                  << " predicted_error_degrees=" << evaluation.predictionTotalErrorDegrees
                  << " unpredicted_tilt_error_degrees=" << evaluation.baselineTiltErrorDegrees
                  << " predicted_tilt_error_degrees=" << evaluation.predictionTiltErrorDegrees
                  << " prediction_improvement_degrees=" << evaluation.totalImprovementDegrees
                  << " prediction_improvement_percent=";
        if (evaluation.totalImprovementPercent.has_value())
        {
            std::cout << *evaluation.totalImprovementPercent;
        }
        else
        {
            std::cout << "unavailable";
        }
    }
    else
    {
        std::cout << " delayed_evaluation=unavailable";
    }
    std::cout << '\n';
}

void printDrift(std::string_view label, const xreal::sensors::DriftMetrics& metrics)
{
    std::cout << "  " << label << ": ";
    if (!metrics.available)
    {
        std::cout << "unavailable (stationary_samples=" << metrics.stationarySampleCount
                  << ", ratio=" << metrics.stationarySampleRatio << ")\n";
        return;
    }
    std::cout << "total=" << metrics.totalDegrees << " deg, tilt=" << metrics.tiltDegrees
              << " deg, yaw=" << metrics.yawDegrees << " deg, pitch="
              << metrics.pitchDegrees << " deg, roll=" << metrics.rollDegrees
              << " deg, total_rate=" << metrics.totalDegreesPerSecond
              << " deg/s, stationary_ratio=" << metrics.stationarySampleRatio << '\n';
}

} // namespace

int main(int argc, char* argv[])
{
    std::vector<std::string_view> arguments;
    arguments.reserve(static_cast<std::size_t>(argc > 0 ? argc - 1 : 0));
    for (int index = 1; index < argc; ++index)
    {
        arguments.emplace_back(argv[index]);
    }

    const auto optionResult = xreal::diagnostics::parseImuDiagnosticOptions(arguments);
    if (optionResult.showHelp)
    {
        printUsage();
        return 0;
    }
    if (!optionResult.options.has_value())
    {
        std::cerr << "Error: " << optionResult.error << '\n';
        printUsage();
        return 2;
    }
    const xreal::diagnostics::ImuDiagnosticOptions& options = *optionResult.options;
    const bool runComparison = options.compareGyroscopeAndFusion;
    const bool runComplementaryFusion = options.fuseGyroscopeAccelerometer
        && options.fusionMode == xreal::diagnostics::FusionMode::complementary;
    const bool runGyroscopeOnlyOrientation = options.integrateGyroscopeOrientation
        || (options.fuseGyroscopeAccelerometer
            && options.fusionMode == xreal::diagnostics::FusionMode::gyroOnly);
    const auto selectedOrientationOutput = options.fuseGyroscopeAccelerometer
        ? options.fusionOutput
        : options.orientationOutput;
    const unsigned int selectedOrientationPrintRate = runComparison
        ? options.comparisonPrintRateHz
        : options.fuseGyroscopeAccelerometer
        ? options.fusionPrintRateHz
        : options.orientationPrintRateHz;

    try
    {
        std::optional<xreal::sensors::GyroscopeScaleProfile> gyroscopeScale;
        if (options.gyroscopeScaleProfilePath.has_value())
        {
            std::ifstream profileInput(*options.gyroscopeScaleProfilePath);
            if (!profileInput)
            {
                std::cerr << "Failed to open gyroscope scale profile: "
                          << *options.gyroscopeScaleProfilePath << '\n';
                return 1;
            }
            std::ostringstream profileJson;
            profileJson << profileInput.rdbuf();
            const auto loaded = xreal::sensors::loadGyroscopeScaleProfileJson(profileJson.str());
            if (!loaded.profile.has_value())
            {
                std::cerr << "Invalid gyroscope scale profile: " << loaded.error << '\n';
                return 1;
            }
            gyroscopeScale = *loaded.profile;
        }
        else if (options.gyroscopeScaleRawPerDegreePerSecond.has_value())
        {
            gyroscopeScale = xreal::sensors::makeExperimentalGyroscopeScaleProfile(
                *options.gyroscopeScaleRawPerDegreePerSecond);
        }
        if (runGyroscopeOnlyOrientation || runComplementaryFusion || runComparison)
        {
            const auto& profile = *gyroscopeScale;
            const bool allAxesUsable = profile.x.enabled && profile.x.valid
                && profile.y.enabled && profile.y.valid
                && profile.z.enabled && profile.z.valid;
            const auto validation = xreal::sensors::validateGyroscopeScaleProfile(profile);
            if (!validation.valid || !allAxesUsable)
            {
                std::cerr << "Gyroscope orientation requires valid, enabled X/Y/Z scale axes: "
                          << (validation.valid ? "one or more axes are disabled or invalid"
                                               : validation.explanation)
                          << '\n';
                return 1;
            }
        }

        std::optional<xreal::sensors::AccelerometerCalibrationProfile> accelerometerProfile;
        if (options.accelerometerProfilePath.has_value())
        {
            std::ifstream profileInput(*options.accelerometerProfilePath);
            if (!profileInput)
            {
                std::cerr << "Failed to open accelerometer calibration profile: "
                          << *options.accelerometerProfilePath << '\n';
                return 1;
            }
            std::ostringstream profileJson;
            profileJson << profileInput.rdbuf();
            const auto loaded = xreal::sensors::loadAccelerometerCalibrationProfileJson(
                profileJson.str(),
                *options.accelerometerProfilePath);
            if (!loaded.profile.has_value())
            {
                std::cerr << "Invalid accelerometer calibration profile: " << loaded.error << '\n';
                return 1;
            }
            accelerometerProfile = *loaded.profile;
        }
        if ((runComplementaryFusion || runComparison) && !accelerometerProfile.has_value())
        {
            std::cerr << "Complementary fusion requires an explicit accelerometer profile.\n";
            return 1;
        }

        const xreal::sensors::XrealDevice deviceEnumerator;
        const auto device = xreal::sensors::XrealImuStream::findInterface(deviceEnumerator.enumerate());
        if (!device.has_value())
        {
            std::cerr << "No exact XREAL Air 2 Ultra VID 0x3318/PID 0x0426/interface 2 was found.\n";
            return 1;
        }

        std::cout << "XREAL Air 2 Ultra IMU on VID 0x3318/PID 0x0426/interface 2\n";
        std::wcout << L"  Product: " << device->productName << L'\n'
                   << L"  Serial:  " << (options.verbose ? device->serialNumber : maskedSerial(device->serialNumber))
                   << L'\n';
        if (options.verbose)
        {
            std::cout << "  HID path: " << device->path << '\n';
        }
        if (gyroscopeScale.has_value())
        {
            std::cout << "  Gyroscope scale source: " << gyroscopeScale->source << '\n'
                      << "  Experimental: " << (gyroscopeScale->experimental ? "yes" : "no") << '\n'
                      << "  Verified: " << (gyroscopeScale->verified ? "yes" : "no") << '\n';
            const auto warning = xreal::sensors::gyroscopeScaleProvenanceWarning(
                *gyroscopeScale);
            if (warning.has_value())
            {
                std::cout << "  WARNING: " << *warning << '\n';
            }
            if (options.compareQ12Scale)
            {
                constexpr double q12CandidateRawPerDegreePerSecond = 4096.0;
                const auto comparison = xreal::sensors::compareGyroscopeScales(
                    gyroscopeScale->x.rawUnitsPerDegreePerSecond,
                    q12CandidateRawPerDegreePerSecond);
                std::cout << "  Q12 comparison absolute scale difference: "
                          << comparison.absoluteScaleDifference << " raw/(degree/s)\n"
                          << "  Q12 comparison relative difference: "
                          << comparison.relativeScaleDifferencePercent << "%\n"
                          << "  Closeness to Q12 is diagnostic evidence only, not documentary proof.\n";
            }
        }
        else
        {
            std::cout << "  Gyroscope physical-unit conversion: disabled (no explicit scale).\n";
        }
        if (runGyroscopeOnlyOrientation)
        {
            std::cout << "\nWARNING: gyro-only orientation is experimental and will drift.\n"
                      << "  It uses calibrated angular velocity and device timestamps only.\n"
                      << "  Axis mapping source: " << gyroscopeScale->axisMapping.source << '\n'
                      << "  Axis mapping verified: "
                      << (gyroscopeScale->axisMapping.verified ? "yes" : "no") << '\n'
                      << "  Maximum accepted device timestamp delta: "
                      << std::chrono::duration<double, std::milli>(
                             options.orientationMaximumDelta).count()
                      << " ms\n";
            if (gyroscopeScale->axisMapping.experimental
                || !gyroscopeScale->axisMapping.verified)
            {
                std::cout << "  WARNING: orientation axis mapping is experimental or unverified.\n";
            }
        }
        if (runComplementaryFusion)
        {
            std::cout << "\nWARNING: gyro/accelerometer complementary fusion is experimental.\n"
                      << "  Accelerometer profile source: " << accelerometerProfile->source << '\n'
                      << "  Accelerometer mapping source: "
                      << accelerometerProfile->axisMapping.source << '\n'
                      << "  Correction time constant: "
                      << options.accelerometerCorrectionTimeConstantSeconds << " seconds\n"
                      << "  Maximum correction rate: "
                      << options.accelerometerMaximumCorrectionDegreesPerSecond
                      << " degrees/second\n"
                      << "  Confidence deviations: full="
                      << options.accelerometerFullConfidenceDeviationG << " g, zero="
                      << options.accelerometerZeroConfidenceDeviationG << " g\n";
            if (accelerometerProfile->experimental || !accelerometerProfile->verified)
            {
                std::cout << "  WARNING: accelerometer calibration or axis mapping is experimental"
                             " or unverified.\n";
            }
        }
        if (runComparison)
        {
            std::cout << "\nWARNING: parallel gyro/fusion orientation comparison is experimental.\n"
                      << "  Both paths consume the same calibrated gyroscope sample and device timestamp.\n"
                      << "  Only the fused path receives accelerometer gravity correction.\n"
                      << "  Absolute and relative frames are reported separately; yaw is not corrected.\n"
                      << "  Gyroscope scale source: " << gyroscopeScale->source << '\n'
                      << "  Accelerometer profile source: " << accelerometerProfile->source << '\n';
        }
        if (options.calibrationName.has_value())
        {
            std::cout << "  Calibration: " << *options.calibrationName << '\n'
                      << "  Raw CSV: " << *options.csvPath << '\n'
                      << "  Stationary gyro stddev threshold: "
                      << stationaryGyroMovementThresholdRaw << " raw units per axis\n";
        }
        if (options.gyroscopeCalibration.has_value())
        {
            const auto& configuration = *options.gyroscopeCalibration;
            std::cout << "\nGyroscope startup bias calibration requested.\n"
                      << "  Keep the glasses completely still until calibration completes.\n"
                      << "  Calibration duration: "
                      << std::chrono::duration<double>(configuration.calibrationDuration).count()
                      << " seconds (device time)\n"
                      << "  Warm-up samples: " << configuration.warmupSampleCount << '\n'
                      << "  Warm-up duration: ";
            if (configuration.warmupDuration.has_value())
            {
                std::cout << std::chrono::duration<double>(*configuration.warmupDuration).count()
                          << " seconds (device time)\n";
            }
            else
            {
                std::cout << "not configured\n";
            }
            std::cout << "  Maximum stddev: " << configuration.maximumStandardDeviationRaw
                      << " raw units per axis\n"
                      << "  Maximum range: " << configuration.maximumRangeRaw
                      << " raw units per axis\n";
        }
        if (options.predictOrientation)
        {
            std::cout << "\nWARNING: orientation pose prediction is experimental.\n"
                      << "  It predicts fused orientation only; gyro scale and axis mappings remain unverified.\n"
                      << "  Mode: " << xreal::sensors::predictionModeText(options.predictionMode)
                      << "\n  Requested horizon: " << options.predictionHorizonMilliseconds
                      << " ms\n  Delayed evaluation: "
                      << (options.predictionEvaluateDelayed ? "enabled" : "disabled") << '\n';
        }

        std::ofstream csv;
        if (options.csvPath.has_value())
        {
            csv.open(*options.csvPath, std::ios::out | std::ios::trunc);
            if (!csv)
            {
                std::cerr << "Failed to open CSV output file: " << *options.csvPath << '\n';
                return 1;
            }
            writeCsvHeader(csv);
        }
        std::ofstream comparisonCsv;
        if (options.comparisonCsvOutputPath.has_value())
        {
            comparisonCsv.imbue(std::locale::classic());
            comparisonCsv.open(*options.comparisonCsvOutputPath, std::ios::out | std::ios::trunc);
            if (!comparisonCsv)
            {
                std::cerr << "Failed to open comparison CSV output file: "
                          << *options.comparisonCsvOutputPath << '\n';
                return 1;
            }
            comparisonCsv << xreal::sensors::orientationComparisonCsvHeader() << '\n';
        }
        std::ofstream predictionCsv;
        if (options.predictionCsvOutputPath.has_value())
        {
            predictionCsv.imbue(std::locale::classic());
            predictionCsv.open(*options.predictionCsvOutputPath, std::ios::out | std::ios::trunc);
            if (!predictionCsv)
            {
                std::cerr << "Failed to open prediction CSV output file: "
                          << *options.predictionCsvOutputPath << '\n';
                return 1;
            }
            predictionCsv << xreal::sensors::orientationPredictionCsvHeader() << '\n';
        }

        auto samples = std::make_unique<SampleQueue<xreal::sensors::ImuSample, 8192>>();
        std::atomic_uint64_t diagnosticQueueDrops{};
        std::optional<xreal::sensors::ImuCalibrationAccumulator> calibrationAccumulator;
        if (options.calibrationName.has_value())
        {
            calibrationAccumulator.emplace(stationaryGyroMovementThresholdRaw);
        }
        std::optional<xreal::sensors::GyroscopeBiasCalibrator> gyroscopeCalibrator;
        if (options.gyroscopeCalibration.has_value())
        {
            gyroscopeCalibrator.emplace(*options.gyroscopeCalibration);
        }
        std::optional<xreal::sensors::GyroscopeBias> acceptedGyroscopeBias;
        std::optional<xreal::sensors::GyroscopeOrientationIntegrator> orientationIntegrator;
        if (runGyroscopeOnlyOrientation)
        {
            xreal::sensors::OrientationIntegratorConfig configuration;
            configuration.maximumDeviceTimestampDelta = options.orientationMaximumDelta;
            configuration.axisMapping = gyroscopeScale->axisMapping;
            orientationIntegrator.emplace(configuration);
        }
        std::optional<std::chrono::steady_clock::time_point> orientationStartTime;
        bool orientationRecentered{};
        std::optional<xreal::sensors::OrientationFusionFilter> fusionFilter;
        if (runComplementaryFusion)
        {
            xreal::sensors::OrientationFusionConfig configuration;
            configuration.maximumDeviceTimestampDelta = options.orientationMaximumDelta;
            configuration.gyroscopeAxisMapping = gyroscopeScale->axisMapping;
            configuration.correctionTimeConstant = std::chrono::duration<double>(
                options.accelerometerCorrectionTimeConstantSeconds);
            configuration.maximumCorrectionDegreesPerSecond =
                options.accelerometerMaximumCorrectionDegreesPerSecond;
            configuration.confidence.fullConfidenceDeviationG =
                options.accelerometerFullConfidenceDeviationG;
            configuration.confidence.zeroConfidenceDeviationG =
                options.accelerometerZeroConfidenceDeviationG;
            configuration.confidence.smoothingTimeConstant = std::chrono::duration<double>(
                options.accelerometerConfidenceSmoothingSeconds);
            configuration.startupMode = options.fusionStartup
                    == xreal::diagnostics::FusionStartupMode::gravity
                ? xreal::sensors::FusionStartupMode::gravity
                : xreal::sensors::FusionStartupMode::identity;
            fusionFilter.emplace(configuration);
        }
        std::optional<xreal::sensors::OrientationFusionResult> latestFusionResult;
        std::optional<xreal::sensors::AccelerometerPhysicalSample> latestPhysicalAcceleration;
        std::optional<std::chrono::steady_clock::time_point> fusionStartTime;
        bool fusionRecentered{};
        xreal::sensors::OrientationExperimentConfig experimentConfiguration;
        experimentConfiguration.stationaryBefore = std::chrono::duration<double>(
            options.experimentStationaryBeforeSeconds);
        experimentConfiguration.motion = std::chrono::duration<double>(
            options.experimentMotionSeconds);
        experimentConfiguration.stationaryAfter = std::chrono::duration<double>(
            options.experimentStationaryAfterSeconds);
        experimentConfiguration.recenterAt = std::chrono::duration<double>(
            options.experimentRecenterSeconds);
        experimentConfiguration.convergenceThresholdsDegrees =
            options.convergenceThresholdsDegrees;
        experimentConfiguration.convergenceSustain = std::chrono::duration<double>(
            options.convergenceSustainSeconds);

        xreal::sensors::StationaryDetectorConfig stationaryConfiguration;
        stationaryConfiguration.maximumGyroscopeDegreesPerSecond =
            options.stationaryGyroscopeThresholdDegreesPerSecond;
        stationaryConfiguration.maximumAccelerationDeviationG =
            options.stationaryAccelerometerDeviationG;
        stationaryConfiguration.minimumDuration = std::chrono::duration<double>(
            options.stationaryMinimumDurationSeconds);

        std::optional<xreal::sensors::OrientationComparisonEngine> comparisonEngine;
        std::optional<xreal::sensors::StationaryDetector> stationaryDetector;
        if (runComparison)
        {
            xreal::sensors::OrientationComparisonConfig configuration;
            configuration.gyroscope.maximumDeviceTimestampDelta = options.orientationMaximumDelta;
            configuration.gyroscope.axisMapping = gyroscopeScale->axisMapping;
            configuration.fusion.maximumDeviceTimestampDelta = options.orientationMaximumDelta;
            configuration.fusion.gyroscopeAxisMapping = gyroscopeScale->axisMapping;
            configuration.fusion.correctionTimeConstant = std::chrono::duration<double>(
                options.accelerometerCorrectionTimeConstantSeconds);
            configuration.fusion.maximumCorrectionDegreesPerSecond =
                options.accelerometerMaximumCorrectionDegreesPerSecond;
            configuration.fusion.confidence.fullConfidenceDeviationG =
                options.accelerometerFullConfidenceDeviationG;
            configuration.fusion.confidence.zeroConfidenceDeviationG =
                options.accelerometerZeroConfidenceDeviationG;
            configuration.fusion.confidence.smoothingTimeConstant = std::chrono::duration<double>(
                options.accelerometerConfidenceSmoothingSeconds);
            configuration.fusion.startupMode = options.fusionStartup
                    == xreal::diagnostics::FusionStartupMode::gravity
                ? xreal::sensors::FusionStartupMode::gravity
                : xreal::sensors::FusionStartupMode::identity;
            comparisonEngine.emplace(configuration);
            stationaryDetector.emplace(stationaryConfiguration);
        }
        std::optional<std::uint64_t> comparisonStartDeviceTimestamp;
        std::optional<xreal::sensors::OrientationComparisonRecord> latestComparisonRecord;
        xreal::sensors::ExperimentPhase previousExperimentPhase{
            xreal::sensors::ExperimentPhase::startup};
        bool comparisonPhaseAnnounced{};
        bool comparisonRecentered{};
        std::optional<xreal::sensors::Quaternion> recoveryTarget;
        std::optional<xreal::sensors::TiltConvergenceTracker> gyroConvergence;
        std::optional<xreal::sensors::TiltConvergenceTracker> fusedConvergence;
        xreal::sensors::DriftAccumulator beforeGyroAbsolute;
        xreal::sensors::DriftAccumulator beforeGyroRelative;
        xreal::sensors::DriftAccumulator beforeFusedAbsolute;
        xreal::sensors::DriftAccumulator beforeFusedRelative;
        xreal::sensors::DriftAccumulator afterGyroAbsolute;
        xreal::sensors::DriftAccumulator afterGyroRelative;
        xreal::sensors::DriftAccumulator afterFusedAbsolute;
        xreal::sensors::DriftAccumulator afterFusedRelative;
        xreal::sensors::PhaseTimeRange stationaryBeforeTime;
        xreal::sensors::PhaseTimeRange motionTime;
        xreal::sensors::PhaseTimeRange stationaryAfterTime;
        std::optional<xreal::sensors::LiveOrientationPredictionSession> predictionSession;
        if (options.predictOrientation)
        {
            xreal::sensors::LiveOrientationPredictionConfig configuration;
            configuration.predictor.mode = options.predictionMode;
            configuration.predictor.horizon = std::chrono::duration<double>(
                options.predictionHorizonMilliseconds / 1000.0);
            configuration.predictor.maximumHorizon = std::chrono::duration<double>(
                options.predictionMaximumHorizonMilliseconds / 1000.0);
            if (options.predictionAngularVelocitySmoothingSeconds.has_value())
            {
                configuration.predictor.angularVelocitySmoothingTimeConstant =
                    std::chrono::duration<double>(
                        *options.predictionAngularVelocitySmoothingSeconds);
            }
            if (options.predictionAngularAccelerationSmoothingSeconds.has_value())
            {
                configuration.predictor.angularAccelerationSmoothingTimeConstant =
                    std::chrono::duration<double>(
                        *options.predictionAngularAccelerationSmoothingSeconds);
            }
            constexpr double degreesToRadians = std::numbers::pi / 180.0;
            configuration.predictor.maximumTimestampDelta = options.orientationMaximumDelta;
            configuration.predictor.maximumAngularSpeedRadiansPerSecond =
                options.predictionMaximumAngularSpeedDegreesPerSecond * degreesToRadians;
            configuration.predictor.maximumAngularAccelerationRadiansPerSecondSquared =
                options.predictionMaximumAngularAccelerationDegreesPerSecondSquared
                    * degreesToRadians;
            configuration.predictor.maximumPredictionAngleRadians =
                options.predictionMaximumAngleDegrees * degreesToRadians;
            configuration.predictor.limitBehavior = options.predictionLimitBehavior;
            configuration.evaluator.tolerance = std::chrono::nanoseconds(
                static_cast<std::int64_t>(
                    options.predictionEvaluationToleranceMilliseconds * 1'000'000.0));
            configuration.evaluateDelayed = options.predictionEvaluateDelayed;
            predictionSession.emplace(configuration);
        }
        std::optional<xreal::sensors::OrientationPredictionRecord> latestPredictionRecord;
        std::optional<std::uint64_t> predictionStartDeviceTimestamp;
        std::uint64_t predictionRecenterGeneration{};

        const auto consumeGyroscopeCalibration = [&](const xreal::sensors::ImuSample& sample) {
            if (!gyroscopeCalibrator.has_value() || gyroscopeCalibrator->isComplete())
            {
                return;
            }
            gyroscopeCalibrator->consume(sample);
            const auto currentResult = gyroscopeCalibrator->result();
            if (currentResult.has_value() && currentResult->accepted)
            {
                acceptedGyroscopeBias = currentResult->biasRaw;
            }
        };
        const auto consumeOrientation = [&](const xreal::sensors::ImuSample& sample) {
            if (!orientationIntegrator.has_value()
                || !acceptedGyroscopeBias.has_value()
                || !gyroscopeScale.has_value())
            {
                return;
            }
            const auto physical = xreal::sensors::convertGyroscopeToPhysicalUnits(
                sample.gyroscopeRaw,
                *acceptedGyroscopeBias,
                *gyroscopeScale);
            const xreal::sensors::AngularVelocityRadians angularVelocity{
                physical.x.radiansPerSecond,
                physical.y.radiansPerSecond,
                physical.z.radiansPerSecond,
            };
            const auto result = orientationIntegrator->update(
                angularVelocity,
                sample.deviceTimestamp.nanoseconds);
            if (!orientationStartTime.has_value()
                && result.reason == xreal::sensors::OrientationRejectionReason::firstTimestamp)
            {
                orientationStartTime = sample.hostReceiveTimestamp;
            }
        };
        const auto consumeFusion = [&](const xreal::sensors::ImuSample& sample) {
            if (!fusionFilter.has_value()
                || !acceptedGyroscopeBias.has_value()
                || !gyroscopeScale.has_value()
                || !accelerometerProfile.has_value())
            {
                return;
            }
            const auto gyroscope = xreal::sensors::convertGyroscopeToPhysicalUnits(
                sample.gyroscopeRaw,
                *acceptedGyroscopeBias,
                *gyroscopeScale);
            if (!gyroscope.x.valid || !gyroscope.y.valid || !gyroscope.z.valid)
            {
                return;
            }
            latestPhysicalAcceleration = xreal::sensors::convertAccelerometerToPhysicalUnits(
                sample.accelerometerRaw,
                *accelerometerProfile);
            const xreal::sensors::AngularVelocityRadians angularVelocity{
                gyroscope.x.radiansPerSecond,
                gyroscope.y.radiansPerSecond,
                gyroscope.z.radiansPerSecond,
            };
            latestFusionResult = fusionFilter->update(
                angularVelocity,
                *latestPhysicalAcceleration,
                sample.deviceTimestamp.nanoseconds);
            if (!fusionStartTime.has_value())
            {
                fusionStartTime = sample.hostReceiveTimestamp;
            }
        };
        const auto consumeComparison = [&](const xreal::sensors::ImuSample& sample) {
            if (!comparisonEngine.has_value() || !stationaryDetector.has_value()
                || !acceptedGyroscopeBias.has_value() || !gyroscopeScale.has_value()
                || !accelerometerProfile.has_value())
            {
                return;
            }
            const auto gyroscope = xreal::sensors::convertGyroscopeToPhysicalUnits(
                sample.gyroscopeRaw, *acceptedGyroscopeBias, *gyroscopeScale);
            if (!gyroscope.x.valid || !gyroscope.y.valid || !gyroscope.z.valid)
            {
                return;
            }
            const xreal::sensors::AngularVelocityRadians angularVelocity{
                gyroscope.x.radiansPerSecond,
                gyroscope.y.radiansPerSecond,
                gyroscope.z.radiansPerSecond,
            };
            const auto acceleration = xreal::sensors::convertAccelerometerToPhysicalUnits(
                sample.accelerometerRaw, *accelerometerProfile);
            const auto update = comparisonEngine->update(
                angularVelocity, acceleration, sample.deviceTimestamp.nanoseconds);
            if (!comparisonStartDeviceTimestamp.has_value())
            {
                comparisonStartDeviceTimestamp = sample.deviceTimestamp.nanoseconds;
                std::cout << "Orientation comparison startup complete; device-time experiment begins.\n";
            }
            const auto elapsedNanoseconds = xreal::sensors::forwardDeviceTimestampDelta(
                sample.deviceTimestamp.nanoseconds, *comparisonStartDeviceTimestamp);
            if (!elapsedNanoseconds.has_value())
            {
                return;
            }
            const auto elapsed = std::chrono::nanoseconds(*elapsedNanoseconds);
            const auto phase = options.orientationComparisonExperiment
                ? xreal::sensors::experimentPhaseAt(elapsed, experimentConfiguration)
                : xreal::sensors::ExperimentPhase::startup;
            if (!comparisonPhaseAnnounced || phase != previousExperimentPhase)
            {
                std::cout << "Orientation comparison phase: "
                          << xreal::sensors::experimentPhaseText(phase)
                          << " (device_ns=" << sample.deviceTimestamp.nanoseconds << ")\n";
                previousExperimentPhase = phase;
                comparisonPhaseAnnounced = true;
            }
            if (options.orientationComparisonExperiment && !comparisonRecentered
                && std::chrono::duration<double>(elapsed).count()
                    >= options.experimentRecenterSeconds)
            {
                comparisonRecentered = comparisonEngine->recenter();
                if (comparisonRecentered)
                {
                    ++predictionRecenterGeneration;
                    std::cout << "Gyro and fused relative orientations recentered together at "
                              << std::chrono::duration<double>(elapsed).count()
                              << " seconds of device time. Absolute states were preserved.\n";
                }
            }
            const auto snapshot = comparisonEngine->snapshot();
            const auto confidence = update.fusion.gravity.confidence.correctionConfidence;
            const auto stationary = stationaryDetector->update(
                angularVelocity, acceleration, confidence, sample.deviceTimestamp.nanoseconds);
            xreal::sensors::OrientationComparisonRecord record;
            record.deviceTimestampNanoseconds = sample.deviceTimestamp.nanoseconds;
            record.elapsedSeconds = std::chrono::duration<double>(elapsed).count();
            record.phase = phase;
            record.sequence = sample.packetSequence;
            record.stationary = stationary;
            record.gyroscopeRadiansPerSecond = angularVelocity;
            record.acceleration = acceleration;
            record.accelerometerConfidence = confidence;
            record.correctionStatus = update.fusion.correctionStatus;
            record.correctionReason = update.fusion.reason;
            record.correctionAngleDegrees = update.fusion.appliedCorrectionAngleRadians
                * (180.0 / std::numbers::pi);
            record.orientations = snapshot;
            record.absoluteDifference = update.absoluteDifference;
            record.relativeDifference = update.relativeDifference;
            latestComparisonRecord = record;

            const auto updatePhaseTime = [&](xreal::sensors::PhaseTimeRange& time) {
                if (!time.startDeviceTimestampNanoseconds.has_value())
                {
                    time.startDeviceTimestampNanoseconds = sample.deviceTimestamp.nanoseconds;
                }
                time.endDeviceTimestampNanoseconds = sample.deviceTimestamp.nanoseconds;
            };

            if (phase == xreal::sensors::ExperimentPhase::stationaryBefore)
            {
                updatePhaseTime(stationaryBeforeTime);
                beforeGyroAbsolute.consume(snapshot.gyroAbsolute.value, stationary.stationary,
                                           sample.deviceTimestamp.nanoseconds);
                beforeGyroRelative.consume(snapshot.gyroRelative.value, stationary.stationary,
                                           sample.deviceTimestamp.nanoseconds);
                beforeFusedAbsolute.consume(snapshot.fusedAbsolute.value, stationary.stationary,
                                            sample.deviceTimestamp.nanoseconds);
                beforeFusedRelative.consume(snapshot.fusedRelative.value, stationary.stationary,
                                            sample.deviceTimestamp.nanoseconds);
                if (stationary.stationary)
                {
                    recoveryTarget = snapshot.fusedAbsolute.value;
                }
            }
            else if (phase == xreal::sensors::ExperimentPhase::stationaryAfter)
            {
                updatePhaseTime(stationaryAfterTime);
                afterGyroAbsolute.consume(snapshot.gyroAbsolute.value, stationary.stationary,
                                          sample.deviceTimestamp.nanoseconds);
                afterGyroRelative.consume(snapshot.gyroRelative.value, stationary.stationary,
                                          sample.deviceTimestamp.nanoseconds);
                afterFusedAbsolute.consume(snapshot.fusedAbsolute.value, stationary.stationary,
                                           sample.deviceTimestamp.nanoseconds);
                afterFusedRelative.consume(snapshot.fusedRelative.value, stationary.stationary,
                                           sample.deviceTimestamp.nanoseconds);
                if (recoveryTarget.has_value() && !gyroConvergence.has_value())
                {
                    gyroConvergence.emplace(
                        *recoveryTarget,
                        experimentConfiguration.convergenceThresholdsDegrees,
                        experimentConfiguration.convergenceSustain);
                    fusedConvergence.emplace(
                        *recoveryTarget,
                        experimentConfiguration.convergenceThresholdsDegrees,
                        experimentConfiguration.convergenceSustain);
                }
                if (gyroConvergence.has_value())
                {
                    gyroConvergence->consume(
                        snapshot.gyroAbsolute.value, sample.deviceTimestamp.nanoseconds);
                    fusedConvergence->consume(
                        snapshot.fusedAbsolute.value, sample.deviceTimestamp.nanoseconds);
                }
            }
            else if (phase == xreal::sensors::ExperimentPhase::motion)
            {
                updatePhaseTime(motionTime);
            }
        };
        const auto consumePrediction = [&](const xreal::sensors::ImuSample& sample) {
            if (!predictionSession.has_value() || !acceptedGyroscopeBias.has_value()
                || !gyroscopeScale.has_value())
            {
                return;
            }
            const auto physical = xreal::sensors::convertGyroscopeToPhysicalUnits(
                sample.gyroscopeRaw, *acceptedGyroscopeBias, *gyroscopeScale);
            if (!physical.x.valid || !physical.y.valid || !physical.z.valid)
            {
                return;
            }
            const xreal::sensors::AngularVelocityRadians sensorAngularVelocity{
                physical.x.radiansPerSecond,
                physical.y.radiansPerSecond,
                physical.z.radiansPerSecond,
            };
            const auto bodyAngularVelocity = xreal::sensors::mapAngularVelocity(
                sensorAngularVelocity, gyroscopeScale->axisMapping);

            xreal::sensors::LiveOrientationPredictionInput input;
            input.bodyAngularVelocity = bodyAngularVelocity;
            input.deviceTimestampNanoseconds = sample.deviceTimestamp.nanoseconds;
            input.hostTimestampNanoseconds = static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    sample.hostReceiveTimestamp.time_since_epoch()).count());
            input.sequence = sample.packetSequence;
            input.recenterGeneration = predictionRecenterGeneration;
            if (comparisonEngine.has_value())
            {
                const auto snapshot = comparisonEngine->snapshot();
                input.fusedOrientationValid = comparisonEngine->fusion().state().valid;
                input.measuredAbsolute = {snapshot.fusedAbsolute.value};
                input.measuredRelative = {snapshot.fusedRelative.value};
                input.recenterReference = snapshot.fusedRecenterReference;
                if (latestComparisonRecord.has_value())
                {
                    input.phase = xreal::sensors::experimentPhaseText(
                        latestComparisonRecord->phase);
                }
            }
            else if (fusionFilter.has_value())
            {
                const auto& state = fusionFilter->state();
                input.fusedOrientationValid = state.valid && latestFusionResult.has_value();
                input.measuredAbsolute = {fusionFilter->orientation()};
                input.measuredRelative = {fusionFilter->relativeOrientation()};
                input.recenterReference = {state.recenterReference, state.recenterActive};
            }
            if (!predictionStartDeviceTimestamp.has_value())
            {
                predictionStartDeviceTimestamp = sample.deviceTimestamp.nanoseconds;
            }
            const auto elapsed = xreal::sensors::forwardDeviceTimestampDelta(
                sample.deviceTimestamp.nanoseconds, *predictionStartDeviceTimestamp);
            input.elapsedSeconds = elapsed.has_value()
                ? std::chrono::duration<double>(std::chrono::nanoseconds(*elapsed)).count()
                : 0.0;
            const auto record = predictionSession->consume(input);
            if (!record.has_value())
            {
                return;
            }
            latestPredictionRecord = *record;
            if (predictionCsv)
            {
                predictionCsv << xreal::sensors::serializeOrientationPredictionCsvRow(*record)
                              << '\n';
            }
        };
        xreal::sensors::XrealImuStream stream(*device);
        if (runComparison)
        {
            std::cout << "Orientation comparison phase: calibration/startup. "
                         "Parallel orientation updates begin only after gyro bias acceptance.\n";
        }
        if (!stream.start([&](std::span<const std::uint8_t, 64>, const xreal::sensors::ImuSample& sample) {
                if (!samples->tryPush(sample))
                {
                    ++diagnosticQueueDrops;
                }
            }))
        {
            std::wcerr << L"Failed to start IMU acquisition: " << stream.errorMessage() << L'\n';
            return 1;
        }

        const auto start = std::chrono::steady_clock::now();
        const auto deadline = start + options.duration;
        const auto printInterval = std::chrono::milliseconds(1000U / options.printRateHz);
        const auto orientationPrintInterval = std::chrono::milliseconds(
            1000U / selectedOrientationPrintRate);
        const auto predictionPrintInterval = std::chrono::milliseconds(
            1000U / options.predictionPrintRateHz);
        constexpr std::chrono::milliseconds processingInterval{20};
        auto nextPrint = start + printInterval;
        auto nextOrientationPrint = start + orientationPrintInterval;
        auto nextPredictionPrint = start + predictionPrintInterval;
        auto nextProcessing = start + processingInterval;
        std::optional<xreal::sensors::ImuSample> latestSample;
        std::optional<xreal::sensors::ImuSample> previousSample;
        std::optional<std::uint64_t> deviceDelta;
        std::optional<std::int64_t> hostDelta;
        std::mutex timerMutex;
        std::condition_variable timer;

        while (std::chrono::steady_clock::now() < deadline && stream.isRunning())
        {
            xreal::sensors::ImuSample sample;
            while (samples->tryPop(sample))
            {
                if (csv)
                {
                    writeCsvSample(csv, sample);
                }
                if (calibrationAccumulator.has_value())
                {
                    calibrationAccumulator->consume(sample);
                }
                consumeGyroscopeCalibration(sample);
                consumeOrientation(sample);
                consumeFusion(sample);
                consumeComparison(sample);
                consumePrediction(sample);

                if (previousSample.has_value())
                {
                    deviceDelta = sample.deviceTimestamp.nanoseconds
                        - previousSample->deviceTimestamp.nanoseconds;
                    hostDelta = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        sample.hostReceiveTimestamp - previousSample->hostReceiveTimestamp).count();
                }
                previousSample = sample;
                latestSample = sample;
            }

            const auto now = std::chrono::steady_clock::now();
            if (fusionFilter.has_value()
                && fusionStartTime.has_value()
                && options.recenterAfterSeconds.has_value()
                && !fusionRecentered
                && std::chrono::duration<double>(now - *fusionStartTime).count()
                    >= *options.recenterAfterSeconds)
            {
                fusionRecentered = fusionFilter->recenter();
                if (fusionRecentered)
                {
                    ++predictionRecenterGeneration;
                    std::cout << "Fused orientation recentered after "
                              << *options.recenterAfterSeconds << " seconds.\n";
                }
            }

            if (orientationIntegrator.has_value()
                && orientationStartTime.has_value()
                && options.recenterAfterSeconds.has_value()
                && !orientationRecentered
                && std::chrono::duration<double>(now - *orientationStartTime).count()
                    >= *options.recenterAfterSeconds)
            {
                orientationRecentered = orientationIntegrator->recenter();
                if (orientationRecentered)
                {
                    std::cout << "Orientation recentered after " << *options.recenterAfterSeconds
                              << " seconds.\n";
                }
            }
            if (now >= nextPrint)
            {
                if (latestSample.has_value())
                {
                    printSample(
                        *latestSample,
                        deviceDelta,
                        hostDelta,
                        options.applyGyroscopeBias ? acceptedGyroscopeBias : std::nullopt,
                        gyroscopeScale,
                        options.printGyroscopeDegrees,
                        options.printGyroscopeRadians);
                }
                nextPrint = now + printInterval;
            }
            if (orientationIntegrator.has_value()
                && orientationStartTime.has_value()
                && now >= nextOrientationPrint)
            {
                printOrientation(*orientationIntegrator, selectedOrientationOutput);
                nextOrientationPrint = now + orientationPrintInterval;
            }
            if (latestComparisonRecord.has_value() && now >= nextOrientationPrint)
            {
                printComparison(*latestComparisonRecord, options.comparisonOutput);
                if (comparisonCsv)
                {
                    comparisonCsv << xreal::sensors::serializeOrientationComparisonCsvRow(
                        *latestComparisonRecord) << '\n';
                }
                nextOrientationPrint = now + orientationPrintInterval;
            }
            if (fusionFilter.has_value()
                && latestFusionResult.has_value()
                && latestPhysicalAcceleration.has_value()
                && now >= nextOrientationPrint)
            {
                printFusion(
                    *fusionFilter,
                    *latestFusionResult,
                    *latestPhysicalAcceleration,
                    options.fusionOutput,
                    options.printAccelerometerPhysical,
                    options.printFusionDiagnostics);
                nextOrientationPrint = now + orientationPrintInterval;
            }
            if (latestPredictionRecord.has_value() && now >= nextPredictionPrint)
            {
                printPrediction(*latestPredictionRecord, options.predictionOutput);
                nextPredictionPrint = now + predictionPrintInterval;
            }
            nextProcessing = now + processingInterval;

            std::unique_lock lock(timerMutex);
            timer.wait_until(lock, std::min({
                nextPrint,
                nextOrientationPrint,
                nextPredictionPrint,
                nextProcessing,
                deadline,
            }));
        }

        stream.stop();
        xreal::sensors::ImuSample remainingSample;
        while (samples->tryPop(remainingSample))
        {
            if (csv)
            {
                writeCsvSample(csv, remainingSample);
            }
            if (calibrationAccumulator.has_value())
            {
                calibrationAccumulator->consume(remainingSample);
            }
            consumeGyroscopeCalibration(remainingSample);
            consumeOrientation(remainingSample);
            consumeFusion(remainingSample);
            consumeComparison(remainingSample);
            consumePrediction(remainingSample);
        }
        if (csv)
        {
            csv.flush();
            if (!csv)
            {
                std::cerr << "Failed while writing CSV output file: " << *options.csvPath << '\n';
                return 1;
            }
        }
        if (comparisonCsv)
        {
            comparisonCsv.flush();
            if (!comparisonCsv)
            {
                std::cerr << "Failed while writing comparison CSV output file: "
                          << *options.comparisonCsvOutputPath << '\n';
                return 1;
            }
        }
        if (predictionCsv)
        {
            predictionCsv.flush();
            if (!predictionCsv)
            {
                std::cerr << "Failed while writing prediction CSV output file: "
                          << *options.predictionCsvOutputPath << '\n';
                return 1;
            }
        }

        const auto elapsed = std::chrono::steady_clock::now() - start;
        const auto statistics = stream.statistics();
        const double seconds = std::chrono::duration<double>(elapsed).count();
        std::cout << "\nIMU acquisition summary\n"
                  << "  Effective packet rate: " << std::fixed << std::setprecision(2)
                  << (seconds > 0.0 ? static_cast<double>(statistics.received) / seconds : 0.0) << " packets/s\n"
                  << "  Received sensor packets: " << statistics.received << '\n'
                  << "  Activation responses: " << statistics.activationResponses << '\n'
                  << "  Estimated sequence gaps: " << statistics.dropped << '\n'
                  << "  Out-of-sequence events: " << statistics.outOfSequence << '\n'
                  << "  Malformed packets: " << statistics.invalid << '\n'
                  << "  Diagnostic queue drops: " << diagnosticQueueDrops.load() << '\n';

        if (orientationIntegrator.has_value())
        {
            const auto& orientationState = orientationIntegrator->state();
            std::cout << "\nGyroscope orientation summary\n"
                      << "  Applied samples: " << orientationState.appliedSampleCount << '\n'
                      << "  Skipped samples: " << orientationState.skippedSampleCount << '\n'
                      << "  Rejected samples: " << orientationState.rejectedSampleCount << '\n'
                      << "  Integrated device duration: "
                      << std::chrono::duration<double>(
                             orientationState.accumulatedIntegrationDuration).count()
                      << " seconds\n"
                      << "  Last rejection: "
                      << xreal::sensors::orientationRejectionReasonText(
                             orientationState.lastRejectionReason)
                      << '\n';
            printOrientation(*orientationIntegrator, selectedOrientationOutput);
        }

        if (fusionFilter.has_value())
        {
            const auto& fusionState = fusionFilter->state();
            std::cout << "\nGyroscope/accelerometer fusion summary\n"
                      << "  Gyroscope samples applied: "
                      << fusionState.appliedGyroscopeSampleCount << '\n'
                      << "  Accelerometer corrections applied: "
                      << fusionState.appliedAccelerometerCorrectionCount << '\n'
                      << "  Accelerometer corrections skipped: "
                      << fusionState.skippedAccelerometerCorrectionCount << '\n'
                      << "  Rejected samples: " << fusionState.rejectedSampleCount << '\n'
                      << "  Integrated device duration: "
                      << std::chrono::duration<double>(fusionState.integrationDuration).count()
                      << " seconds\n"
                      << "  Current acceleration norm: "
                      << fusionState.currentAccelerationNormG << " g\n"
                      << "  Current accelerometer confidence: "
                      << fusionState.currentAccelerometerConfidence << '\n'
                      << "  Accumulated correction angle: "
                      << fusionState.accumulatedCorrectionAngleRadians
                          * (180.0 / std::numbers::pi)
                      << " degrees\n"
                      << "  Last status: "
                      << xreal::sensors::fusionReasonText(fusionState.lastReason) << '\n';
            if (latestFusionResult.has_value() && latestPhysicalAcceleration.has_value())
            {
                printFusion(
                    *fusionFilter,
                    *latestFusionResult,
                    *latestPhysicalAcceleration,
                    options.fusionOutput,
                    options.printAccelerometerPhysical,
                    true);
            }
        }

        if (predictionSession.has_value())
        {
            predictionSession->finish();
            const auto& predictorStatistics = predictionSession->predictor().state().statistics;
            const auto& evaluationStatistics = predictionSession->evaluator().statistics();
            const std::uint64_t clampedCount = predictorStatistics.clampedHorizons
                + predictorStatistics.clampedSpeeds + predictorStatistics.clampedAccelerations
                + predictorStatistics.clampedAngles;
            std::cout << "\nOrientation pose prediction summary\n"
                      << "  Requests: " << predictorStatistics.requested << '\n'
                      << "  Applied: " << predictorStatistics.produced << '\n'
                      << "  Rejected: " << predictorStatistics.rejected << '\n'
                      << "  Clamping events: " << clampedCount << '\n'
                      << "  Constant-velocity predictions: "
                      << predictorStatistics.constantVelocityPredictions << '\n'
                      << "  Constant-acceleration predictions: "
                      << predictorStatistics.constantAccelerationPredictions << '\n'
                      << "  Acceleration fallbacks: "
                      << predictorStatistics.constantVelocityFallbacks << '\n'
                      << "  Delayed evaluations matched: " << evaluationStatistics.matched << '\n'
                      << "  Delayed evaluations unmatched: " << evaluationStatistics.unmatched << '\n'
                      << "  Mean predicted total error: "
                      << evaluationStatistics.meanPredictionTotalErrorDegrees << " degrees\n"
                      << "  Mean unpredicted total error: "
                      << evaluationStatistics.meanBaselineTotalErrorDegrees << " degrees\n"
                      << "  Mean improvement: "
                      << evaluationStatistics.meanTotalImprovementDegrees << " degrees\n"
                      << "  Improved evaluation ratio: "
                      << evaluationStatistics.improvedRatio << '\n';
            if (latestPredictionRecord.has_value())
            {
                printPrediction(*latestPredictionRecord, options.predictionOutput);
            }
            if (options.predictionJsonOutputPath.has_value())
            {
                xreal::sensors::OrientationPredictionMetadata metadata;
                metadata.gyroscopeScaleSource = gyroscopeScale->source;
                metadata.accelerometerProfileSource = accelerometerProfile->source;
                metadata.gyroscopeAxisMappingSource = gyroscopeScale->axisMapping.source;
                metadata.accelerometerAxisMappingSource = accelerometerProfile->axisMapping.source;
                metadata.recenterGeneration = predictionRecenterGeneration;
                metadata.recenterActive = comparisonEngine.has_value()
                    ? comparisonEngine->snapshot().fusedRecenterReference.active
                    : fusionFilter->state().recenterActive;
                std::ofstream json(
                    *options.predictionJsonOutputPath, std::ios::out | std::ios::trunc);
                if (!json)
                {
                    std::cerr << "Failed to open prediction JSON output file: "
                              << *options.predictionJsonOutputPath << '\n';
                    return 1;
                }
                json << xreal::sensors::serializeOrientationPredictionJson(
                    predictionSession->predictor().configuration(),
                    predictorStatistics,
                    evaluationStatistics,
                    latestPredictionRecord,
                    metadata);
                if (!json)
                {
                    std::cerr << "Failed while writing prediction JSON output file: "
                              << *options.predictionJsonOutputPath << '\n';
                    return 1;
                }
                std::cout << "  Prediction JSON: " << *options.predictionJsonOutputPath << '\n';
            }
        }

        if (comparisonEngine.has_value() && latestComparisonRecord.has_value())
        {
            const auto& comparisonState = comparisonEngine->fusion().state();
            xreal::sensors::OrientationComparisonSummary summary;
            summary.experiment = experimentConfiguration;
            summary.stationary = stationaryConfiguration;
            summary.finalOrientations = comparisonEngine->snapshot();
            summary.stationaryBefore = {
                beforeGyroAbsolute.metrics(), beforeGyroRelative.metrics(),
                beforeFusedAbsolute.metrics(), beforeFusedRelative.metrics()};
            summary.stationaryAfter = {
                afterGyroAbsolute.metrics(), afterGyroRelative.metrics(),
                afterFusedAbsolute.metrics(), afterFusedRelative.metrics()};
            summary.stationaryBeforeTime = stationaryBeforeTime;
            summary.motionTime = motionTime;
            summary.stationaryAfterTime = stationaryAfterTime;
            if (gyroConvergence.has_value())
            {
                summary.gyroConvergence = gyroConvergence->results();
                summary.fusedConvergence = fusedConvergence->results();
                const auto gyroFinal = gyroConvergence->finalErrorDegrees();
                const auto fusedFinal = fusedConvergence->finalErrorDegrees();
                summary.gyroRecovery = {
                    gyroFinal.has_value(), gyroConvergence->maximumErrorDegrees(),
                    gyroConvergence->meanErrorDegrees(), gyroConvergence->rmsErrorDegrees(),
                    gyroFinal.value_or(0.0)};
                summary.fusedRecovery = {
                    fusedFinal.has_value(), fusedConvergence->maximumErrorDegrees(),
                    fusedConvergence->meanErrorDegrees(), fusedConvergence->rmsErrorDegrees(),
                    fusedFinal.value_or(0.0)};
            }
            summary.finalAbsoluteDifference = latestComparisonRecord->absoluteDifference;
            summary.finalRelativeDifference = latestComparisonRecord->relativeDifference;
            summary.gyroscopeProfileSource = gyroscopeScale->source;
            summary.accelerometerProfileSource = accelerometerProfile->source;
            summary.receivedPackets = statistics.received;
            summary.gyroscopeSamplesApplied = comparisonState.appliedGyroscopeSampleCount;
            summary.accelerometerCorrectionsApplied =
                comparisonState.appliedAccelerometerCorrectionCount;
            summary.accelerometerCorrectionsSkipped =
                comparisonState.skippedAccelerometerCorrectionCount;
            summary.rejectedSamples = comparisonState.rejectedSampleCount;

            std::cout << "\nOrientation comparison summary\n";
            printDrift("stationary_before gyro absolute", summary.stationaryBefore.gyroAbsolute);
            printDrift("stationary_before gyro relative", summary.stationaryBefore.gyroRelative);
            printDrift("stationary_before fused absolute", summary.stationaryBefore.fusedAbsolute);
            printDrift("stationary_before fused relative", summary.stationaryBefore.fusedRelative);
            printDrift("stationary_after gyro absolute", summary.stationaryAfter.gyroAbsolute);
            printDrift("stationary_after gyro relative", summary.stationaryAfter.gyroRelative);
            printDrift("stationary_after fused absolute", summary.stationaryAfter.fusedAbsolute);
            printDrift("stationary_after fused relative", summary.stationaryAfter.fusedRelative);
            if (gyroConvergence.has_value())
            {
                for (std::size_t index = 0; index < summary.gyroConvergence.size(); ++index)
                {
                    const auto& gyro = summary.gyroConvergence[index];
                    const auto& fused = summary.fusedConvergence[index];
                    std::cout << "  convergence " << gyro.thresholdDegrees << " deg: gyro=";
                    if (gyro.reachedAfterSeconds.has_value())
                    {
                        std::cout << *gyro.reachedAfterSeconds << " s";
                    }
                    else
                    {
                        std::cout << "unavailable";
                    }
                    std::cout << ", fused=";
                    if (fused.reachedAfterSeconds.has_value())
                    {
                        std::cout << *fused.reachedAfterSeconds << " s";
                    }
                    else
                    {
                        std::cout << "unavailable";
                    }
                    std::cout << '\n';
                }
                std::cout << "  gyro recovery tilt: max=" << summary.gyroRecovery.maximumErrorDegrees
                          << " mean=" << summary.gyroRecovery.meanErrorDegrees
                          << " rms=" << summary.gyroRecovery.rmsErrorDegrees
                          << " final=" << summary.gyroRecovery.finalErrorDegrees << " deg\n"
                          << "  fused recovery tilt: max="
                          << summary.fusedRecovery.maximumErrorDegrees
                          << " mean=" << summary.fusedRecovery.meanErrorDegrees
                          << " rms=" << summary.fusedRecovery.rmsErrorDegrees
                          << " final=" << summary.fusedRecovery.finalErrorDegrees << " deg\n";
            }
            printComparison(*latestComparisonRecord, options.comparisonOutput);

            if (options.comparisonJsonOutputPath.has_value())
            {
                std::ofstream json(
                    *options.comparisonJsonOutputPath, std::ios::out | std::ios::trunc);
                if (!json)
                {
                    std::cerr << "Failed to open comparison JSON output file: "
                              << *options.comparisonJsonOutputPath << '\n';
                    return 1;
                }
                json << xreal::sensors::serializeOrientationComparisonJson(summary);
                if (!json)
                {
                    std::cerr << "Failed while writing comparison JSON output file: "
                              << *options.comparisonJsonOutputPath << '\n';
                    return 1;
                }
                std::cout << "  Comparison JSON: " << *options.comparisonJsonOutputPath << '\n';
            }
        }

        bool calibrationRejected{};
        if (gyroscopeCalibrator.has_value())
        {
            const auto result = gyroscopeCalibrator->finish();
            const auto& gyro = result.statistics.gyroscopeRaw;
            std::cout << "\nGyroscope bias calibration (raw units)\n"
                      << "  Status: " << (result.accepted ? "accepted" : "rejected") << '\n'
                      << "  Samples: " << result.statistics.sampleCount << '\n'
                      << "  Device duration: "
                      << std::chrono::duration<double>(result.statistics.captureDuration).count()
                      << " seconds\n"
                      << "  Packet rate: " << result.statistics.packetRate << " packets/s\n";
            printVectorStatistics("gyro_raw", gyro);
            std::cout << "  gyro_range_raw: [" << result.statistics.rangeRaw.x << ", "
                      << result.statistics.rangeRaw.y << ", "
                      << result.statistics.rangeRaw.z << "]\n";

            if (result.accepted && result.biasRaw.has_value())
            {
                std::cout << "  Bias raw units: [" << result.biasRaw->x << ", "
                          << result.biasRaw->y << ", " << result.biasRaw->z << "]\n";
            }
            else
            {
                std::cerr << "Gyroscope bias calibration rejected: "
                          << xreal::sensors::gyroscopeBiasCalibrationRejectionReasonText(
                                 result.rejectionReason)
                          << ".\n";
                calibrationRejected = true;
            }

            if (options.gyroscopeCalibrationOutputPath.has_value())
            {
                const xreal::sensors::GyroscopeBiasCalibrationDevice calibrationDevice{
                    xreal::sensors::XrealImuStream::vendorId,
                    xreal::sensors::XrealImuStream::productId,
                    xreal::sensors::XrealImuStream::interfaceNumber,
                    narrowAscii(device->productName),
                };
                std::ofstream json(
                    *options.gyroscopeCalibrationOutputPath,
                    std::ios::out | std::ios::trunc);
                if (!json)
                {
                    std::cerr << "Failed to open gyroscope calibration JSON output file: "
                              << *options.gyroscopeCalibrationOutputPath << '\n';
                    return 1;
                }
                json << xreal::sensors::serializeGyroscopeBiasCalibrationJson(
                    calibrationDevice,
                    result);
                if (!json)
                {
                    std::cerr << "Failed while writing gyroscope calibration JSON output file: "
                              << *options.gyroscopeCalibrationOutputPath << '\n';
                    return 1;
                }
                std::cout << "  Gyroscope calibration JSON: "
                          << *options.gyroscopeCalibrationOutputPath << '\n';
            }
        }

        if (calibrationAccumulator.has_value())
        {
            const auto calibrationStatistics = calibrationAccumulator->statistics();
            const auto stationary = calibrationAccumulator->stationaryResult();
            if (!stationary.has_value())
            {
                std::cerr << "Calibration failed: no decoded sensor samples were recorded.\n";
                return 1;
            }

            std::cout << "\nCalibration analysis (raw units)\n"
                      << "  Name: " << *options.calibrationName << '\n'
                      << "  Samples: " << calibrationStatistics.sampleCount << '\n'
                      << "  Capture duration: "
                      << std::chrono::duration<double>(calibrationStatistics.captureDuration).count()
                      << " seconds\n";
            printVectorStatistics("gyro_raw", calibrationStatistics.gyroscopeRaw);
            printVectorStatistics("accel_raw", calibrationStatistics.accelerometerRaw);
            printScalarStatistics(
                "device_timestamp_delta_ns",
                calibrationStatistics.timestampDeltas.deviceNanoseconds);
            printScalarStatistics(
                "host_timestamp_delta_ns",
                calibrationStatistics.timestampDeltas.hostNanoseconds);
            std::cout << "  Gyro stationary estimate: " << (stationary->accepted ? "accepted" : "rejected")
                      << '\n'
                      << "  Gyro bias raw: [" << stationary->gyroBias.x << ", "
                      << stationary->gyroBias.y << ", " << stationary->gyroBias.z << "]\n"
                      << "  Accel mean raw: [" << stationary->accelMean.x << ", "
                      << stationary->accelMean.y << ", " << stationary->accelMean.z << "]\n"
                      << "  Accel vector magnitude raw: " << stationary->accelVectorMagnitudeRaw << '\n';

            const xreal::sensors::ImuCalibrationReport report{
                *options.calibrationName,
                calibrationStatistics,
                *stationary,
                seconds > 0.0 ? static_cast<double>(statistics.received) / seconds : 0.0,
                statistics.dropped,
                statistics.outOfSequence,
                statistics.invalid,
                diagnosticQueueDrops.load(),
            };

            if (options.calibrationOutputPath.has_value())
            {
                std::ofstream json(*options.calibrationOutputPath, std::ios::out | std::ios::trunc);
                if (!json)
                {
                    std::cerr << "Failed to open calibration JSON output file: "
                              << *options.calibrationOutputPath << '\n';
                    return 1;
                }
                json << xreal::sensors::serializeCalibrationReportJson(report);
                if (!json)
                {
                    std::cerr << "Failed while writing calibration JSON output file: "
                              << *options.calibrationOutputPath << '\n';
                    return 1;
                }
                std::cout << "  Calibration JSON: " << *options.calibrationOutputPath << '\n';
            }

            if (diagnosticQueueDrops.load() != 0U)
            {
                std::cerr << "Calibration rejected because decoded samples were dropped by the diagnostic queue.\n";
                calibrationRejected = true;
            }
            if (calibrationStatistics.sampleCount != statistics.received)
            {
                std::cerr << "Calibration rejected because the analyzed sample count does not match"
                             " the decoded packet count.\n";
                calibrationRejected = true;
            }
            const bool rotationalCalibration = options.calibrationName->ends_with("-rotation");
            if (!rotationalCalibration && !stationary->accepted)
            {
                std::cerr << "Stationary calibration rejected because gyroscope movement exceeded the raw threshold.\n";
                calibrationRejected = true;
            }
        }

        if (orientationIntegrator.has_value()
            && (options.orientationProfileOutputPath.has_value()
                || (options.fuseGyroscopeAccelerometer
                    && options.fusionJsonOutputPath.has_value())))
        {
            const auto& outputPath = options.fuseGyroscopeAccelerometer
                ? *options.fusionJsonOutputPath
                : *options.orientationProfileOutputPath;
            std::ofstream json(
                outputPath,
                std::ios::out | std::ios::trunc);
            if (!json)
            {
                std::cerr << "Failed to open orientation JSON output file: "
                          << outputPath << '\n';
                return 1;
            }
            const bool includeEuler = selectedOrientationOutput
                != xreal::diagnostics::OrientationOutputMode::quaternion;
            json << xreal::sensors::serializeGyroscopeOrientationJson(
                *orientationIntegrator,
                *gyroscopeScale,
                includeEuler);
            if (!json)
            {
                std::cerr << "Failed while writing orientation JSON output file: "
                          << outputPath << '\n';
                return 1;
            }
            std::cout << "  Orientation JSON: " << outputPath << '\n';
        }

        if (fusionFilter.has_value() && options.fusionJsonOutputPath.has_value())
        {
            std::ofstream json(*options.fusionJsonOutputPath, std::ios::out | std::ios::trunc);
            if (!json)
            {
                std::cerr << "Failed to open fusion JSON output file: "
                          << *options.fusionJsonOutputPath << '\n';
                return 1;
            }
            const bool includeEuler = options.fusionOutput
                != xreal::diagnostics::OrientationOutputMode::quaternion;
            json << xreal::sensors::serializeOrientationFusionJson(
                *fusionFilter,
                *gyroscopeScale,
                *accelerometerProfile,
                includeEuler);
            if (!json)
            {
                std::cerr << "Failed while writing fusion JSON output file: "
                          << *options.fusionJsonOutputPath << '\n';
                return 1;
            }
            std::cout << "  Fusion JSON: " << *options.fusionJsonOutputPath << '\n';
        }

        const std::wstring streamError = stream.errorMessage();
        if (!streamError.empty())
        {
            std::wcerr << L"IMU acquisition stopped with an error: " << streamError << L'\n';
            return 1;
        }

        return calibrationRejected ? 1 : 0;
    }
    catch (const std::exception& exception)
    {
        std::cerr << "IMU diagnostic startup failed: " << exception.what() << '\n';
        return 1;
    }
}
