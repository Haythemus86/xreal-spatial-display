#include "sensors/GyroscopeRecordingAnalysis.hpp"

#include "sensors/DeviceTimestampDelta.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <numbers>
#include <sstream>
#include <stdexcept>

namespace xreal::sensors
{
namespace
{

struct AxisAccumulator
{
    double minimum{std::numeric_limits<double>::infinity()};
    double maximum{-std::numeric_limits<double>::infinity()};
    double sum{};
    double sumSquared{};
    double positive{};
    double negative{};
    double energy{};
    double peak{};
    std::uint64_t count{};
    std::uint64_t aboveStartNanoseconds{};
    std::uint64_t aboveStopNanoseconds{};
};

[[nodiscard]] std::array<double, 3> values(const CorrectedGyroscopeRaw& value) noexcept
{
    return {value.x, value.y, value.z};
}

[[nodiscard]] double component(const RawVector3d& value, GyroscopeAxis axis) noexcept
{
    switch (axis)
    {
    case GyroscopeAxis::x: return value.x;
    case GyroscopeAxis::y: return value.y;
    case GyroscopeAxis::z: return value.z;
    }
    return 0.0;
}

[[nodiscard]] GyroscopeAxis axisFromIndex(std::size_t index) noexcept
{
    if (index == 0U) return GyroscopeAxis::x;
    if (index == 1U) return GyroscopeAxis::y;
    return GyroscopeAxis::z;
}

[[nodiscard]] std::size_t axisIndex(GyroscopeAxis axis) noexcept
{
    if (axis == GyroscopeAxis::x) return 0U;
    if (axis == GyroscopeAxis::y) return 1U;
    return 2U;
}

[[nodiscard]] GyroscopeAxis dominantAxis(const std::array<double, 3>& metric) noexcept
{
    return axisFromIndex(static_cast<std::size_t>(
        std::distance(metric.begin(), std::max_element(metric.begin(), metric.end()))));
}

[[nodiscard]] double clampUnit(double value) noexcept
{
    return std::clamp(value, 0.0, 1.0);
}

void accumulateSignedTrapezoid(
    double previous,
    double current,
    double seconds,
    double& positive,
    double& negative) noexcept
{
    if (previous >= 0.0 && current >= 0.0)
    {
        positive += (previous + current) * 0.5 * seconds;
        return;
    }
    if (previous <= 0.0 && current <= 0.0)
    {
        negative += (previous + current) * 0.5 * seconds;
        return;
    }
    const double total = std::abs(previous) + std::abs(current);
    if (total == 0.0)
    {
        return;
    }
    const double firstSeconds = seconds * std::abs(previous) / total;
    const double secondSeconds = seconds - firstSeconds;
    const double firstArea = previous * 0.5 * firstSeconds;
    const double secondArea = current * 0.5 * secondSeconds;
    (firstArea >= 0.0 ? positive : negative) += firstArea;
    (secondArea >= 0.0 ? positive : negative) += secondArea;
}

[[nodiscard]] GyroscopeRecordingAxisAnalysis finishAxis(
    const AxisAccumulator& accumulator,
    std::uint64_t sampleCount,
    std::chrono::nanoseconds duration,
    double totalEnergy)
{
    GyroscopeRecordingAxisAnalysis result;
    result.sampleCount = sampleCount;
    result.validSampleCount = accumulator.count;
    result.captureDuration = duration;
    if (accumulator.count != 0U)
    {
        const double count = static_cast<double>(accumulator.count);
        result.minimumCorrectedRaw = accumulator.minimum;
        result.maximumCorrectedRaw = accumulator.maximum;
        result.meanCorrectedRaw = accumulator.sum / count;
        result.standardDeviationCorrectedRaw = std::sqrt(std::max(
            0.0,
            accumulator.sumSquared / count
                - result.meanCorrectedRaw * result.meanCorrectedRaw));
        result.rootMeanSquareCorrectedRaw = std::sqrt(accumulator.sumSquared / count);
    }
    result.peakAbsoluteCorrectedRaw = accumulator.peak;
    result.positiveIntegratedRawAngle = accumulator.positive;
    result.negativeIntegratedRawAngle = accumulator.negative;
    result.signedIntegratedRawAngle = accumulator.positive + accumulator.negative;
    result.absoluteIntegratedRawAngle = accumulator.positive - accumulator.negative;
    result.durationAboveStartThreshold =
        std::chrono::nanoseconds(accumulator.aboveStartNanoseconds);
    result.durationAboveStopThreshold =
        std::chrono::nanoseconds(accumulator.aboveStopNanoseconds);
    result.rotationalEnergy = accumulator.energy;
    result.rotationalEnergyPercent = totalEnergy > 0.0
        ? accumulator.energy / totalEnergy * 100.0
        : 0.0;
    result.dominantAxisScore = totalEnergy > 0.0 ? accumulator.energy / totalEnergy : 0.0;
    return result;
}

[[nodiscard]] GyroscopeRotationSegment buildSegment(
    std::span<const GyroscopeRecordingSample> samples,
    std::size_t begin,
    std::size_t end,
    bool stillnessBefore,
    bool stillnessAfter)
{
    GyroscopeRotationSegment segment;
    segment.startDeviceTimestamp = samples[begin].original.deviceTimestamp.nanoseconds;
    segment.endDeviceTimestamp = samples[end].original.deviceTimestamp.nanoseconds;
    segment.stillnessBefore = stillnessBefore;
    segment.stillnessAfter = stillnessAfter;
    segment.sampleCount = end - begin + 1U;

    std::array<double, 3> integrated{};
    std::array<double, 3> peak{};
    std::array<double, 3> energy{};
    std::uint64_t durationNanoseconds{};
    for (std::size_t index = begin; index <= end; ++index)
    {
        const auto current = values(samples[index].corrected);
        for (std::size_t axis = 0; axis < 3U; ++axis)
        {
            peak[axis] = std::max(peak[axis], std::abs(current[axis]));
        }
        if (index == begin || !samples[index].deviceDeltaNanoseconds.has_value()
            || !samples[index].sampleValid || !samples[index - 1U].sampleValid)
        {
            continue;
        }
        const auto previous = values(samples[index - 1U].corrected);
        const std::uint64_t delta = *samples[index].deviceDeltaNanoseconds;
        const double seconds = static_cast<double>(delta) / 1'000'000'000.0;
        durationNanoseconds += delta;
        for (std::size_t axis = 0; axis < 3U; ++axis)
        {
            integrated[axis] += (previous[axis] + current[axis]) * 0.5 * seconds;
            energy[axis] += (previous[axis] * previous[axis] + current[axis] * current[axis])
                * 0.5 * seconds;
        }
    }

    segment.duration = std::chrono::nanoseconds(durationNanoseconds);
    segment.integratedRawAngle = {integrated[0], integrated[1], integrated[2]};
    segment.peakAbsoluteCorrectedRaw = {peak[0], peak[1], peak[2]};
    segment.dominantAxis = dominantAxis(energy);
    const std::size_t dominant = axisIndex(segment.dominantAxis);
    const double otherPeak = std::max(
        peak[(dominant + 1U) % 3U],
        peak[(dominant + 2U) % 3U]);
    segment.crossAxisRatio = peak[dominant] > 0.0 ? otherPeak / peak[dominant] : 0.0;
    const double totalEnergy = energy[0] + energy[1] + energy[2];
    segment.dominanceScore = totalEnergy > 0.0 ? energy[dominant] / totalEnergy : 0.0;
    segment.direction = integrated[dominant] > 0.0
        ? RotationDirection::positive
        : (integrated[dominant] < 0.0
            ? RotationDirection::negative
            : RotationDirection::automatic);
    const double durationSeconds = std::chrono::duration<double>(segment.duration).count();
    const double durationScore = std::min(durationSeconds, 1.0)
        * std::min(10.0 / std::max(durationSeconds, 1.0e-9), 1.0);
    const double stillnessScore = (stillnessBefore ? 0.5 : 0.0)
        + (stillnessAfter ? 0.5 : 0.0);
    segment.selectionScore = 0.5 * segment.dominanceScore
        + 0.25 / (1.0 + segment.crossAxisRatio)
        + 0.15 * stillnessScore
        + 0.10 * durationScore;
    return segment;
}

[[nodiscard]] std::string escapeJson(std::string_view value)
{
    std::ostringstream output;
    for (const char character : value)
    {
        if (character == '"') output << "\\\"";
        else if (character == '\\') output << "\\\\";
        else if (character == '\n') output << "\\n";
        else output << character;
    }
    return output.str();
}

void writeAxisJson(
    std::ostringstream& output,
    std::string_view name,
    const GyroscopeRecordingAxisAnalysis& axis)
{
    output << "    \"" << name << "\":{\"sample_count\":" << axis.sampleCount
           << ",\"valid_sample_count\":" << axis.validSampleCount
           << ",\"capture_duration_seconds\":"
           << std::chrono::duration<double>(axis.captureDuration).count()
           << ",\"minimum_corrected_raw\":" << axis.minimumCorrectedRaw
           << ",\"maximum_corrected_raw\":" << axis.maximumCorrectedRaw
           << ",\"mean_corrected_raw\":" << axis.meanCorrectedRaw
           << ",\"standard_deviation\":" << axis.standardDeviationCorrectedRaw
           << ",\"rms\":" << axis.rootMeanSquareCorrectedRaw
           << ",\"peak_absolute_corrected_raw\":" << axis.peakAbsoluteCorrectedRaw
           << ",\"positive_integrated_raw_angle\":" << axis.positiveIntegratedRawAngle
           << ",\"negative_integrated_raw_angle\":" << axis.negativeIntegratedRawAngle
           << ",\"signed_integrated_raw_angle\":" << axis.signedIntegratedRawAngle
           << ",\"absolute_integrated_raw_angle\":" << axis.absoluteIntegratedRawAngle
           << ",\"duration_above_start_seconds\":"
           << std::chrono::duration<double>(axis.durationAboveStartThreshold).count()
           << ",\"duration_above_stop_seconds\":"
           << std::chrono::duration<double>(axis.durationAboveStopThreshold).count()
           << ",\"rotational_energy_percent\":" << axis.rotationalEnergyPercent
           << ",\"dominant_axis_score\":" << axis.dominantAxisScore << '}';
}

} // namespace

GyroscopeRecordingAnalysisResult analyzeGyroscopeRecording(
    std::span<const ImuSample> samples,
    const GyroscopeBias& bias,
    GyroscopeRecordingAnalysisConfig configuration)
{
    if (!std::isfinite(configuration.startThresholdRaw)
        || !std::isfinite(configuration.stopThresholdRaw)
        || configuration.startThresholdRaw <= configuration.stopThresholdRaw
        || configuration.stopThresholdRaw <= 0.0
        || configuration.stillnessDuration <= std::chrono::nanoseconds::zero()
        || configuration.maximumDeviceDelta <= std::chrono::nanoseconds::zero()
        || (configuration.expectedAngleDegrees.has_value()
            && (!std::isfinite(*configuration.expectedAngleDegrees)
                || *configuration.expectedAngleDegrees <= 0.0)))
    {
        throw std::invalid_argument("Gyroscope recording analysis configuration is invalid.");
    }

    GyroscopeRecordingAnalysisResult result;
    result.configuration = configuration;
    result.bias = bias;
    result.samples.reserve(samples.size());
    if (samples.empty())
    {
        result.failure = GyroscopeRecordingAnalysisFailure::noSamples;
        result.warnings.emplace_back("The recording contains no decoded IMU samples.");
        return result;
    }

    std::optional<std::uint64_t> previousTimestamp;
    std::optional<std::uint8_t> previousSequence;
    std::uint64_t elapsed{};
    for (const auto& sample : samples)
    {
        GyroscopeRecordingSample recorded;
        recorded.original = sample;
        recorded.corrected = applyGyroscopeBias(sample.gyroscopeRaw, bias);
        if (previousSequence.has_value())
        {
            const std::uint8_t expected = static_cast<std::uint8_t>(*previousSequence + 1U);
            const std::uint8_t distance = static_cast<std::uint8_t>(sample.packetSequence - expected);
            if (distance != 0U && distance < 128U)
            {
                recorded.sequenceGap = distance;
                result.sequenceGaps += distance;
            }
        }
        previousSequence = sample.packetSequence;

        if (!previousTimestamp.has_value())
        {
            recorded.timestampStatus = RecordingTimestampStatus::firstSample;
            ++result.validTimestampSamples;
        }
        else
        {
            const auto delta = forwardDeviceTimestampDelta(
                sample.deviceTimestamp.nanoseconds,
                *previousTimestamp);
            if (!delta.has_value())
            {
                recorded.timestampStatus = sample.deviceTimestamp.nanoseconds == *previousTimestamp
                    ? RecordingTimestampStatus::duplicate
                    : RecordingTimestampStatus::decreasing;
                recorded.sampleValid = false;
                ++result.invalidTimestampSamples;
            }
            else if (*delta > static_cast<std::uint64_t>(configuration.maximumDeviceDelta.count()))
            {
                recorded.timestampStatus = RecordingTimestampStatus::deltaTooLarge;
                recorded.sampleValid = false;
                ++result.invalidTimestampSamples;
            }
            else
            {
                recorded.timestampStatus = RecordingTimestampStatus::valid;
                recorded.deviceDeltaNanoseconds = *delta;
                elapsed += *delta;
                ++result.validTimestampSamples;
            }
        }
        recorded.elapsedDeviceNanoseconds = elapsed;
        previousTimestamp = sample.deviceTimestamp.nanoseconds;
        result.samples.push_back(recorded);
    }

    std::array<AxisAccumulator, 3> accumulators;
    for (std::size_t index = 0; index < result.samples.size(); ++index)
    {
        const auto current = values(result.samples[index].corrected);
        if (result.samples[index].sampleValid)
        {
            for (std::size_t axis = 0; axis < 3U; ++axis)
            {
                auto& accumulator = accumulators[axis];
                accumulator.minimum = std::min(accumulator.minimum, current[axis]);
                accumulator.maximum = std::max(accumulator.maximum, current[axis]);
                accumulator.sum += current[axis];
                accumulator.sumSquared += current[axis] * current[axis];
                accumulator.peak = std::max(accumulator.peak, std::abs(current[axis]));
                ++accumulator.count;
            }
        }
        if (index == 0U || !result.samples[index].deviceDeltaNanoseconds.has_value()
            || !result.samples[index - 1U].sampleValid)
        {
            continue;
        }
        const auto previous = values(result.samples[index - 1U].corrected);
        const std::uint64_t delta = *result.samples[index].deviceDeltaNanoseconds;
        const double seconds = static_cast<double>(delta) / 1'000'000'000.0;
        for (std::size_t axis = 0; axis < 3U; ++axis)
        {
            auto& accumulator = accumulators[axis];
            accumulateSignedTrapezoid(
                previous[axis], current[axis], seconds, accumulator.positive, accumulator.negative);
            accumulator.energy += (previous[axis] * previous[axis]
                + current[axis] * current[axis]) * 0.5 * seconds;
            if (std::max(std::abs(previous[axis]), std::abs(current[axis]))
                >= configuration.startThresholdRaw)
            {
                accumulator.aboveStartNanoseconds += delta;
            }
            if (std::max(std::abs(previous[axis]), std::abs(current[axis]))
                >= configuration.stopThresholdRaw)
            {
                accumulator.aboveStopNanoseconds += delta;
            }
        }
    }

    const double totalEnergy = accumulators[0].energy + accumulators[1].energy
        + accumulators[2].energy;
    const auto duration = std::chrono::nanoseconds(elapsed);
    result.x = finishAxis(accumulators[0], samples.size(), duration, totalEnergy);
    result.y = finishAxis(accumulators[1], samples.size(), duration, totalEnergy);
    result.z = finishAxis(accumulators[2], samples.size(), duration, totalEnergy);
    result.dominantAxis = dominantAxis(
        {accumulators[0].energy, accumulators[1].energy, accumulators[2].energy});

    bool active{};
    std::size_t segmentBegin{};
    std::size_t stillnessBegin{};
    std::uint64_t stillnessElapsed{};
    std::uint64_t priorStillness{};
    bool segmentStillnessBefore{};
    for (std::size_t index = 0; index < result.samples.size(); ++index)
    {
        if (!result.samples[index].sampleValid)
        {
            continue;
        }
        const auto corrected = values(result.samples[index].corrected);
        const double maximum = *std::max_element(
            corrected.begin(),
            corrected.end(),
            [](double left, double right) { return std::abs(left) < std::abs(right); });
        const double magnitude = std::abs(maximum);
        const std::uint64_t delta = result.samples[index].deviceDeltaNanoseconds.value_or(0U);
        if (!active)
        {
            if (magnitude >= configuration.startThresholdRaw)
            {
                active = true;
                segmentBegin = index == 0U ? 0U : index - 1U;
                segmentStillnessBefore = priorStillness
                    >= static_cast<std::uint64_t>(configuration.stillnessDuration.count());
                stillnessElapsed = 0U;
            }
            else if (magnitude <= configuration.stopThresholdRaw)
            {
                priorStillness += delta;
            }
            else
            {
                priorStillness = 0U;
            }
            continue;
        }

        if (magnitude <= configuration.stopThresholdRaw)
        {
            if (stillnessElapsed == 0U)
            {
                stillnessBegin = index;
            }
            stillnessElapsed += delta;
            if (stillnessElapsed
                >= static_cast<std::uint64_t>(configuration.stillnessDuration.count()))
            {
                const std::size_t segmentEnd = stillnessBegin > segmentBegin
                    ? stillnessBegin
                    : index;
                result.segments.push_back(buildSegment(
                    result.samples, segmentBegin, segmentEnd, segmentStillnessBefore, true));
                active = false;
                priorStillness = stillnessElapsed;
                stillnessElapsed = 0U;
            }
        }
        else
        {
            stillnessElapsed = 0U;
        }
    }
    if (active)
    {
        result.segments.push_back(buildSegment(
            result.samples,
            segmentBegin,
            result.samples.size() - 1U,
            segmentStillnessBefore,
            false));
    }

    if (result.invalidTimestampSamples != 0U)
    {
        result.warnings.emplace_back("One or more samples had invalid device timestamps and were excluded from integration.");
    }
    for (const auto& segment : result.segments)
    {
        if (segment.crossAxisRatio > 0.5)
        {
            result.warnings.emplace_back("A detected segment contains substantial cross-axis motion.");
            break;
        }
    }
    if (result.segments.empty())
    {
        result.failure = GyroscopeRecordingAnalysisFailure::noUsableSegment;
        result.warnings.emplace_back("No motion segment crossed the configured start threshold.");
        return result;
    }

    result.bestSegmentIndex = static_cast<std::size_t>(std::distance(
        result.segments.begin(),
        std::max_element(
            result.segments.begin(),
            result.segments.end(),
            [](const auto& left, const auto& right) {
                return left.selectionScore < right.selectionScore;
            })));
    result.usable = true;

    if (configuration.expectedAngleDegrees.has_value())
    {
        const auto& segment = result.segments[*result.bestSegmentIndex];
        GyroscopeAxis scaleAxis = segment.dominantAxis;
        if (configuration.axisSelection != GyroscopeAxisSelection::automatic)
        {
            scaleAxis = axisFromIndex(static_cast<std::size_t>(configuration.axisSelection));
        }
        const double integrated = component(segment.integratedRawAngle, scaleAxis);
        const double rawPerDegree = std::abs(integrated) / *configuration.expectedAngleDegrees;
        if (std::isfinite(rawPerDegree) && rawPerDegree > 0.0)
        {
            const double rawPerRadian = rawPerDegree * 180.0 / std::numbers::pi;
            const RotationDirection measured = integrated > 0.0
                ? RotationDirection::positive
                : RotationDirection::negative;
            const bool directionMatches = configuration.requestedDirection
                    == RotationDirection::automatic
                || configuration.requestedDirection == measured;
            const auto axisAnalysis = scaleAxis == GyroscopeAxis::x ? result.x
                : (scaleAxis == GyroscopeAxis::y ? result.y : result.z);
            const double timestampScore = static_cast<double>(result.validTimestampSamples)
                / static_cast<double>(result.samples.size());
            const double durationSeconds = std::chrono::duration<double>(segment.duration).count();
            const double durationScore = std::min(durationSeconds, 1.0)
                * std::min(10.0 / std::max(durationSeconds, 1.0e-9), 1.0);
            const double stillnessScore = (segment.stillnessBefore ? 0.5 : 0.0)
                + (segment.stillnessAfter ? 0.5 : 0.0);
            const double agreement = axisAnalysis.absoluteIntegratedRawAngle > 0.0
                ? std::abs(integrated) / axisAnalysis.absoluteIntegratedRawAngle
                : 0.0;
            const double confidence = clampUnit(
                segment.dominanceScore
                * (1.0 / (1.0 + segment.crossAxisRatio))
                * timestampScore
                * durationScore
                * (0.5 + 0.5 * stillnessScore)
                * clampUnit(agreement));
            result.scaleEstimate = {
                true,
                true,
                scaleAxis,
                *configuration.expectedAngleDegrees,
                rawPerDegree,
                rawPerRadian,
                1.0 / rawPerDegree,
                1.0 / rawPerRadian,
                directionMatches,
                confidence,
            };
            if (!directionMatches)
            {
                result.warnings.emplace_back("The selected segment direction differs from the requested direction.");
            }
        }
    }
    return result;
}

std::string gyroscopeRecordingCsvHeader(
    bool includeAccelerometer,
    bool includeHostTimestamps)
{
    std::string header = "sequence,device_timestamp,device_delta_ns,elapsed_device_ns,"
        "sample_valid,timestamp_valid,timestamp_status,sequence_gap,gyro_raw_x,gyro_raw_y,"
        "gyro_raw_z,bias_raw_x,bias_raw_y,bias_raw_z,gyro_corrected_x,gyro_corrected_y,"
        "gyro_corrected_z";
    if (includeHostTimestamps)
    {
        header += ",host_timestamp_ns";
    }
    if (includeAccelerometer)
    {
        header += ",accel_raw_x,accel_raw_y,accel_raw_z";
    }
    return header + '\n';
}

std::string serializeGyroscopeRecordingCsvRow(
    const GyroscopeRecordingSample& sample,
    const GyroscopeBias& bias,
    bool includeAccelerometer,
    bool includeHostTimestamps)
{
    std::ostringstream output;
    output << std::setprecision(std::numeric_limits<double>::max_digits10)
           << static_cast<unsigned int>(sample.original.packetSequence) << ','
           << sample.original.deviceTimestamp.nanoseconds << ',';
    if (sample.deviceDeltaNanoseconds.has_value()) output << *sample.deviceDeltaNanoseconds;
    output << ',' << sample.elapsedDeviceNanoseconds << ','
           << (sample.sampleValid ? "true" : "false") << ','
           << (sample.timestampStatus == RecordingTimestampStatus::firstSample
                   || sample.timestampStatus == RecordingTimestampStatus::valid
               ? "true" : "false")
           << ',' << recordingTimestampStatusText(sample.timestampStatus) << ','
           << sample.sequenceGap << ','
           << sample.original.gyroscopeRaw.x << ',' << sample.original.gyroscopeRaw.y << ','
           << sample.original.gyroscopeRaw.z << ',' << bias.x << ',' << bias.y << ',' << bias.z
           << ',' << sample.corrected.x << ',' << sample.corrected.y << ',' << sample.corrected.z;
    if (includeHostTimestamps)
    {
        output << ',' << std::chrono::duration_cast<std::chrono::nanoseconds>(
            sample.original.hostReceiveTimestamp.time_since_epoch()).count();
    }
    if (includeAccelerometer)
    {
        output << ',' << sample.original.accelerometerRaw.x << ','
               << sample.original.accelerometerRaw.y << ','
               << sample.original.accelerometerRaw.z;
    }
    output << '\n';
    return output.str();
}

std::string serializeGyroscopeRecordingAnalysisJson(
    const GyroscopeRecordingMetadata& metadata,
    const GyroscopeRecordingAnalysisResult& analysis)
{
    std::ostringstream output;
    output << std::setprecision(std::numeric_limits<double>::max_digits10)
           << "{\n  \"schema_version\":1,\n  \"mode\":\"record-only\",\n"
           << "  \"device\":{\"vendor_id\":\"0x" << std::hex << std::uppercase
           << std::setw(4) << std::setfill('0') << metadata.device.vendorId
           << "\",\"product_id\":\"0x" << std::setw(4) << metadata.device.productId
           << std::dec << std::nouppercase << std::setfill(' ')
           << "\",\"interface_number\":" << metadata.device.interfaceNumber
           << ",\"product\":\"" << escapeJson(metadata.device.productName) << "\"},\n"
           << "  \"bias_calibration\":{\"accepted\":"
           << (metadata.biasCalibration.accepted ? "true" : "false")
           << ",\"bias_raw\":{\"x\":" << analysis.bias.x << ",\"y\":" << analysis.bias.y
           << ",\"z\":" << analysis.bias.z << "}},\n"
           << "  \"recording\":{\"requested_duration_seconds\":"
           << metadata.requestedDurationSeconds << ",\"measured_duration_seconds\":"
           << metadata.measuredDurationSeconds << ",\"sample_count\":" << analysis.samples.size()
           << ",\"valid_timestamp_samples\":" << analysis.validTimestampSamples
           << ",\"invalid_timestamp_samples\":" << analysis.invalidTimestampSamples
           << ",\"sequence_gaps\":" << analysis.sequenceGaps
           << ",\"malformed_packets\":" << metadata.malformedPackets
           << ",\"out_of_sequence_events\":" << metadata.outOfSequenceEvents
           << ",\"diagnostic_queue_drops\":" << metadata.diagnosticQueueDrops << "},\n"
           << "  \"analysis\":{\"usable\":" << (analysis.usable ? "true" : "false")
           << ",\"failure_reason\":\""
           << escapeJson(gyroscopeRecordingAnalysisFailureText(analysis.failure))
           << "\",\"dominant_axis\":\"" << gyroscopeAxisText(analysis.dominantAxis) << "\"},\n"
           << "  \"axis_analysis\":{\n";
    writeAxisJson(output, "x", analysis.x); output << ",\n";
    writeAxisJson(output, "y", analysis.y); output << ",\n";
    writeAxisJson(output, "z", analysis.z); output << "\n  },\n  \"segments\":[\n";
    for (std::size_t index = 0; index < analysis.segments.size(); ++index)
    {
        const auto& segment = analysis.segments[index];
        output << "    {\"start_device_timestamp\":" << segment.startDeviceTimestamp
               << ",\"end_device_timestamp\":" << segment.endDeviceTimestamp
               << ",\"duration_seconds\":" << std::chrono::duration<double>(segment.duration).count()
               << ",\"dominant_axis\":\"" << gyroscopeAxisText(segment.dominantAxis)
               << "\",\"direction\":\"" << rotationDirectionText(segment.direction)
               << "\",\"integrated_raw_angle\":{\"x\":" << segment.integratedRawAngle.x
               << ",\"y\":" << segment.integratedRawAngle.y << ",\"z\":"
               << segment.integratedRawAngle.z << "},\"peak_absolute_corrected_raw\":{\"x\":"
               << segment.peakAbsoluteCorrectedRaw.x << ",\"y\":"
               << segment.peakAbsoluteCorrectedRaw.y << ",\"z\":"
               << segment.peakAbsoluteCorrectedRaw.z << "},\"cross_axis_ratio\":"
               << segment.crossAxisRatio << ",\"dominance_score\":" << segment.dominanceScore
               << ",\"stillness_before\":" << (segment.stillnessBefore ? "true" : "false")
               << ",\"stillness_after\":" << (segment.stillnessAfter ? "true" : "false")
               << ",\"sample_count\":" << segment.sampleCount
               << ",\"selection_score\":" << segment.selectionScore << '}';
        if (index + 1U != analysis.segments.size()) output << ',';
        output << '\n';
    }
    output << "  ],\n  \"best_segment_index\":";
    if (analysis.bestSegmentIndex.has_value()) output << *analysis.bestSegmentIndex;
    else output << "null";
    output << ",\n  \"best_segment_selection\":\"highest deterministic weighted score: 50% axis dominance, 25% inverse cross-axis ratio, 15% endpoint stillness, 10% duration quality\",\n"
           << "  \"scale_estimate\":{\"available\":"
           << (analysis.scaleEstimate.available ? "true" : "false")
           << ",\"experimental\":true,\"axis\":\""
           << gyroscopeAxisText(analysis.scaleEstimate.axis)
           << "\",\"expected_degrees\":" << analysis.scaleEstimate.expectedAngleDegrees
           << ",\"raw_units_per_degree_per_second\":"
           << analysis.scaleEstimate.rawUnitsPerDegreePerSecond
           << ",\"raw_units_per_radian_per_second\":"
           << analysis.scaleEstimate.rawUnitsPerRadianPerSecond
           << ",\"degrees_per_second_per_raw_unit\":"
           << analysis.scaleEstimate.degreesPerSecondPerRawUnit
           << ",\"radians_per_second_per_raw_unit\":"
           << analysis.scaleEstimate.radiansPerSecondPerRawUnit
           << ",\"direction_matches_request\":"
           << (analysis.scaleEstimate.directionMatchesRequest ? "true" : "false")
           << ",\"confidence\":" << analysis.scaleEstimate.confidence << "},\n"
           << "  \"configuration\":{\"axis\":\""
           << gyroscopeAxisSelectionText(analysis.configuration.axisSelection)
           << "\",\"direction\":\""
           << rotationDirectionText(analysis.configuration.requestedDirection)
           << "\",\"start_threshold_raw\":" << analysis.configuration.startThresholdRaw
           << ",\"stop_threshold_raw\":" << analysis.configuration.stopThresholdRaw
           << ",\"stillness_duration_ns\":"
           << analysis.configuration.stillnessDuration.count()
           << ",\"maximum_device_delta_ns\":"
           << analysis.configuration.maximumDeviceDelta.count() << "},\n"
           << "  \"warnings\":[";
    for (std::size_t index = 0; index < analysis.warnings.size(); ++index)
    {
        if (index != 0U) output << ',';
        output << '"' << escapeJson(analysis.warnings[index]) << '"';
    }
    output << "]\n}\n";
    return output.str();
}

std::string recordingTimestampStatusText(RecordingTimestampStatus status)
{
    switch (status)
    {
    case RecordingTimestampStatus::firstSample: return "first-sample";
    case RecordingTimestampStatus::valid: return "valid";
    case RecordingTimestampStatus::duplicate: return "duplicate";
    case RecordingTimestampStatus::decreasing: return "decreasing";
    case RecordingTimestampStatus::deltaTooLarge: return "delta-too-large";
    }
    return "unknown";
}

std::string gyroscopeRecordingAnalysisFailureText(GyroscopeRecordingAnalysisFailure failure)
{
    switch (failure)
    {
    case GyroscopeRecordingAnalysisFailure::none: return {};
    case GyroscopeRecordingAnalysisFailure::noSamples: return "no decoded samples were recorded";
    case GyroscopeRecordingAnalysisFailure::noUsableSegment: return "no usable motion segment was detected";
    }
    return "unknown analysis failure";
}

std::string gyroscopeAxisSelectionText(GyroscopeAxisSelection selection)
{
    switch (selection)
    {
    case GyroscopeAxisSelection::x: return "x";
    case GyroscopeAxisSelection::y: return "y";
    case GyroscopeAxisSelection::z: return "z";
    case GyroscopeAxisSelection::automatic: return "auto";
    }
    return "unknown";
}

} // namespace xreal::sensors
