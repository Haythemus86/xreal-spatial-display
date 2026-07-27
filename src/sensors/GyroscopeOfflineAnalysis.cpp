#include "sensors/GyroscopeOfflineAnalysis.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <numbers>
#include <numeric>
#include <sstream>
#include <stdexcept>

namespace xreal::sensors
{
namespace
{

using Vector = std::array<double, 3>;

[[nodiscard]] Vector vector(const CorrectedGyroscopeRaw& value) noexcept
{
    return {value.x, value.y, value.z};
}

[[nodiscard]] RawVector3d rawVector(const Vector& value) noexcept
{
    return {value[0], value[1], value[2]};
}

[[nodiscard]] std::size_t axisIndex(GyroscopeAxis axis) noexcept
{
    if (axis == GyroscopeAxis::x)
    {
        return 0U;
    }
    if (axis == GyroscopeAxis::y)
    {
        return 1U;
    }
    return 2U;
}

[[nodiscard]] GyroscopeAxis axisFromIndex(std::size_t index) noexcept
{
    if (index == 0U)
    {
        return GyroscopeAxis::x;
    }
    if (index == 1U)
    {
        return GyroscopeAxis::y;
    }
    return GyroscopeAxis::z;
}

[[nodiscard]] double median(std::vector<double> values)
{
    if (values.empty())
    {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    const std::size_t middle = values.size() / 2U;
    return values.size() % 2U == 0U
        ? (values[middle - 1U] + values[middle]) * 0.5
        : values[middle];
}

[[nodiscard]] GyroscopeStillnessAxisStatistics axisStatistics(
    std::span<const GyroscopeRecordingSample> samples,
    std::size_t begin,
    std::size_t end,
    std::size_t axis)
{
    GyroscopeStillnessAxisStatistics result;
    std::vector<double> valuesForMedian;
    valuesForMedian.reserve(end - begin + 1U);
    double sum{};
    double sumSquared{};
    for (std::size_t index = begin; index <= end; ++index)
    {
        const double value = vector(samples[index].corrected)[axis];
        sum += value;
        sumSquared += value * value;
        result.peakAbsolute = std::max(result.peakAbsolute, std::abs(value));
        valuesForMedian.push_back(value);
    }
    const double count = static_cast<double>(valuesForMedian.size());
    result.mean = sum / count;
    result.rootMeanSquare = std::sqrt(sumSquared / count);
    result.standardDeviation = std::sqrt(std::max(
        0.0, sumSquared / count - result.mean * result.mean));
    const double center = median(valuesForMedian);
    for (double& value : valuesForMedian)
    {
        value = std::abs(value - center);
    }
    result.medianAbsoluteDeviation = median(std::move(valuesForMedian));
    return result;
}

[[nodiscard]] GyroscopeStillnessWindow makeStillnessWindow(
    std::span<const GyroscopeRecordingSample> samples,
    std::size_t begin,
    std::size_t end,
    double maximumRms,
    double maximumPeak)
{
    GyroscopeStillnessWindow window;
    window.found = true;
    window.startSampleIndex = begin;
    window.endSampleIndex = end;
    window.startDeviceTimestamp = samples[begin].original.deviceTimestamp.nanoseconds;
    window.endDeviceTimestamp = samples[end].original.deviceTimestamp.nanoseconds;
    window.duration = std::chrono::nanoseconds(
        samples[end].elapsedDeviceNanoseconds - samples[begin].elapsedDeviceNanoseconds);
    window.x = axisStatistics(samples, begin, end, 0U);
    window.y = axisStatistics(samples, begin, end, 1U);
    window.z = axisStatistics(samples, begin, end, 2U);
    const auto acceptedAxis = [&](const GyroscopeStillnessAxisStatistics& axis) {
        return axis.rootMeanSquare <= maximumRms && axis.peakAbsolute <= maximumPeak;
    };
    window.accepted = acceptedAxis(window.x) && acceptedAxis(window.y)
        && acceptedAxis(window.z);
    return window;
}

[[nodiscard]] GyroscopeStillnessWindow findStillnessWindow(
    std::span<const GyroscopeRecordingSample> samples,
    std::size_t searchBegin,
    std::chrono::nanoseconds requiredDuration,
    double maximumRms,
    double maximumPeak)
{
    if (samples.empty() || searchBegin >= samples.size())
    {
        return {};
    }
    const std::uint64_t required = static_cast<std::uint64_t>(requiredDuration.count());
    std::size_t end = searchBegin;
    for (std::size_t begin = searchBegin; begin < samples.size(); ++begin)
    {
        end = std::max(end, begin);
        while (end + 1U < samples.size()
               && samples[end].elapsedDeviceNanoseconds
                       - samples[begin].elapsedDeviceNanoseconds < required)
        {
            ++end;
        }
        if (samples[end].elapsedDeviceNanoseconds
                - samples[begin].elapsedDeviceNanoseconds < required)
        {
            break;
        }
        const auto candidate = makeStillnessWindow(
            samples, begin, end, maximumRms, maximumPeak);
        if (candidate.accepted)
        {
            return candidate;
        }
    }
    return {};
}

[[nodiscard]] Vector means(const GyroscopeStillnessWindow& window) noexcept
{
    return {window.x.mean, window.y.mean, window.z.mean};
}

[[nodiscard]] Vector deadbands(
    const GyroscopeStillnessWindow& window,
    const GyroscopeOfflineAnalysisConfig& configuration) noexcept
{
    return {
        std::max(configuration.minimumDeadbandRaw,
                 configuration.noiseStandardDeviationMultiplier * window.x.standardDeviation),
        std::max(configuration.minimumDeadbandRaw,
                 configuration.noiseStandardDeviationMultiplier * window.y.standardDeviation),
        std::max(configuration.minimumDeadbandRaw,
                 configuration.noiseStandardDeviationMultiplier * window.z.standardDeviation),
    };
}

[[nodiscard]] std::vector<Vector> deadbandedSamples(
    std::span<const GyroscopeRecordingSample> samples,
    const Vector& deadband,
    GyroscopeDeadbandMode mode)
{
    std::vector<Vector> result;
    result.reserve(samples.size());
    for (const auto& sample : samples)
    {
        auto value = vector(sample.corrected);
        for (std::size_t axis = 0; axis < 3U; ++axis)
        {
            value[axis] = applyGyroscopeDeadband(value[axis], deadband[axis], mode);
        }
        result.push_back(value);
    }
    return result;
}

[[nodiscard]] std::vector<double> movementEnvelope(
    std::span<const GyroscopeRecordingSample> samples,
    std::span<const Vector> signal,
    std::chrono::nanoseconds window)
{
    std::vector<double> envelope(samples.size());
    Vector sumSquared{};
    std::size_t begin{};
    for (std::size_t end = 0; end < samples.size(); ++end)
    {
        for (std::size_t axis = 0; axis < 3U; ++axis)
        {
            sumSquared[axis] += signal[end][axis] * signal[end][axis];
        }
        while (begin < end
               && samples[end].elapsedDeviceNanoseconds
                       - samples[begin].elapsedDeviceNanoseconds
                   > static_cast<std::uint64_t>(window.count()))
        {
            for (std::size_t axis = 0; axis < 3U; ++axis)
            {
                sumSquared[axis] -= signal[begin][axis] * signal[begin][axis];
            }
            ++begin;
        }
        const double count = static_cast<double>(end - begin + 1U);
        double maximumRms{};
        for (std::size_t axis = 0; axis < 3U; ++axis)
        {
            maximumRms = std::max(maximumRms, std::sqrt(std::max(0.0, sumSquared[axis] / count)));
        }
        envelope[end] = maximumRms;
    }
    return envelope;
}

[[nodiscard]] Vector interpolateResidual(
    const Vector& pre,
    const Vector& post,
    double fraction,
    GyroscopeResidualBiasMode mode) noexcept
{
    if (mode == GyroscopeResidualBiasMode::none)
    {
        return {};
    }
    if (mode == GyroscopeResidualBiasMode::preStillness)
    {
        return pre;
    }
    return {
        pre[0] + (post[0] - pre[0]) * fraction,
        pre[1] + (post[1] - pre[1]) * fraction,
        pre[2] + (post[2] - pre[2]) * fraction,
    };
}

struct Integration
{
    Vector raw{};
    Vector deadbanded{};
    Vector residual{};
    Vector peak{};
    Vector energy{};
    std::uint64_t duration{};
};

[[nodiscard]] Integration integrate(
    std::span<const GyroscopeRecordingSample> samples,
    std::span<const Vector> deadbanded,
    std::size_t begin,
    std::size_t end,
    const Vector& preMean,
    const Vector& postMean,
    GyroscopeResidualBiasMode residualMode)
{
    Integration result;
    const double denominator = static_cast<double>(std::max<std::size_t>(1U, end - begin));
    for (std::size_t index = begin; index <= end; ++index)
    {
        const Vector current = vector(samples[index].corrected);
        for (std::size_t axis = 0; axis < 3U; ++axis)
        {
            result.peak[axis] = std::max(result.peak[axis], std::abs(current[axis]));
        }
        if (index == begin || !samples[index].deviceDeltaNanoseconds.has_value()
            || !samples[index].sampleValid || !samples[index - 1U].sampleValid)
        {
            continue;
        }
        const Vector previous = vector(samples[index - 1U].corrected);
        const double previousFraction = static_cast<double>(index - 1U - begin) / denominator;
        const double currentFraction = static_cast<double>(index - begin) / denominator;
        const Vector previousResidual = interpolateResidual(
            preMean, postMean, previousFraction, residualMode);
        const Vector currentResidual = interpolateResidual(
            preMean, postMean, currentFraction, residualMode);
        const double seconds = static_cast<double>(*samples[index].deviceDeltaNanoseconds)
            / 1'000'000'000.0;
        result.duration += *samples[index].deviceDeltaNanoseconds;
        for (std::size_t axis = 0; axis < 3U; ++axis)
        {
            result.raw[axis] += (previous[axis] + current[axis]) * 0.5 * seconds;
            result.deadbanded[axis] += (deadbanded[index - 1U][axis]
                + deadbanded[index][axis]) * 0.5 * seconds;
            const double previousCorrected = previous[axis] - previousResidual[axis];
            const double currentCorrected = current[axis] - currentResidual[axis];
            result.residual[axis] += (previousCorrected + currentCorrected) * 0.5 * seconds;
            result.energy[axis] += (deadbanded[index - 1U][axis] * deadbanded[index - 1U][axis]
                + deadbanded[index][axis] * deadbanded[index][axis]) * 0.5 * seconds;
        }
    }
    return result;
}

[[nodiscard]] GyroscopeScaleMethodEstimate scaleEstimate(double integral, double degrees)
{
    GyroscopeScaleMethodEstimate result;
    const double rawPerDegree = std::abs(integral) / degrees;
    if (!std::isfinite(rawPerDegree) || rawPerDegree <= 0.0)
    {
        return result;
    }
    result.available = true;
    result.rawUnitsPerDegreePerSecond = rawPerDegree;
    result.rawUnitsPerRadianPerSecond = rawPerDegree * 180.0 / std::numbers::pi;
    result.degreesPerSecondPerRawUnit = 1.0 / result.rawUnitsPerDegreePerSecond;
    result.radiansPerSecondPerRawUnit = 1.0 / result.rawUnitsPerRadianPerSecond;
    return result;
}

[[nodiscard]] double clampUnit(double value) noexcept
{
    return std::clamp(value, 0.0, 1.0);
}

[[nodiscard]] std::string escapeJson(std::string_view value)
{
    std::ostringstream output;
    for (const char character : value)
    {
        if (character == '"')
        {
            output << "\\\"";
        }
        else if (character == '\\')
        {
            output << "\\\\";
        }
        else if (character == '\n')
        {
            output << "\\n";
        }
        else
        {
            output << character;
        }
    }
    return output.str();
}

void writeVector(std::ostringstream& output, const RawVector3d& value)
{
    output << "{\"x\":" << value.x << ",\"y\":" << value.y << ",\"z\":" << value.z << '}';
}

void writeStillness(std::ostringstream& output, const GyroscopeStillnessWindow& window)
{
    output << "{\"found\":" << (window.found ? "true" : "false")
           << ",\"accepted\":" << (window.accepted ? "true" : "false")
           << ",\"start_device_timestamp\":" << window.startDeviceTimestamp
           << ",\"end_device_timestamp\":" << window.endDeviceTimestamp
           << ",\"duration_seconds\":" << std::chrono::duration<double>(window.duration).count()
           << ",\"mean_raw\":{" << "\"x\":" << window.x.mean << ",\"y\":" << window.y.mean
           << ",\"z\":" << window.z.mean << "},\"stddev_raw\":{" << "\"x\":"
           << window.x.standardDeviation << ",\"y\":" << window.y.standardDeviation
           << ",\"z\":" << window.z.standardDeviation << "},\"rms_raw\":{" << "\"x\":"
           << window.x.rootMeanSquare << ",\"y\":" << window.y.rootMeanSquare
           << ",\"z\":" << window.z.rootMeanSquare << "},\"peak_raw\":{" << "\"x\":"
           << window.x.peakAbsolute << ",\"y\":" << window.y.peakAbsolute
           << ",\"z\":" << window.z.peakAbsolute << "},\"mad_raw\":{" << "\"x\":"
           << window.x.medianAbsoluteDeviation << ",\"y\":" << window.y.medianAbsoluteDeviation
           << ",\"z\":" << window.z.medianAbsoluteDeviation << "}}";
}

[[nodiscard]] double methodDifference(const std::vector<double>& values)
{
    if (values.size() < 2U)
    {
        return 0.0;
    }
    const auto [minimum, maximum] = std::minmax_element(values.begin(), values.end());
    const double middle = median(values);
    return middle > 0.0 ? (*maximum - *minimum) / middle : 0.0;
}

} // namespace

double applyGyroscopeDeadband(double value, double deadband, GyroscopeDeadbandMode mode) noexcept
{
    if (mode == GyroscopeDeadbandMode::none || deadband <= 0.0)
    {
        return value;
    }
    if (std::abs(value) <= deadband)
    {
        return 0.0;
    }
    if (mode == GyroscopeDeadbandMode::hard)
    {
        return value;
    }
    return std::copysign(std::abs(value) - deadband, value);
}

GyroscopeOfflineAnalysisResult analyzeGyroscopeRecordingOffline(
    std::span<const ImuSample> samples,
    const GyroscopeBias& bias,
    GyroscopeOfflineAnalysisConfig configuration)
{
    if (configuration.preStillnessDuration <= std::chrono::nanoseconds::zero()
        || configuration.postStillnessDuration <= std::chrono::nanoseconds::zero()
        || configuration.envelopeWindow <= std::chrono::nanoseconds::zero()
        || configuration.startSustainDuration <= std::chrono::nanoseconds::zero()
        || configuration.stopSustainDuration <= std::chrono::nanoseconds::zero()
        || configuration.preStillnessMaximumRmsRaw <= 0.0
        || configuration.preStillnessMaximumPeakRaw <= 0.0
        || configuration.postStillnessMaximumRmsRaw <= 0.0
        || configuration.postStillnessMaximumPeakRaw <= 0.0
        || configuration.noiseStandardDeviationMultiplier <= 0.0
        || configuration.minimumDeadbandRaw < 0.0
        || configuration.trimEnvelopeFraction < 0.0
        || configuration.trimEnvelopeFraction > 1.0)
    {
        throw std::invalid_argument("Offline gyroscope analysis configuration is invalid.");
    }

    GyroscopeOfflineAnalysisResult result;
    result.configuration = configuration;
    result.backwardCompatible = analyzeGyroscopeRecording(samples, bias, configuration.base);
    result.warnings = result.backwardCompatible.warnings;
    const auto& recorded = result.backwardCompatible.samples;
    if (recorded.empty())
    {
        return result;
    }

    result.preStillness = findStillnessWindow(
        recorded,
        0U,
        configuration.preStillnessDuration,
        configuration.preStillnessMaximumRmsRaw,
        configuration.preStillnessMaximumPeakRaw);
    if (!result.preStillness.accepted)
    {
        result.warnings.emplace_back("No verified pre-motion stillness window was found; refined segmentation was not attempted.");
        return result;
    }

    const Vector preMean = means(result.preStillness);
    const Vector deadband = deadbands(result.preStillness, configuration);
    result.noiseFloor = {
        rawVector(preMean),
        {result.preStillness.x.standardDeviation, result.preStillness.y.standardDeviation,
         result.preStillness.z.standardDeviation},
        {result.preStillness.x.rootMeanSquare, result.preStillness.y.rootMeanSquare,
         result.preStillness.z.rootMeanSquare},
        {result.preStillness.x.peakAbsolute, result.preStillness.y.peakAbsolute,
         result.preStillness.z.peakAbsolute},
        {result.preStillness.x.medianAbsoluteDeviation,
         result.preStillness.y.medianAbsoluteDeviation,
         result.preStillness.z.medianAbsoluteDeviation},
        rawVector(deadband),
        configuration.deadbandMode,
    };
    const auto deadbanded = deadbandedSamples(recorded, deadband, configuration.deadbandMode);
    result.movementEnvelope = movementEnvelope(
        recorded, deadbanded, configuration.envelopeWindow);

    double totalEnergy{};
    for (std::size_t index = 1U; index < recorded.size(); ++index)
    {
        if (!recorded[index].deviceDeltaNanoseconds.has_value())
        {
            continue;
        }
        const double seconds = static_cast<double>(*recorded[index].deviceDeltaNanoseconds)
            / 1'000'000'000.0;
        for (std::size_t axis = 0; axis < 3U; ++axis)
        {
            totalEnergy += (deadbanded[index - 1U][axis] * deadbanded[index - 1U][axis]
                + deadbanded[index][axis] * deadbanded[index][axis]) * 0.5 * seconds;
        }
    }

    std::size_t index = result.preStillness.endSampleIndex + 1U;
    while (index < recorded.size())
    {
        while (index < recorded.size()
               && result.movementEnvelope[index] < configuration.base.startThresholdRaw)
        {
            ++index;
        }
        if (index >= recorded.size())
        {
            break;
        }
        const std::size_t startSustainBegin = index;
        while (index < recorded.size()
               && result.movementEnvelope[index] >= configuration.base.startThresholdRaw
               && recorded[index].elapsedDeviceNanoseconds
                       - recorded[startSustainBegin].elapsedDeviceNanoseconds
                   < static_cast<std::uint64_t>(configuration.startSustainDuration.count()))
        {
            ++index;
        }
        if (index >= recorded.size()
            || result.movementEnvelope[index] < configuration.base.startThresholdRaw)
        {
            index = startSustainBegin + 1U;
            continue;
        }
        const std::size_t roughStart = startSustainBegin;
        std::size_t roughEnd = recorded.size() - 1U;
        std::size_t stopSearch = index;
        GyroscopeStillnessWindow post;
        while (stopSearch < recorded.size())
        {
            while (stopSearch < recorded.size()
                   && result.movementEnvelope[stopSearch] > configuration.base.stopThresholdRaw)
            {
                ++stopSearch;
            }
            if (stopSearch >= recorded.size())
            {
                break;
            }
            const std::size_t stopBegin = stopSearch;
            while (stopSearch < recorded.size()
                   && result.movementEnvelope[stopSearch] <= configuration.base.stopThresholdRaw
                   && recorded[stopSearch].elapsedDeviceNanoseconds
                           - recorded[stopBegin].elapsedDeviceNanoseconds
                       < static_cast<std::uint64_t>(configuration.stopSustainDuration.count()))
            {
                ++stopSearch;
            }
            if (stopSearch < recorded.size()
                && result.movementEnvelope[stopSearch] > configuration.base.stopThresholdRaw)
            {
                ++stopSearch;
                continue;
            }
            roughEnd = stopBegin > roughStart ? stopBegin : stopSearch;
            post = findStillnessWindow(
                recorded,
                stopBegin,
                configuration.postStillnessDuration,
                configuration.postStillnessMaximumRmsRaw,
                configuration.postStillnessMaximumPeakRaw);
            break;
        }

        std::size_t refinedStart = roughStart;
        std::size_t refinedEnd = roughEnd;
        if (configuration.trimCandidates && refinedEnd > refinedStart)
        {
            const double peakEnvelope = *std::max_element(
                result.movementEnvelope.begin() + static_cast<std::ptrdiff_t>(roughStart),
                result.movementEnvelope.begin() + static_cast<std::ptrdiff_t>(roughEnd + 1U));
            const double trimThreshold = std::max(
                std::max({deadband[0], deadband[1], deadband[2]}),
                peakEnvelope * configuration.trimEnvelopeFraction);
            while (refinedStart < refinedEnd
                   && result.movementEnvelope[refinedStart] < trimThreshold)
            {
                ++refinedStart;
            }
            while (refinedEnd > refinedStart
                   && result.movementEnvelope[refinedEnd] < trimThreshold)
            {
                --refinedEnd;
            }
        }

        const Vector postMean = post.accepted ? means(post) : preMean;
        const auto integrated = integrate(
            recorded,
            deadbanded,
            refinedStart,
            refinedEnd,
            preMean,
            postMean,
            configuration.residualBiasMode);
        const double energySum = integrated.energy[0] + integrated.energy[1] + integrated.energy[2];
        const std::size_t dominant = static_cast<std::size_t>(std::distance(
            integrated.energy.begin(),
            std::max_element(integrated.energy.begin(), integrated.energy.end())));
        const double otherPeak = std::max(
            integrated.peak[(dominant + 1U) % 3U],
            integrated.peak[(dominant + 2U) % 3U]);
        GyroscopeRefinedSegment segment;
        segment.roughStartSampleIndex = roughStart;
        segment.roughEndSampleIndex = roughEnd;
        segment.refinedStartSampleIndex = refinedStart;
        segment.refinedEndSampleIndex = refinedEnd;
        segment.roughStartDeviceTimestamp = recorded[roughStart].original.deviceTimestamp.nanoseconds;
        segment.roughEndDeviceTimestamp = recorded[roughEnd].original.deviceTimestamp.nanoseconds;
        segment.refinedStartDeviceTimestamp = recorded[refinedStart].original.deviceTimestamp.nanoseconds;
        segment.refinedEndDeviceTimestamp = recorded[refinedEnd].original.deviceTimestamp.nanoseconds;
        segment.roughDuration = std::chrono::nanoseconds(
            recorded[roughEnd].elapsedDeviceNanoseconds - recorded[roughStart].elapsedDeviceNanoseconds);
        segment.refinedDuration = std::chrono::nanoseconds(integrated.duration);
        segment.leadingSamplesTrimmed = refinedStart - roughStart;
        segment.trailingSamplesTrimmed = roughEnd - refinedEnd;
        segment.leadingDurationRemoved = std::chrono::nanoseconds(
            recorded[refinedStart].elapsedDeviceNanoseconds - recorded[roughStart].elapsedDeviceNanoseconds);
        segment.trailingDurationRemoved = std::chrono::nanoseconds(
            recorded[roughEnd].elapsedDeviceNanoseconds - recorded[refinedEnd].elapsedDeviceNanoseconds);
        segment.dominantAxis = axisFromIndex(dominant);
        segment.integratedRaw = rawVector(integrated.raw);
        segment.integratedDeadbanded = rawVector(integrated.deadbanded);
        segment.integratedResidualCorrected = rawVector(integrated.residual);
        segment.peakAbsoluteRaw = rawVector(integrated.peak);
        segment.dominantAxisRatio = energySum > 0.0 ? integrated.energy[dominant] / energySum : 0.0;
        segment.crossAxisRatio = integrated.peak[dominant] > 0.0
            ? otherPeak / integrated.peak[dominant] : 0.0;
        segment.rotationalEnergyFraction = totalEnergy > 0.0 ? energySum / totalEnergy : 0.0;
        segment.preStillnessVerified = result.preStillness.accepted;
        segment.postStillnessVerified = post.accepted;
        segment.startEnvelope = result.movementEnvelope[roughStart];
        segment.endEnvelope = result.movementEnvelope[roughEnd];
        const double selectedIntegral = integrated.residual[dominant];
        segment.direction = selectedIntegral > 0.0 ? RotationDirection::positive
            : (selectedIntegral < 0.0 ? RotationDirection::negative : RotationDirection::automatic);
        const double durationSeconds = std::chrono::duration<double>(segment.refinedDuration).count();
        const double residualDifference = std::max({
            std::abs(postMean[0] - preMean[0]), std::abs(postMean[1] - preMean[1]),
            std::abs(postMean[2] - preMean[2])});
        segment.score = {
            segment.preStillnessVerified ? 1.0 : 0.0,
            segment.postStillnessVerified ? 1.0 : 0.0,
            segment.dominantAxisRatio,
            1.0 / (1.0 + segment.crossAxisRatio),
            std::min(durationSeconds, 1.0) * std::min(10.0 / std::max(durationSeconds, 1.0e-9), 1.0),
            1.0,
            static_cast<double>(result.backwardCompatible.validTimestampSamples)
                / static_cast<double>(recorded.size()),
            clampUnit(segment.rotationalEnergyFraction),
            0.5 * clampUnit(segment.startEnvelope / configuration.base.startThresholdRaw)
                + 0.5 * clampUnit(configuration.base.stopThresholdRaw
                    / std::max(segment.endEnvelope, 1.0)),
            1.0 / (1.0 + residualDifference
                / configuration.residualMeanDisagreementWarningRaw),
            0.0,
        };
        segment.score.total = 0.12 * segment.score.preStillness
            + 0.12 * segment.score.postStillness + 0.18 * segment.score.dominantAxis
            + 0.12 * segment.score.crossAxis + 0.08 * segment.score.duration
            + 0.10 * segment.score.sustainedMovement + 0.08 * segment.score.timestampValidity
            + 0.10 * segment.score.energyContainment + 0.05 * segment.score.thresholdQuality
            + 0.05 * segment.score.residualStability;
        result.segments.push_back(segment);
        if (post.accepted && !result.postStillness.accepted)
        {
            result.postStillness = post;
        }
        index = post.accepted ? post.endSampleIndex + 1U : roughEnd + 1U;
    }

    if (result.segments.empty())
    {
        result.warnings.emplace_back("No sustained motion segment followed the verified pre-motion stillness window.");
        return result;
    }
    result.bestSegmentIndex = static_cast<std::size_t>(std::distance(
        result.segments.begin(),
        std::max_element(result.segments.begin(), result.segments.end(),
            [](const auto& left, const auto& right) { return left.score.total < right.score.total; })));
    const auto& best = result.segments[*result.bestSegmentIndex];
    if (!best.postStillnessVerified)
    {
        result.warnings.emplace_back("The selected segment has no verified post-motion stillness window.");
    }
    const Vector postMean = result.postStillness.accepted ? means(result.postStillness) : preMean;
    result.postStillnessMeanRaw = rawVector(postMean);
    const double residualDisagreement = std::max({
        std::abs(postMean[0] - preMean[0]), std::abs(postMean[1] - preMean[1]),
        std::abs(postMean[2] - preMean[2])});
    if (result.postStillness.accepted
        && residualDisagreement > configuration.residualMeanDisagreementWarningRaw)
    {
        result.warnings.emplace_back("Pre- and post-stillness residual means disagree beyond the configured warning threshold.");
    }

    if (configuration.base.expectedAngleDegrees.has_value())
    {
        GyroscopeAxis scaleAxis = best.dominantAxis;
        if (configuration.base.axisSelection != GyroscopeAxisSelection::automatic)
        {
            scaleAxis = axisFromIndex(static_cast<std::size_t>(configuration.base.axisSelection));
        }
        const std::size_t selected = axisIndex(scaleAxis);
        const Vector raw = {best.integratedRaw.x, best.integratedRaw.y, best.integratedRaw.z};
        const Vector dead = {best.integratedDeadbanded.x, best.integratedDeadbanded.y,
                             best.integratedDeadbanded.z};
        const Vector residual = {best.integratedResidualCorrected.x,
                                 best.integratedResidualCorrected.y,
                                 best.integratedResidualCorrected.z};
        result.scaleEstimates.axis = scaleAxis;
        result.scaleEstimates.expectedAngleDegrees = *configuration.base.expectedAngleDegrees;
        result.scaleEstimates.raw = scaleEstimate(raw[selected], *configuration.base.expectedAngleDegrees);
        result.scaleEstimates.deadbanded = scaleEstimate(dead[selected], *configuration.base.expectedAngleDegrees);
        result.scaleEstimates.residualCorrected = scaleEstimate(
            residual[selected], *configuration.base.expectedAngleDegrees);
        if (configuration.residualBiasMode != GyroscopeResidualBiasMode::none
            && result.scaleEstimates.residualCorrected.available)
        {
            result.scaleEstimates.recommendedMethod = "residual-corrected";
            result.scaleEstimates.recommended = result.scaleEstimates.residualCorrected;
        }
        else if (configuration.deadbandMode != GyroscopeDeadbandMode::none
                 && result.scaleEstimates.deadbanded.available)
        {
            result.scaleEstimates.recommendedMethod = "deadbanded";
            result.scaleEstimates.recommended = result.scaleEstimates.deadbanded;
        }
        else
        {
            result.scaleEstimates.recommendedMethod = "raw";
            result.scaleEstimates.recommended = result.scaleEstimates.raw;
        }
        std::vector<double> estimates;
        if (result.scaleEstimates.raw.available)
        {
            estimates.push_back(result.scaleEstimates.raw.rawUnitsPerDegreePerSecond);
        }
        if (result.scaleEstimates.deadbanded.available)
        {
            estimates.push_back(result.scaleEstimates.deadbanded.rawUnitsPerDegreePerSecond);
        }
        if (result.scaleEstimates.residualCorrected.available)
        {
            estimates.push_back(result.scaleEstimates.residualCorrected.rawUnitsPerDegreePerSecond);
        }
        result.scaleEstimates.maximumRelativeMethodDifference = methodDifference(estimates);
        const Vector recommendedSignal = configuration.residualBiasMode != GyroscopeResidualBiasMode::none
            ? residual : (configuration.deadbandMode != GyroscopeDeadbandMode::none ? dead : raw);
        const RotationDirection measured = recommendedSignal[selected] >= 0.0
            ? RotationDirection::positive : RotationDirection::negative;
        result.scaleEstimates.directionMatchesRequest = configuration.base.requestedDirection
                == RotationDirection::automatic
            || configuration.base.requestedDirection == measured;
        result.scaleEstimates.confidence = clampUnit(best.score.total
            * (1.0 - clampUnit(result.scaleEstimates.maximumRelativeMethodDifference)));
        if (result.scaleEstimates.maximumRelativeMethodDifference > 0.2)
        {
            result.warnings.emplace_back("Raw, deadbanded, and residual-corrected scale estimates disagree by more than 20 percent.");
        }
    }
    return result;
}

GyroscopeBatchAnalysisResult aggregateGyroscopeRecordings(
    std::span<const std::string> names,
    std::span<const GyroscopeOfflineAnalysisResult> analyses,
    GyroscopeBatchConfig configuration)
{
    if (names.size() != analyses.size() || configuration.outlierPercent <= 0.0
        || configuration.minimumAcceptedRecordings == 0U
        || configuration.maximumCoefficientOfVariationPercent <= 0.0
        || configuration.maximumDirectionDisagreementPercent <= 0.0
        || configuration.minimumConfidence < 0.0 || configuration.minimumConfidence > 1.0)
    {
        throw std::invalid_argument("Gyroscope batch configuration is invalid.");
    }
    GyroscopeBatchAnalysisResult result;
    std::vector<double> candidates;
    for (std::size_t index = 0; index < analyses.size(); ++index)
    {
        GyroscopeBatchRecordingResult recording;
        recording.inputName = names[index];
        const auto& analysis = analyses[index];
        if (!analysis.bestSegmentIndex.has_value()
            || !analysis.scaleEstimates.recommended.available)
        {
            recording.rejectionReason = "no recommended scale estimate";
        }
        else
        {
            const auto& segment = analysis.segments[*analysis.bestSegmentIndex];
            recording.recommendedScale = analysis.scaleEstimates.recommended.rawUnitsPerDegreePerSecond;
            recording.direction = segment.direction;
            recording.dominantAxis = segment.dominantAxis;
            recording.selectedDurationSeconds = std::chrono::duration<double>(segment.refinedDuration).count();
            recording.crossAxisRatio = segment.crossAxisRatio;
            recording.confidence = analysis.scaleEstimates.confidence;
            if (recording.confidence < configuration.minimumConfidence)
            {
                recording.rejectionReason = "confidence below configured minimum";
            }
            else
            {
                recording.accepted = true;
                candidates.push_back(recording.recommendedScale);
            }
        }
        result.recordings.push_back(recording);
    }
    if (candidates.empty())
    {
        result.rejectionReason = "no recordings produced an eligible estimate";
        result.rejectedCount = result.recordings.size();
        return result;
    }
    const double initialMedian = median(candidates);
    std::vector<double> retained;
    std::vector<double> positive;
    std::vector<double> negative;
    for (auto& recording : result.recordings)
    {
        if (!recording.accepted)
        {
            continue;
        }
        const double deviation = std::abs(recording.recommendedScale - initialMedian)
            / initialMedian * 100.0;
        if (deviation > configuration.outlierPercent)
        {
            recording.accepted = false;
            recording.rejectionReason = "median-relative scale outlier";
            continue;
        }
        retained.push_back(recording.recommendedScale);
        if (recording.direction == RotationDirection::positive)
        {
            positive.push_back(recording.recommendedScale);
        }
        else if (recording.direction == RotationDirection::negative)
        {
            negative.push_back(recording.recommendedScale);
        }
    }
    result.acceptedCount = retained.size();
    result.rejectedCount = result.recordings.size() - retained.size();
    if (retained.size() < configuration.minimumAcceptedRecordings)
    {
        result.rejectionReason = "too few recordings remained after rejection";
        return result;
    }
    result.median = median(retained);
    result.minimum = *std::min_element(retained.begin(), retained.end());
    result.maximum = *std::max_element(retained.begin(), retained.end());
    result.mean = std::accumulate(retained.begin(), retained.end(), 0.0)
        / static_cast<double>(retained.size());
    double squared{};
    for (const double value : retained)
    {
        squared += (value - result.mean) * (value - result.mean);
    }
    result.standardDeviation = std::sqrt(squared / static_cast<double>(retained.size()));
    result.coefficientOfVariation = result.mean > 0.0
        ? result.standardDeviation / result.mean : 0.0;
    if (result.coefficientOfVariation * 100.0
        > configuration.maximumCoefficientOfVariationPercent)
    {
        result.rejectionReason = "batch coefficient of variation was excessive";
        return result;
    }
    if (!positive.empty() && !negative.empty())
    {
        result.directionDisagreementPercent = std::abs(median(positive) - median(negative))
            / result.median * 100.0;
        if (result.directionDisagreementPercent
            > configuration.maximumDirectionDisagreementPercent)
        {
            result.rejectionReason = "opposite rotation directions disagreed";
            return result;
        }
    }
    result.accepted = true;
    return result;
}

std::string serializeGyroscopeOfflineAnalysisJson(
    const GyroscopeRecordingMetadata& metadata,
    const GyroscopeOfflineAnalysisResult& analysis)
{
    std::string output = serializeGyroscopeRecordingAnalysisJson(
        metadata, analysis.backwardCompatible);
    const std::size_t closing = output.rfind("\n}\n");
    if (closing != std::string::npos)
    {
        output.erase(closing);
    }
    std::ostringstream refined;
    refined << std::setprecision(std::numeric_limits<double>::max_digits10)
            << ",\n  \"refined_analysis_schema_version\":1,\n  \"pre_stillness\":";
    writeStillness(refined, analysis.preStillness);
    refined << ",\n  \"post_stillness\":";
    writeStillness(refined, analysis.postStillness);
    refined << ",\n  \"noise_floor\":{\"stddev_raw\":";
    writeVector(refined, analysis.noiseFloor.standardDeviationRaw);
    refined << ",\"deadband_raw\":";
    writeVector(refined, analysis.noiseFloor.deadbandRaw);
    refined << ",\"deadband_mode\":\"" << gyroscopeDeadbandModeText(analysis.noiseFloor.mode)
            << "\"},\n  \"residual_bias\":{\"mode\":\""
            << gyroscopeResidualBiasModeText(analysis.configuration.residualBiasMode)
            << "\",\"pre_mean_raw\":";
    writeVector(refined, analysis.noiseFloor.meanRaw);
    refined << ",\"post_mean_raw\":";
    writeVector(refined, analysis.postStillnessMeanRaw);
    refined << "},\n  \"refined_segments\":[\n";
    for (std::size_t index = 0; index < analysis.segments.size(); ++index)
    {
        const auto& segment = analysis.segments[index];
        refined << "    {\"rough_start_device_timestamp\":" << segment.roughStartDeviceTimestamp
                << ",\"rough_end_device_timestamp\":" << segment.roughEndDeviceTimestamp
                << ",\"refined_start_device_timestamp\":" << segment.refinedStartDeviceTimestamp
                << ",\"refined_end_device_timestamp\":" << segment.refinedEndDeviceTimestamp
                << ",\"rough_duration_seconds\":" << std::chrono::duration<double>(segment.roughDuration).count()
                << ",\"refined_duration_seconds\":" << std::chrono::duration<double>(segment.refinedDuration).count()
                << ",\"leading_samples_trimmed\":" << segment.leadingSamplesTrimmed
                << ",\"trailing_samples_trimmed\":" << segment.trailingSamplesTrimmed
                << ",\"leading_duration_removed_seconds\":"
                << std::chrono::duration<double>(segment.leadingDurationRemoved).count()
                << ",\"trailing_duration_removed_seconds\":"
                << std::chrono::duration<double>(segment.trailingDurationRemoved).count()
                << ",\"dominant_axis\":\"" << gyroscopeAxisText(segment.dominantAxis)
                << "\",\"direction\":\"" << rotationDirectionText(segment.direction)
                << "\",\"integrated_raw\":";
        writeVector(refined, segment.integratedRaw);
        refined << ",\"integrated_deadbanded\":";
        writeVector(refined, segment.integratedDeadbanded);
        refined << ",\"integrated_residual_corrected\":";
        writeVector(refined, segment.integratedResidualCorrected);
        refined << ",\"cross_axis_ratio\":" << segment.crossAxisRatio
                << ",\"energy_fraction\":" << segment.rotationalEnergyFraction
                << ",\"score_breakdown\":{\"pre_stillness\":" << segment.score.preStillness
                << ",\"post_stillness\":" << segment.score.postStillness
                << ",\"dominant_axis\":" << segment.score.dominantAxis
                << ",\"cross_axis\":" << segment.score.crossAxis
                << ",\"duration\":" << segment.score.duration
                << ",\"sustained_movement\":" << segment.score.sustainedMovement
                << ",\"timestamp_validity\":" << segment.score.timestampValidity
                << ",\"energy_containment\":" << segment.score.energyContainment
                << ",\"threshold_quality\":" << segment.score.thresholdQuality
                << ",\"residual_stability\":" << segment.score.residualStability
                << ",\"total\":" << segment.score.total << "}}";
        if (index + 1U != analysis.segments.size())
        {
            refined << ',';
        }
        refined << '\n';
    }
    refined << "  ],\n  \"refined_best_segment_index\":";
    if (analysis.bestSegmentIndex.has_value())
    {
        refined << *analysis.bestSegmentIndex;
    }
    else
    {
        refined << "null";
    }
    refined << ",\n  \"refined_best_segment_selection\":\"highest deterministic weighted score: pre/post stillness 12% each, dominance 18%, cross-axis 12%, duration 8%, sustained movement 10%, timestamp validity 8%, energy containment 10%, thresholds 5%, residual stability 5%\",\n"
            << "  \"scale_estimates\":{\"experimental\":true,\"axis\":\""
            << gyroscopeAxisText(analysis.scaleEstimates.axis)
            << "\",\"expected_degrees\":" << analysis.scaleEstimates.expectedAngleDegrees;
    const auto writeMethod = [&](std::string_view name, const GyroscopeScaleMethodEstimate& estimate) {
        refined << ",\"" << name << "\":{\"available\":" << (estimate.available ? "true" : "false")
                << ",\"raw_units_per_degree_per_second\":" << estimate.rawUnitsPerDegreePerSecond
                << ",\"raw_units_per_radian_per_second\":" << estimate.rawUnitsPerRadianPerSecond << '}';
    };
    writeMethod("raw", analysis.scaleEstimates.raw);
    writeMethod("deadbanded", analysis.scaleEstimates.deadbanded);
    writeMethod("residual_corrected", analysis.scaleEstimates.residualCorrected);
    refined << ",\"recommended_method\":\"" << analysis.scaleEstimates.recommendedMethod
            << "\",\"maximum_relative_method_difference\":"
            << analysis.scaleEstimates.maximumRelativeMethodDifference
            << ",\"confidence\":" << analysis.scaleEstimates.confidence << "},\n"
            << "  \"refined_warnings\":[";
    for (std::size_t index = 0; index < analysis.warnings.size(); ++index)
    {
        if (index != 0U)
        {
            refined << ',';
        }
        refined << '"' << escapeJson(analysis.warnings[index]) << '"';
    }
    refined << "]\n}\n";
    return output + refined.str();
}

std::string serializeGyroscopeBatchAnalysisJson(
    const GyroscopeBatchAnalysisResult& batch,
    std::span<const GyroscopeOfflineAnalysisResult>)
{
    std::ostringstream output;
    output << std::setprecision(std::numeric_limits<double>::max_digits10)
           << "{\n  \"schema_version\":1,\n  \"mode\":\"gyroscope-recording-batch\",\n"
           << "  \"accepted\":" << (batch.accepted ? "true" : "false")
           << ",\n  \"rejection_reason\":\"" << escapeJson(batch.rejectionReason)
           << "\",\n  \"accepted_recordings\":" << batch.acceptedCount
           << ",\n  \"rejected_recordings\":" << batch.rejectedCount
           << ",\n  \"statistics\":{\"median\":" << batch.median << ",\"mean\":" << batch.mean
           << ",\"standard_deviation\":" << batch.standardDeviation
           << ",\"coefficient_of_variation\":" << batch.coefficientOfVariation
           << ",\"minimum\":" << batch.minimum << ",\"maximum\":" << batch.maximum
           << ",\"direction_disagreement_percent\":" << batch.directionDisagreementPercent
           << "},\n  \"recordings\":[\n";
    for (std::size_t index = 0; index < batch.recordings.size(); ++index)
    {
        const auto& recording = batch.recordings[index];
        output << "    {\"input\":\"" << escapeJson(recording.inputName)
               << "\",\"accepted\":" << (recording.accepted ? "true" : "false")
               << ",\"rejection_reason\":\"" << escapeJson(recording.rejectionReason)
               << "\",\"scale\":" << recording.recommendedScale
               << ",\"direction\":\"" << rotationDirectionText(recording.direction)
               << "\",\"dominant_axis\":\"" << gyroscopeAxisText(recording.dominantAxis)
               << "\",\"selected_duration_seconds\":" << recording.selectedDurationSeconds
               << ",\"cross_axis_ratio\":" << recording.crossAxisRatio
               << ",\"confidence\":" << recording.confidence << '}';
        if (index + 1U != batch.recordings.size())
        {
            output << ',';
        }
        output << '\n';
    }
    output << "  ]\n}\n";
    return output.str();
}

std::string gyroscopeDeadbandModeText(GyroscopeDeadbandMode mode)
{
    switch (mode)
    {
    case GyroscopeDeadbandMode::none: return "none";
    case GyroscopeDeadbandMode::hard: return "hard";
    case GyroscopeDeadbandMode::soft: return "soft";
    }
    return "unknown";
}

std::string gyroscopeResidualBiasModeText(GyroscopeResidualBiasMode mode)
{
    switch (mode)
    {
    case GyroscopeResidualBiasMode::none: return "none";
    case GyroscopeResidualBiasMode::preStillness: return "pre";
    case GyroscopeResidualBiasMode::linear: return "linear";
    }
    return "unknown";
}

} // namespace xreal::sensors
