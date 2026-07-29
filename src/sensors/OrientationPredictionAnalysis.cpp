#include "sensors/OrientationPredictionAnalysis.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <numbers>
#include <sstream>
#include <unordered_map>

namespace xreal::sensors
{
namespace
{
[[nodiscard]] std::vector<std::string> parseCsvLine(std::string_view line)
{
    std::vector<std::string> fields;
    std::string field;
    bool quoted{};
    for (std::size_t index = 0; index < line.size(); ++index)
    {
        const char value = line[index];
        if (quoted)
        {
            if (value == '"' && index + 1U < line.size() && line[index + 1U] == '"')
            {
                field += '"';
                ++index;
            }
            else if (value == '"')
            {
                quoted = false;
            }
            else
            {
                field += value;
            }
        }
        else if (value == '"')
        {
            quoted = true;
        }
        else if (value == ',')
        {
            fields.push_back(std::move(field));
            field.clear();
        }
        else if (value != '\r')
        {
            field += value;
        }
    }
    fields.push_back(std::move(field));
    return fields;
}

template <typename Value>
[[nodiscard]] std::optional<Value> parseNumber(std::string_view text)
{
    Value value{};
    const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
    if (text.empty() || parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size())
    {
        return std::nullopt;
    }
    if constexpr (std::is_floating_point_v<Value>)
    {
        if (!std::isfinite(value)) { return std::nullopt; }
    }
    return value;
}

[[nodiscard]] PredictionErrorStatistics statistics(std::vector<double> values)
{
    PredictionErrorStatistics result;
    if (values.empty()) { return result; }
    double sum{};
    double squaredSum{};
    for (const double value : values)
    {
        sum += value;
        squaredSum += value * value;
        result.maximum = std::max(result.maximum, value);
    }
    const double count = static_cast<double>(values.size());
    result.mean = sum / count;
    result.rms = std::sqrt(squaredSum / count);
    std::sort(values.begin(), values.end());
    const std::size_t middle = values.size() / 2U;
    result.median = values.size() % 2U == 0U
        ? (values[middle - 1U] + values[middle]) * 0.5 : values[middle];
    const std::size_t percentileIndex = static_cast<std::size_t>(
        std::ceil(0.95 * static_cast<double>(values.size()))) - 1U;
    result.percentile95 = values[std::min(percentileIndex, values.size() - 1U)];
    return result;
}

[[nodiscard]] std::string escapeJson(std::string_view value)
{
    std::string result;
    for (const char character : value)
    {
        if (character == '\\') { result += "\\\\"; }
        else if (character == '"') { result += "\\\""; }
        else if (character == '\n') { result += "\\n"; }
        else { result += character; }
    }
    return result;
}
} // namespace

OrientationPredictionCsvLoadResult loadOrientationPredictionCsv(std::string_view csv)
{
    OrientationPredictionCsvLoadResult result;
    const std::size_t headerEnd = csv.find('\n');
    if (headerEnd == std::string_view::npos)
    {
        result.error = "CSV header is missing.";
        return result;
    }
    const auto headers = parseCsvLine(csv.substr(0, headerEnd));
    std::unordered_map<std::string, std::size_t> columns;
    for (std::size_t index = 0; index < headers.size(); ++index) { columns[headers[index]] = index; }
    constexpr std::string_view required[]{
        "device_timestamp_ns", "measured_absolute_w", "measured_absolute_x",
        "measured_absolute_y", "measured_absolute_z", "angular_velocity_raw_rad_s_x",
        "angular_velocity_raw_rad_s_y", "angular_velocity_raw_rad_s_z"};
    for (const auto name : required)
    {
        if (!columns.contains(std::string(name)))
        {
            result.error = "Missing required CSV column: " + std::string(name);
            return result;
        }
    }
    const auto field = [&](const std::vector<std::string>& fields, std::string_view name)
        -> std::string_view {
        const auto found = columns.find(std::string(name));
        return found != columns.end() && found->second < fields.size()
            ? std::string_view(fields[found->second]) : std::string_view{};
    };
    std::size_t begin = headerEnd + 1U;
    std::size_t lineNumber = 2U;
    while (begin < csv.size())
    {
        const std::size_t end = csv.find('\n', begin);
        const auto line = csv.substr(begin, end == std::string_view::npos ? csv.size() - begin : end - begin);
        if (!line.empty() && line != "\r")
        {
            const auto fields = parseCsvLine(line);
            const auto timestamp = parseNumber<std::uint64_t>(field(fields, "device_timestamp_ns"));
            const auto w = parseNumber<double>(field(fields, "measured_absolute_w"));
            const auto x = parseNumber<double>(field(fields, "measured_absolute_x"));
            const auto y = parseNumber<double>(field(fields, "measured_absolute_y"));
            const auto z = parseNumber<double>(field(fields, "measured_absolute_z"));
            const auto velocityX = parseNumber<double>(field(fields, "angular_velocity_raw_rad_s_x"));
            const auto velocityY = parseNumber<double>(field(fields, "angular_velocity_raw_rad_s_y"));
            const auto velocityZ = parseNumber<double>(field(fields, "angular_velocity_raw_rad_s_z"));
            if (!timestamp || !w || !x || !y || !z || !velocityX || !velocityY || !velocityZ)
            {
                result.error = "Malformed required numeric field at CSV line "
                    + std::to_string(lineNumber) + '.';
                result.samples.clear();
                return result;
            }
            if (!result.samples.empty() && *timestamp <= result.samples.back().deviceTimestampNanoseconds)
            {
                result.error = "Non-monotonic device timestamp at CSV line "
                    + std::to_string(lineNumber) + '.';
                result.samples.clear();
                return result;
            }
            std::uint64_t generation{};
            const auto generationText = field(fields, "recenter_generation");
            if (!generationText.empty())
            {
                const auto parsed = parseNumber<std::uint64_t>(generationText);
                if (!parsed)
                {
                    result.error = "Malformed recenter generation at CSV line "
                        + std::to_string(lineNumber) + '.';
                    result.samples.clear();
                    return result;
                }
                generation = *parsed;
            }
            const Quaternion orientation{*w, *x, *y, *z};
            const auto normalized = orientation.normalized();
            if (!normalized)
            {
                result.error = "Invalid measured quaternion at CSV line "
                    + std::to_string(lineNumber) + '.';
                result.samples.clear();
                return result;
            }
            result.samples.push_back({*timestamp, {*normalized}, {*velocityX, *velocityY, *velocityZ},
                                      generation, std::string(field(fields, "phase"))});
        }
        if (end == std::string_view::npos) { break; }
        begin = end + 1U;
        ++lineNumber;
    }
    if (result.samples.empty()) { result.error = "CSV contains no prediction samples."; }
    return result;
}

OrientationPredictionMultiHorizonAnalysis analyzeOrientationPredictionHorizons(
    std::span<const OrientationPredictionAnalysisSample> samples,
    std::span<const double> horizonsMilliseconds,
    const OrientationPredictorConfig& baseConfiguration,
    std::chrono::nanoseconds tolerance)
{
    OrientationPredictionMultiHorizonAnalysis output;
    for (const double horizonMilliseconds : horizonsMilliseconds)
    {
        OrientationPredictorConfig configuration = baseConfiguration;
        configuration.horizon = std::chrono::duration<double>(horizonMilliseconds / 1000.0);
        OrientationPredictor predictor(configuration);
        OrientationPredictionHorizonAnalysis result;
        result.horizonMilliseconds = horizonMilliseconds;
        result.mode = configuration.mode;
        result.inputSampleCount = samples.size();
        std::vector<double> predictedTotal;
        std::vector<double> baselineTotal;
        std::vector<double> predictedTilt;
        std::vector<double> baselineTilt;
        double improvementSum{};
        double improvementPercentSum{};
        std::uint64_t improvementPercentCount{};
        std::uint64_t improved{};
        for (std::size_t index = 0; index < samples.size(); ++index)
        {
            const auto& sample = samples[index];
            const auto prediction = predictor.predict({sample.measuredAbsolute, {{}, false},
                sample.bodyAngularVelocity, sample.deviceTimestampNanoseconds});
            ++result.eligiblePredictionCount;
            if (prediction.validity != PredictionValidity::valid)
            {
                ++result.rejectedPredictionCount;
                continue;
            }
            if (prediction.horizon.clamped || prediction.diagnostics.speedClamped
                || prediction.diagnostics.accelerationClamped || prediction.diagnostics.angleClamped)
            {
                ++result.clampedPredictionCount;
            }
            const auto horizonNanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(
                prediction.horizon.applied).count();
            const std::uint64_t target = sample.deviceTimestampNanoseconds
                + static_cast<std::uint64_t>(std::max<std::int64_t>(0, horizonNanoseconds));
            const auto first = samples.begin() + static_cast<std::ptrdiff_t>(index);
            const auto found = std::lower_bound(first, samples.end(), target,
                [](const OrientationPredictionAnalysisSample& candidate, std::uint64_t value) {
                    return candidate.deviceTimestampNanoseconds < value;
                });
            std::optional<std::size_t> matchedIndex;
            const auto consider = [&](std::size_t candidateIndex) {
                const auto& candidate = samples[candidateIndex];
                if (candidate.recenterGeneration != sample.recenterGeneration) { return; }
                const std::uint64_t difference = candidate.deviceTimestampNanoseconds >= target
                    ? candidate.deviceTimestampNanoseconds - target
                    : target - candidate.deviceTimestampNanoseconds;
                if (difference <= static_cast<std::uint64_t>(std::max<std::int64_t>(0, tolerance.count()))
                    && (!matchedIndex.has_value()
                        || difference < (samples[*matchedIndex].deviceTimestampNanoseconds >= target
                            ? samples[*matchedIndex].deviceTimestampNanoseconds - target
                            : target - samples[*matchedIndex].deviceTimestampNanoseconds)))
                {
                    matchedIndex = candidateIndex;
                }
            };
            if (found != samples.end()) { consider(static_cast<std::size_t>(found - samples.begin())); }
            if (found != first) { consider(static_cast<std::size_t>((found - samples.begin()) - 1)); }
            if (!matchedIndex.has_value())
            {
                ++result.unmatchedPredictionCount;
                continue;
            }
            const auto& future = samples[*matchedIndex].measuredAbsolute.value;
            const auto predictionTotalDistance = quaternionAngularDistance(prediction.absolute.value, future);
            const auto baselineTotalDistance = quaternionAngularDistance(sample.measuredAbsolute.value, future);
            const auto predictionTiltDistance = quaternionTiltDistance(prediction.absolute.value, future);
            const auto baselineTiltDistance = quaternionTiltDistance(sample.measuredAbsolute.value, future);
            if (!predictionTotalDistance || !baselineTotalDistance
                || !predictionTiltDistance || !baselineTiltDistance)
            {
                ++result.unmatchedPredictionCount;
                continue;
            }
            ++result.matchedPredictionCount;
            predictedTotal.push_back(predictionTotalDistance->degrees);
            baselineTotal.push_back(baselineTotalDistance->degrees);
            predictedTilt.push_back(predictionTiltDistance->degrees);
            baselineTilt.push_back(baselineTiltDistance->degrees);
            const double improvement = baselineTotalDistance->degrees - predictionTotalDistance->degrees;
            improvementSum += improvement;
            if (improvement > 0.0) { ++improved; }
            if (baselineTotalDistance->degrees > std::numeric_limits<double>::epsilon())
            {
                improvementPercentSum += improvement * 100.0 / baselineTotalDistance->degrees;
                ++improvementPercentCount;
            }
        }
        result.predictedTotal = statistics(std::move(predictedTotal));
        result.unpredictedTotal = statistics(std::move(baselineTotal));
        result.predictedTilt = statistics(std::move(predictedTilt));
        result.unpredictedTilt = statistics(std::move(baselineTilt));
        if (result.matchedPredictionCount != 0U)
        {
            result.meanImprovementDegrees = improvementSum
                / static_cast<double>(result.matchedPredictionCount);
            result.improvedRatio = static_cast<double>(improved)
                / static_cast<double>(result.matchedPredictionCount);
        }
        if (improvementPercentCount != 0U)
        {
            result.meanImprovementPercent = improvementPercentSum
                / static_cast<double>(improvementPercentCount);
        }
        output.horizons.push_back(result);
    }
    const auto best = [&](auto projection, bool maximum) -> std::optional<double> {
        const OrientationPredictionHorizonAnalysis* selected{};
        for (const auto& value : output.horizons)
        {
            if (value.matchedPredictionCount == 0U) { continue; }
            if (selected == nullptr || (maximum ? projection(value) > projection(*selected)
                                                : projection(value) < projection(*selected)))
            {
                selected = &value;
            }
        }
        return selected == nullptr ? std::nullopt
            : std::optional<double>(selected->horizonMilliseconds);
    };
    output.bestTotalErrorHorizonMilliseconds = best(
        [](const auto& value) { return value.predictedTotal.rms; }, false);
    output.bestTiltErrorHorizonMilliseconds = best(
        [](const auto& value) { return value.predictedTilt.rms; }, false);
    output.bestImprovementHorizonMilliseconds = best(
        [](const auto& value) { return value.meanImprovementDegrees; }, true);
    return output;
}

std::string orientationPredictionAnalysisCsvHeader()
{
    return "prediction_horizon_ms,prediction_mode,input_sample_count,eligible_prediction_count,"
           "matched_prediction_count,unmatched_prediction_count,mean_unpredicted_error_degrees,"
           "rms_unpredicted_error_degrees,maximum_unpredicted_error_degrees,"
           "mean_predicted_error_degrees,rms_predicted_error_degrees,maximum_predicted_error_degrees,"
           "mean_unpredicted_tilt_error_degrees,rms_unpredicted_tilt_error_degrees,"
           "mean_predicted_tilt_error_degrees,rms_predicted_tilt_error_degrees,"
           "mean_improvement_degrees,mean_improvement_percent,improved_ratio,"
           "rejected_prediction_count,clamped_prediction_count";
}

std::string serializeOrientationPredictionAnalysisCsv(
    const OrientationPredictionMultiHorizonAnalysis& analysis)
{
    std::ostringstream output;
    output.imbue(std::locale::classic());
    output << orientationPredictionAnalysisCsvHeader() << '\n'
           << std::setprecision(std::numeric_limits<double>::max_digits10);
    for (const auto& value : analysis.horizons)
    {
        output << value.horizonMilliseconds << ",\"" << predictionModeText(value.mode) << "\","
               << value.inputSampleCount << ',' << value.eligiblePredictionCount << ','
               << value.matchedPredictionCount << ',' << value.unmatchedPredictionCount << ','
               << value.unpredictedTotal.mean << ',' << value.unpredictedTotal.rms << ','
               << value.unpredictedTotal.maximum << ',' << value.predictedTotal.mean << ','
               << value.predictedTotal.rms << ',' << value.predictedTotal.maximum << ','
               << value.unpredictedTilt.mean << ',' << value.unpredictedTilt.rms << ','
               << value.predictedTilt.mean << ',' << value.predictedTilt.rms << ','
               << value.meanImprovementDegrees << ',' << value.meanImprovementPercent << ','
               << value.improvedRatio << ',' << value.rejectedPredictionCount << ','
               << value.clampedPredictionCount << '\n';
    }
    return output.str();
}

std::string serializeOrientationPredictionAnalysisJson(
    const OrientationPredictionMultiHorizonAnalysis& analysis,
    const OrientationPredictorConfig& configuration,
    std::string_view inputPath,
    std::uint64_t inputSampleCount)
{
    std::ostringstream output;
    output.imbue(std::locale::classic());
    output << std::setprecision(std::numeric_limits<double>::max_digits10)
           << "{\n  \"schema_version\":1,\n  \"analyzer_type\":\"orientation-prediction-multi-horizon\","
           << "\n  \"experimental\":true,\n  \"input\":{\"path\":\"" << escapeJson(inputPath)
           << "\",\"sample_count\":" << inputSampleCount << "},\n  \"configuration\":{\"mode\":\""
           << predictionModeText(configuration.mode) << "\",\"limit_behavior\":\""
           << (configuration.limitBehavior == PredictionLimitBehavior::clamp ? "clamp" : "reject")
           << "\"},\n  \"results\":[";
    for (std::size_t index = 0; index < analysis.horizons.size(); ++index)
    {
        const auto& value = analysis.horizons[index];
        if (index != 0U) { output << ','; }
        output << "{\"horizon_ms\":" << value.horizonMilliseconds
               << ",\"matched\":" << value.matchedPredictionCount
               << ",\"unmatched\":" << value.unmatchedPredictionCount
               << ",\"predicted_total\":{\"mean\":" << value.predictedTotal.mean
               << ",\"rms\":" << value.predictedTotal.rms << ",\"maximum\":"
               << value.predictedTotal.maximum << ",\"median\":" << value.predictedTotal.median
               << ",\"percentile_95\":" << value.predictedTotal.percentile95 << "},"
               << "\"unpredicted_total\":{\"mean\":" << value.unpredictedTotal.mean
               << ",\"rms\":" << value.unpredictedTotal.rms << "},"
               << "\"predicted_tilt_rms\":" << value.predictedTilt.rms
               << ",\"unpredicted_tilt_rms\":" << value.unpredictedTilt.rms
               << ",\"mean_improvement_degrees\":" << value.meanImprovementDegrees
               << ",\"improved_ratio\":" << value.improvedRatio << '}';
    }
    const auto optional = [&](const std::optional<double>& value) {
        if (value.has_value()) { output << *value; } else { output << "null"; }
    };
    output << "],\n  \"best_horizon_for_this_dataset\":{\"lowest_rms_total_error_ms\":";
    optional(analysis.bestTotalErrorHorizonMilliseconds);
    output << ",\"lowest_rms_tilt_error_ms\":";
    optional(analysis.bestTiltErrorHorizonMilliseconds);
    output << ",\"highest_mean_improvement_ms\":";
    optional(analysis.bestImprovementHorizonMilliseconds);
    output << "},\n  \"limitations\":[\"dataset-specific result\",\"no motion-to-photon measurement\","
              "\"gyro scale and mappings remain experimental\"]\n}\n";
    return output.str();
}

} // namespace xreal::sensors
