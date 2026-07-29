#include "sensors/LiveOrientationPrediction.hpp"
#include "sensors/OrientationPredictionAnalysis.hpp"
#include "JsonSyntaxParser.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numbers>
#include <string>
#include <vector>

namespace
{
int failures{};
void expect(bool condition, const char* message)
{
    if (!condition) { std::cerr << "FAILED: " << message << '\n'; ++failures; }
}
[[nodiscard]] xreal::sensors::Quaternion yaw(double radians)
{
    return {std::cos(radians * 0.5), 0, 0, std::sin(radians * 0.5)};
}
[[nodiscard]] std::vector<xreal::sensors::OrientationPredictionAnalysisSample> motion()
{
    std::vector<xreal::sensors::OrientationPredictionAnalysisSample> samples;
    for (std::uint64_t index = 0; index <= 100U; ++index)
    {
        const double seconds = static_cast<double>(index) * 0.001;
        samples.push_back({index * 1'000'000U, {yaw(seconds)}, {0, 0, 1}, 0, "motion"});
    }
    return samples;
}
[[nodiscard]] std::string minimalCsv(bool negateAlternating = false)
{
    std::string csv = "device_timestamp_ns,measured_absolute_w,measured_absolute_x,"
                      "measured_absolute_y,measured_absolute_z,angular_velocity_raw_rad_s_x,"
                      "angular_velocity_raw_rad_s_y,angular_velocity_raw_rad_s_z,"
                      "recenter_generation,phase\n";
    for (std::uint64_t index = 0; index < 3; ++index)
    {
        auto q = yaw(static_cast<double>(index) * 0.001);
        if (negateAlternating && index == 1U) { q = {-q.w, -q.x, -q.y, -q.z}; }
        csv += std::to_string(index * 1'000'000U) + ',' + std::to_string(q.w)
            + ",0,0," + std::to_string(q.z) + ",0,0,1,,\"motion\"\n";
    }
    return csv;
}

void testCsvLoading()
{
    using namespace xreal::sensors;
    const auto valid = loadOrientationPredictionCsv(minimalCsv());
    expect(valid.error.empty() && valid.samples.size() == 3U,
           "valid CSV and blank optional recenter field load");
    const auto signs = loadOrientationPredictionCsv(minimalCsv(true));
    expect(signs.error.empty(), "q and -q samples both load");
    auto missing = minimalCsv();
    missing.replace(0, std::string("device_timestamp_ns").size(), "wrong_timestamp");
    expect(loadOrientationPredictionCsv(missing).error.find("Missing required") != std::string::npos,
           "missing required column fails clearly");
    auto malformed = minimalCsv();
    const auto position = malformed.find("1000000");
    malformed.replace(position, 7, "invalid");
    expect(loadOrientationPredictionCsv(malformed).error.find("Malformed") != std::string::npos,
           "malformed number fails clearly");
    auto nonMonotonic = minimalCsv();
    const auto last = nonMonotonic.rfind("2000000");
    nonMonotonic.replace(last, 7, "1000000");
    expect(loadOrientationPredictionCsv(nonMonotonic).error.find("Non-monotonic") != std::string::npos,
           "non-monotonic timestamp is reported");
}

void testMultiHorizonAnalysis()
{
    using namespace xreal::sensors;
    const auto samples = motion();
    const double horizons[]{0, 5, 10, 30};
    const auto analysis = analyzeOrientationPredictionHorizons(
        samples, horizons, {}, std::chrono::microseconds(100));
    expect(analysis.horizons.size() == 4U, "multiple horizons stay independent");
    expect(analysis.horizons[0].predictedTotal.rms < 1.0e-6
               && analysis.horizons[0].unpredictedTotal.rms < 1.0e-6,
           "zero horizon equals unpredicted baseline");
    expect(analysis.horizons[2].predictedTotal.rms < 1.0e-5
               && analysis.horizons[2].unpredictedTotal.mean > 0.5,
           "correct positive horizon reduces constant-rate error");
    expect(analysis.horizons[3].unmatchedPredictionCount > analysis.horizons[1].unmatchedPredictionCount,
           "longer horizon reports more insufficient future samples");
    expect(analysis.bestTotalErrorHorizonMilliseconds.has_value(),
           "best dataset horizon is selected deterministically");
    for (const auto& result : analysis.horizons)
    {
        expect(std::isfinite(result.predictedTotal.rms)
                   && std::isfinite(result.meanImprovementDegrees),
               "all analysis outputs remain finite");
    }
    const auto csv = serializeOrientationPredictionAnalysisCsv(analysis);
    expect(csv.starts_with(orientationPredictionAnalysisCsvHeader())
               && static_cast<std::size_t>(std::count(csv.begin(), csv.end(), '\n')) == 5U,
           "summary CSV has one parseable row per horizon");
    const auto json = serializeOrientationPredictionAnalysisJson(analysis, {}, "input\".csv", samples.size());
    expect(JsonSyntaxParser(json).valid()
               && json.find("best_horizon_for_this_dataset") != std::string::npos
               && json.find("input\\\".csv") != std::string::npos,
           "analysis JSON has valid boundaries, escaped input, and dataset-specific wording");
}

void testLiveOrchestration()
{
    using namespace xreal::sensors;
    LiveOrientationPredictionConfig config;
    config.predictor.horizon = std::chrono::milliseconds(10);
    config.evaluateDelayed = true;
    config.evaluator.tolerance = std::chrono::nanoseconds::zero();
    LiveOrientationPredictionSession session(config);
    LiveOrientationPredictionInput input;
    input.fusedOrientationValid = true;
    input.bodyAngularVelocity = {0, 0, 1};
    auto first = session.consume(input);
    expect(first.has_value() && first->prediction.validity == PredictionValidity::valid,
           "valid fused measured sample produces prediction");
    input.fusedOrientationValid = false;
    expect(!session.consume(input).has_value(), "invalid fused sample skips prediction");
    input.fusedOrientationValid = true;
    input.deviceTimestampNanoseconds = 10'000'000;
    input.measuredAbsolute = {yaw(0.010)};
    input.measuredRelative = {input.measuredAbsolute.value};
    auto evaluated = session.consume(input);
    expect(evaluated && evaluated->delayedEvaluation.has_value(),
           "future fused measurement completes delayed evaluation");

    LiveOrientationPredictionConfig rejectConfig;
    rejectConfig.predictor.maximumAngularSpeedRadiansPerSecond = 0.1;
    LiveOrientationPredictionSession rejectSession(rejectConfig);
    input.bodyAngularVelocity = {0, 0, 1};
    const auto rejected = rejectSession.consume(input);
    expect(rejected && rejected->prediction.validity == PredictionValidity::rejected
               && quaternionAngularDistance(rejected->measuredAbsolute.value,
                      rejected->prediction.absolute.value)->degrees < 1.0e-9,
           "prediction rejection preserves measured orientation");
}
} // namespace

int main()
{
    testCsvLoading();
    testMultiHorizonAnalysis();
    testLiveOrchestration();
    if (failures != 0) { return 1; }
    std::cout << "All orientation prediction analysis tests passed.\n";
    return 0;
}
