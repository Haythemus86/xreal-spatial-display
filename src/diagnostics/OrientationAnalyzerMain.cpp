#include "sensors/OrientationPredictionAnalysis.hpp"

#include <charconv>
#include <cmath>
#include <fstream>
#include <iostream>
#include <numbers>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace
{
struct Options
{
    bool evaluate{};
    std::string input;
    std::vector<double> horizons{0, 5, 10, 15, 20, 30};
    xreal::sensors::OrientationPredictorConfig predictor;
    double toleranceMilliseconds{2.0};
    std::string jsonOutput;
    std::string csvOutput;
};

[[nodiscard]] std::optional<double> number(std::string_view text, bool allowZero)
{
    double value{};
    const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
    if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size()
        || !std::isfinite(value) || value < 0.0 || (!allowZero && value == 0.0))
    {
        return std::nullopt;
    }
    return value;
}

[[nodiscard]] std::optional<std::vector<double>> horizons(std::string_view text)
{
    std::vector<double> result;
    std::size_t begin{};
    while (begin < text.size())
    {
        const std::size_t comma = text.find(',', begin);
        const auto value = number(text.substr(begin,
            comma == std::string_view::npos ? text.size() - begin : comma - begin), true);
        if (!value.has_value()) { return std::nullopt; }
        result.push_back(*value);
        if (comma == std::string_view::npos) { break; }
        begin = comma + 1U;
    }
    if (result.empty()) { return std::nullopt; }
    return result;
}

void usage()
{
    std::cout << "Usage: xreal-orientation-analyzer --evaluate-orientation-prediction"
                 " --prediction-input-csv <file> --prediction-horizons-ms <csv>"
                 " --prediction-mode <constant-velocity|constant-acceleration>"
                 " [--prediction-evaluation-tolerance-ms <value>]"
                 " [--prediction-angular-velocity-smoothing-seconds <value>]"
                 " [--prediction-angular-acceleration-smoothing-seconds <value>]"
                 " [--prediction-max-angular-speed-dps <value>]"
                 " [--prediction-max-angular-acceleration-dps2 <value>]"
                 " [--prediction-max-angle-degrees <value>]"
                 " [--prediction-limit-behavior <reject|clamp>]"
                 " --prediction-evaluation-json <file> --prediction-evaluation-csv <file>\n";
}

[[nodiscard]] std::optional<Options> parse(int argc, char* argv[], std::string& error)
{
    Options options;
    constexpr double degreesToRadians = std::numbers::pi / 180.0;
    for (int index = 1; index < argc; ++index)
    {
        const std::string_view argument(argv[index]);
        if (argument == "--help" || argument == "-h") { usage(); return std::nullopt; }
        if (argument == "--evaluate-orientation-prediction") { options.evaluate = true; continue; }
        if (index + 1 >= argc) { error = "Missing value for " + std::string(argument); return std::nullopt; }
        const std::string_view value(argv[++index]);
        if (argument == "--prediction-input-csv") { options.input = value; }
        else if (argument == "--prediction-evaluation-json") { options.jsonOutput = value; }
        else if (argument == "--prediction-evaluation-csv") { options.csvOutput = value; }
        else if (argument == "--prediction-horizons-ms")
        {
            const auto parsed = horizons(value);
            if (!parsed) { error = "Invalid --prediction-horizons-ms list."; return std::nullopt; }
            options.horizons = *parsed;
        }
        else if (argument == "--prediction-mode")
        {
            if (value == "constant-velocity") { options.predictor.mode = xreal::sensors::PredictionMode::constantVelocity; }
            else if (value == "constant-acceleration") { options.predictor.mode = xreal::sensors::PredictionMode::constantAcceleration; }
            else { error = "Invalid prediction mode."; return std::nullopt; }
        }
        else if (argument == "--prediction-limit-behavior")
        {
            if (value == "reject") { options.predictor.limitBehavior = xreal::sensors::PredictionLimitBehavior::reject; }
            else if (value == "clamp") { options.predictor.limitBehavior = xreal::sensors::PredictionLimitBehavior::clamp; }
            else { error = "Invalid prediction limit behavior."; return std::nullopt; }
        }
        else
        {
            const bool allowZero = argument == "--prediction-angular-velocity-smoothing-seconds"
                || argument == "--prediction-angular-acceleration-smoothing-seconds";
            const auto parsed = number(value, allowZero);
            if (!parsed) { error = "Invalid numeric value for " + std::string(argument); return std::nullopt; }
            if (argument == "--prediction-evaluation-tolerance-ms") { options.toleranceMilliseconds = *parsed; }
            else if (argument == "--prediction-angular-velocity-smoothing-seconds") { options.predictor.angularVelocitySmoothingTimeConstant = std::chrono::duration<double>(*parsed); }
            else if (argument == "--prediction-angular-acceleration-smoothing-seconds") { options.predictor.angularAccelerationSmoothingTimeConstant = std::chrono::duration<double>(*parsed); }
            else if (argument == "--prediction-max-angular-speed-dps") { options.predictor.maximumAngularSpeedRadiansPerSecond = *parsed * degreesToRadians; }
            else if (argument == "--prediction-max-angular-acceleration-dps2") { options.predictor.maximumAngularAccelerationRadiansPerSecondSquared = *parsed * degreesToRadians; }
            else if (argument == "--prediction-max-angle-degrees") { options.predictor.maximumPredictionAngleRadians = *parsed * degreesToRadians; }
            else { error = "Unknown option: " + std::string(argument); return std::nullopt; }
        }
    }
    if (!options.evaluate || options.input.empty() || options.jsonOutput.empty() || options.csvOutput.empty())
    {
        error = "Evaluation mode, input CSV, JSON output and CSV output are required.";
        return std::nullopt;
    }
    for (const double horizon : options.horizons)
    {
        if (horizon > options.predictor.maximumHorizon.count() * 1000.0
            && options.predictor.limitBehavior == xreal::sensors::PredictionLimitBehavior::reject)
        {
            error = "A requested horizon exceeds the predictor maximum.";
            return std::nullopt;
        }
    }
    return options;
}
} // namespace

int main(int argc, char* argv[])
{
    std::string error;
    const auto options = parse(argc, argv, error);
    if (!options.has_value())
    {
        if (!error.empty()) { std::cerr << "Error: " << error << '\n'; usage(); return 2; }
        return 0;
    }
    std::ifstream input(options->input);
    if (!input) { std::cerr << "Failed to open prediction input CSV: " << options->input << '\n'; return 1; }
    std::ostringstream content;
    content << input.rdbuf();
    const auto loaded = xreal::sensors::loadOrientationPredictionCsv(content.str());
    if (!loaded.error.empty()) { std::cerr << "Invalid prediction input CSV: " << loaded.error << '\n'; return 1; }
    const auto analysis = xreal::sensors::analyzeOrientationPredictionHorizons(
        loaded.samples, options->horizons, options->predictor,
        std::chrono::nanoseconds(static_cast<std::int64_t>(
            options->toleranceMilliseconds * 1'000'000.0)));
    std::ofstream csv(options->csvOutput, std::ios::out | std::ios::trunc);
    if (!csv) { std::cerr << "Failed to open evaluation CSV: " << options->csvOutput << '\n'; return 1; }
    csv << xreal::sensors::serializeOrientationPredictionAnalysisCsv(analysis);
    if (!csv) { std::cerr << "Failed while writing evaluation CSV.\n"; return 1; }
    std::ofstream json(options->jsonOutput, std::ios::out | std::ios::trunc);
    if (!json) { std::cerr << "Failed to open evaluation JSON: " << options->jsonOutput << '\n'; return 1; }
    json << xreal::sensors::serializeOrientationPredictionAnalysisJson(
        analysis, options->predictor, options->input, loaded.samples.size());
    if (!json) { std::cerr << "Failed while writing evaluation JSON.\n"; return 1; }
    std::cout << "Orientation prediction multi-horizon analysis\n";
    for (const auto& result : analysis.horizons)
    {
        std::cout << "  horizon=" << result.horizonMilliseconds << " ms matched="
                  << result.matchedPredictionCount << " rms_predicted="
                  << result.predictedTotal.rms << " deg mean_improvement="
                  << result.meanImprovementDegrees << " deg\n";
    }
    return 0;
}
