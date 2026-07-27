#pragma once

#include "sensors/GyroscopeOfflineAnalysis.hpp"

#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace xreal::diagnostics
{

struct GyroRecordingAnalyzerOptions
{
    std::vector<std::string> inputPaths;
    std::string outputPath;
    sensors::GyroscopeOfflineAnalysisConfig analysis;
    sensors::GyroscopeBatchConfig batch;
    std::optional<double> biasX;
    std::optional<double> biasY;
    std::optional<double> biasZ;
};

struct GyroRecordingAnalyzerOptionResult
{
    std::optional<GyroRecordingAnalyzerOptions> options;
    std::string error;
    bool showHelp{};
};

[[nodiscard]] GyroRecordingAnalyzerOptionResult parseGyroRecordingAnalyzerOptions(
    std::span<const std::string_view> arguments);

[[nodiscard]] std::string gyroRecordingAnalyzerUsage();

} // namespace xreal::diagnostics
