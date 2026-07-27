#include "diagnostics/GyroRecordingAnalyzerOptions.hpp"

#include <array>
#include <iostream>
#include <string_view>

namespace
{

int failures{};

void expect(bool condition, std::string_view description)
{
    if (!condition)
    {
        ++failures;
        std::cerr << "FAILED: " << description << '\n';
    }
}

template <std::size_t Size>
[[nodiscard]] auto parse(const std::array<std::string_view, Size>& arguments)
{
    return xreal::diagnostics::parseGyroRecordingAnalyzerOptions(arguments);
}

void testValidSingleAndBatch()
{
    constexpr std::array single{
        std::string_view("--input"), std::string_view("recording.csv"),
        std::string_view("--axis"), std::string_view("auto"),
        std::string_view("--expected-degrees"), std::string_view("360"),
        std::string_view("--direction"), std::string_view("auto"),
        std::string_view("--deadband-mode"), std::string_view("soft"),
        std::string_view("--residual-bias-mode"), std::string_view("linear"),
        std::string_view("--pre-stillness-seconds"), std::string_view("1"),
        std::string_view("--post-stillness-seconds"), std::string_view("1"),
        std::string_view("--envelope-window-ms"), std::string_view("50"),
        std::string_view("--output"), std::string_view("analysis.json")};
    const auto result = parse(single);
    expect(result.options.has_value(), "valid single-file analyzer options are accepted");
    expect(result.options->analysis.base.axisSelection
               == xreal::sensors::GyroscopeAxisSelection::automatic,
           "axis auto is parsed without hardware mode");
    expect(result.options->analysis.deadbandMode == xreal::sensors::GyroscopeDeadbandMode::soft
               && result.options->analysis.residualBiasMode
                   == xreal::sensors::GyroscopeResidualBiasMode::linear,
           "analysis refinement modes are parsed");

    constexpr std::array batch{
        std::string_view("--input"), std::string_view("a.csv"),
        std::string_view("--input"), std::string_view("b.csv"),
        std::string_view("--axis"), std::string_view("z"),
        std::string_view("--batch-outlier-percent"), std::string_view("15"),
        std::string_view("--batch-min-recordings"), std::string_view("2"),
        std::string_view("--output"), std::string_view("batch.json")};
    const auto batchResult = parse(batch);
    expect(batchResult.options.has_value() && batchResult.options->inputPaths.size() == 2U,
           "repeated input options create a hardware-independent batch");
}

void testInvalidOptions()
{
    constexpr std::array missingInput{
        std::string_view("--axis"), std::string_view("auto"),
        std::string_view("--output"), std::string_view("out.json")};
    expect(!parse(missingInput).options.has_value(), "input is required");

    constexpr std::array invalidAxis{
        std::string_view("--input"), std::string_view("a.csv"),
        std::string_view("--axis"), std::string_view("yaw"),
        std::string_view("--output"), std::string_view("out.json")};
    expect(!parse(invalidAxis).options.has_value(), "invalid axis is rejected");

    constexpr std::array zeroAngle{
        std::string_view("--input"), std::string_view("a.csv"),
        std::string_view("--axis"), std::string_view("auto"),
        std::string_view("--expected-degrees"), std::string_view("0"),
        std::string_view("--output"), std::string_view("out.json")};
    expect(!parse(zeroAngle).options.has_value(), "zero expected angle is rejected");

    constexpr std::array partialBias{
        std::string_view("--input"), std::string_view("a.csv"),
        std::string_view("--axis"), std::string_view("auto"),
        std::string_view("--bias-x"), std::string_view("1"),
        std::string_view("--output"), std::string_view("out.json")};
    expect(!parse(partialBias).options.has_value(), "partial explicit bias is rejected");
}

} // namespace

int main()
{
    testValidSingleAndBatch();
    testInvalidOptions();
    if (failures != 0)
    {
        std::cerr << failures << " analyzer option test(s) failed.\n";
        return 1;
    }
    std::cout << "All gyroscope recording analyzer option tests passed.\n";
    return 0;
}
