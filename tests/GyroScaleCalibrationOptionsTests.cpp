#include "diagnostics/GyroScaleCalibrationOptions.hpp"

#include <array>
#include <chrono>
#include <iostream>
#include <string_view>

namespace
{

int failureCount{};

void expect(bool condition, std::string_view description)
{
    if (!condition)
    {
        ++failureCount;
        std::cerr << "FAILED: " << description << '\n';
    }
}

template <std::size_t Size>
[[nodiscard]] auto parse(const std::array<std::string_view, Size>& arguments)
{
    return xreal::diagnostics::parseGyroScaleCalibrationOptions(arguments);
}

void testValidOptions()
{
    constexpr std::array arguments{
        std::string_view("--axis"), std::string_view("z"),
        std::string_view("--expected-degrees"), std::string_view("360"),
        std::string_view("--direction"), std::string_view("negative"),
        std::string_view("--trials"), std::string_view("8"),
        std::string_view("--gyro-calibrate-seconds"), std::string_view("3"),
        std::string_view("--gyro-warmup-seconds"), std::string_view("1.5"),
        std::string_view("--start-threshold"), std::string_view("6000"),
        std::string_view("--stop-threshold"), std::string_view("1200"),
        std::string_view("--stillness-seconds"), std::string_view("2"),
        std::string_view("--min-rotation-seconds"), std::string_view("0.75"),
        std::string_view("--max-rotation-seconds"), std::string_view("15"),
        std::string_view("--countdown-seconds"), std::string_view("0"),
        std::string_view("--max-cross-axis-ratio"), std::string_view("0.25"),
        std::string_view("--max-trial-variation-percent"), std::string_view("8"),
        std::string_view("--min-accepted-trials"), std::string_view("5"),
        std::string_view("--output"), std::string_view("scale.json"),
        std::string_view("--csv-prefix"), std::string_view("scale-z"),
    };
    const auto result = parse(arguments);
    expect(result.options.has_value(), "a complete valid command is accepted");
    if (!result.options.has_value())
    {
        return;
    }
    expect(result.options->scaleConfiguration.axis == xreal::sensors::GyroscopeAxis::z,
           "axis is parsed");
    expect(result.options->scaleConfiguration.expectedAngleDegrees == 360.0,
           "expected angle is parsed");
    expect(result.options->scaleConfiguration.direction
               == xreal::sensors::RotationDirection::negative,
           "direction is parsed");
    expect(result.options->trialCount == 8U, "trial count is parsed");
    expect(result.options->biasConfiguration.calibrationDuration == std::chrono::seconds(3),
           "bias duration is parsed");
    expect(result.options->countdownSeconds == 0U, "zero countdown is accepted");
    expect(result.options->scaleConfiguration.minimumAcceptedTrials == 5U,
           "minimum accepted trials is parsed");
    expect(result.options->outputPath == "scale.json", "output path is parsed");
    expect(result.options->csvPrefix == "scale-z", "CSV prefix is parsed");
}

void testRequiredAndScalarValidation()
{
    constexpr std::array missingAxis{
        std::string_view("--expected-degrees"), std::string_view("360"),
        std::string_view("--output"), std::string_view("scale.json")};
    expect(!parse(missingAxis).options.has_value(), "axis is required");

    constexpr std::array invalidAxis{
        std::string_view("--axis"), std::string_view("yaw"),
        std::string_view("--expected-degrees"), std::string_view("360"),
        std::string_view("--output"), std::string_view("scale.json")};
    expect(!parse(invalidAxis).options.has_value(), "invalid axis is rejected");

    constexpr std::array zeroAngle{
        std::string_view("--axis"), std::string_view("z"),
        std::string_view("--expected-degrees"), std::string_view("0"),
        std::string_view("--output"), std::string_view("scale.json")};
    expect(!parse(zeroAngle).options.has_value(), "zero expected angle is rejected");

    constexpr std::array negativeAngle{
        std::string_view("--axis"), std::string_view("z"),
        std::string_view("--expected-degrees"), std::string_view("-90"),
        std::string_view("--output"), std::string_view("scale.json")};
    expect(!parse(negativeAngle).options.has_value(), "negative expected angle is rejected");

    constexpr std::array zeroTrials{
        std::string_view("--axis"), std::string_view("z"),
        std::string_view("--expected-degrees"), std::string_view("360"),
        std::string_view("--trials"), std::string_view("0"),
        std::string_view("--output"), std::string_view("scale.json")};
    expect(!parse(zeroTrials).options.has_value(), "zero trials is rejected");
}

void testRelationalValidation()
{
    constexpr std::array thresholds{
        std::string_view("--axis"), std::string_view("x"),
        std::string_view("--expected-degrees"), std::string_view("90"),
        std::string_view("--start-threshold"), std::string_view("1000"),
        std::string_view("--stop-threshold"), std::string_view("1000"),
        std::string_view("--output"), std::string_view("scale.json")};
    expect(!parse(thresholds).options.has_value(), "inconsistent thresholds are rejected");

    constexpr std::array durations{
        std::string_view("--axis"), std::string_view("x"),
        std::string_view("--expected-degrees"), std::string_view("90"),
        std::string_view("--min-rotation-seconds"), std::string_view("2"),
        std::string_view("--max-rotation-seconds"), std::string_view("2"),
        std::string_view("--output"), std::string_view("scale.json")};
    expect(!parse(durations).options.has_value(), "inconsistent durations are rejected");

    constexpr std::array crossAxis{
        std::string_view("--axis"), std::string_view("x"),
        std::string_view("--expected-degrees"), std::string_view("90"),
        std::string_view("--max-cross-axis-ratio"), std::string_view("1.1"),
        std::string_view("--output"), std::string_view("scale.json")};
    expect(!parse(crossAxis).options.has_value(), "cross-axis ratio above one is rejected");

    constexpr std::array minimumTrials{
        std::string_view("--axis"), std::string_view("x"),
        std::string_view("--expected-degrees"), std::string_view("90"),
        std::string_view("--trials"), std::string_view("3"),
        std::string_view("--min-accepted-trials"), std::string_view("4"),
        std::string_view("--output"), std::string_view("scale.json")};
    expect(!parse(minimumTrials).options.has_value(),
           "minimum accepted count above requested trials is rejected");
}

void testRecordOnlyOptions()
{
    constexpr std::array arguments{
        std::string_view("--record-only"),
        std::string_view("--record-seconds"), std::string_view("15"),
        std::string_view("--gyro-calibrate-seconds"), std::string_view("2"),
        std::string_view("--gyro-warmup-seconds"), std::string_view("1"),
        std::string_view("--countdown-seconds"), std::string_view("3"),
        std::string_view("--axis"), std::string_view("auto"),
        std::string_view("--expected-degrees"), std::string_view("360"),
        std::string_view("--direction"), std::string_view("auto"),
        std::string_view("--csv-output"), std::string_view("recording.csv"),
        std::string_view("--analysis-output"), std::string_view("analysis.json"),
        std::string_view("--print-rate"), std::string_view("5"),
        std::string_view("--analysis-start-threshold"), std::string_view("6000"),
        std::string_view("--analysis-stop-threshold"), std::string_view("1200"),
        std::string_view("--analysis-stillness-seconds"), std::string_view("1.5"),
        std::string_view("--max-device-delta-ms"), std::string_view("25"),
        std::string_view("--include-accelerometer"),
        std::string_view("--include-host-timestamps"),
    };
    const auto result = parse(arguments);
    expect(result.options.has_value() && result.options->recordOnly,
           "valid record-only options are accepted");
    if (!result.options.has_value())
    {
        return;
    }
    expect(result.options->recordDuration == std::chrono::seconds(15),
           "recording duration is parsed");
    expect(result.options->recordingAnalysisConfiguration.axisSelection
               == xreal::sensors::GyroscopeAxisSelection::automatic,
           "axis auto is accepted in record-only mode");
    expect(result.options->recordingAnalysisConfiguration.expectedAngleDegrees == 360.0,
           "optional expected angle enables offline scale estimation");
    expect(result.options->csvOutputPath == "recording.csv"
               && result.options->analysisOutputPath == "analysis.json",
           "record-only output paths are parsed");

    constexpr std::array withoutAngle{
        std::string_view("--record-only"),
        std::string_view("--record-seconds"), std::string_view("5"),
        std::string_view("--axis"), std::string_view("z"),
        std::string_view("--csv-output"), std::string_view("recording.csv"),
        std::string_view("--analysis-output"), std::string_view("analysis.json")};
    const auto optionalAngle = parse(withoutAngle);
    expect(optionalAngle.options.has_value()
               && !optionalAngle.options->recordingAnalysisConfiguration.expectedAngleDegrees
                       .has_value(),
           "record-only analysis does not require expected degrees");
}

void testInvalidRecordOnlyOptions()
{
    constexpr std::array zeroDuration{
        std::string_view("--record-only"),
        std::string_view("--record-seconds"), std::string_view("0"),
        std::string_view("--axis"), std::string_view("auto"),
        std::string_view("--csv-output"), std::string_view("recording.csv"),
        std::string_view("--analysis-output"), std::string_view("analysis.json")};
    expect(!parse(zeroDuration).options.has_value(), "zero recording duration is rejected");

    constexpr std::array negativeDuration{
        std::string_view("--record-only"),
        std::string_view("--record-seconds"), std::string_view("-1"),
        std::string_view("--axis"), std::string_view("auto"),
        std::string_view("--csv-output"), std::string_view("recording.csv"),
        std::string_view("--analysis-output"), std::string_view("analysis.json")};
    expect(!parse(negativeDuration).options.has_value(),
           "negative recording duration is rejected");

    constexpr std::array invalidAxis{
        std::string_view("--record-only"),
        std::string_view("--record-seconds"), std::string_view("5"),
        std::string_view("--axis"), std::string_view("yaw"),
        std::string_view("--csv-output"), std::string_view("recording.csv"),
        std::string_view("--analysis-output"), std::string_view("analysis.json")};
    expect(!parse(invalidAxis).options.has_value(), "invalid record-only axis is rejected");

    constexpr std::array invalidAngle{
        std::string_view("--record-only"),
        std::string_view("--record-seconds"), std::string_view("5"),
        std::string_view("--axis"), std::string_view("auto"),
        std::string_view("--expected-degrees"), std::string_view("0"),
        std::string_view("--csv-output"), std::string_view("recording.csv"),
        std::string_view("--analysis-output"), std::string_view("analysis.json")};
    expect(!parse(invalidAngle).options.has_value(), "invalid optional expected angle is rejected");
}

} // namespace

int main()
{
    testValidOptions();
    testRequiredAndScalarValidation();
    testRelationalValidation();
    testRecordOnlyOptions();
    testInvalidRecordOnlyOptions();
    if (failureCount != 0)
    {
        std::cerr << failureCount << " gyroscope scale option test(s) failed.\n";
        return 1;
    }
    std::cout << "All gyroscope scale option tests passed.\n";
    return 0;
}
