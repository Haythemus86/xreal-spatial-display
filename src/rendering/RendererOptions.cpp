#include "rendering/RendererOptions.hpp"

#include <charconv>
#include <cmath>
#include <limits>
#include <string_view>

namespace xreal::rendering
{
namespace
{

[[nodiscard]] std::optional<double> parseDouble(std::string_view value)
{
    double parsed{};
    const auto result = std::from_chars(value.data(), value.data() + value.size(), parsed);
    if (result.ec != std::errc{} || result.ptr != value.data() + value.size()
        || !std::isfinite(parsed))
    {
        return std::nullopt;
    }
    return parsed;
}

template<typename Integer>
[[nodiscard]] std::optional<Integer> parseInteger(std::string_view value)
{
    Integer parsed{};
    const auto result = std::from_chars(value.data(), value.data() + value.size(), parsed);
    if (result.ec != std::errc{} || result.ptr != value.data() + value.size())
    {
        return std::nullopt;
    }
    return parsed;
}

[[nodiscard]] bool requiresValue(std::string_view option)
{
    return option == "--window-width" || option == "--window-height"
        || option == "--window-x" || option == "--window-y"
        || option == "--monitor-index" || option == "--render-duration"
        || option == "--panel-distance" || option == "--panel-width"
        || option == "--panel-height" || option == "--field-of-view-degrees"
        || option == "--near-plane" || option == "--far-plane"
        || option == "--render-orientation-source"
        || option == "--render-orientation-frame"
        || option == "--recenter-key" || option == "--reset-recenter-key"
        || option == "--gyro-calibrate-seconds" || option == "--gyro-warmup-seconds"
        || option == "--gyro-scale-raw-per-dps" || option == "--accelerometer-profile"
        || option == "--fusion-mode" || option == "--fusion-startup"
        || option == "--accelerometer-correction-time-constant"
        || option == "--accelerometer-max-correction-dps"
        || option == "--prediction-mode" || option == "--prediction-horizon-ms"
        || option == "--prediction-max-horizon-ms"
        || option == "--prediction-angular-velocity-smoothing-seconds"
        || option == "--prediction-max-angular-speed-dps"
        || option == "--prediction-max-angle-degrees"
        || option == "--prediction-limit-behavior"
        || option == "--target-fps" || option == "--render-diagnostics-rate"
        || option == "--render-json-output" || option == "--smoke-test-frames";
}

[[nodiscard]] RendererOptionResult fail(std::string error)
{
    return {std::nullopt, std::move(error)};
}

} // namespace

RendererOptionResult parseRendererOptions(int argc, char* argv[])
{
    RendererOptions options;
    bool predictionOptionExplicit{};
    for (int index = 1; index < argc; ++index)
    {
        const std::string_view argument(argv[index]);
        if (argument == "--help" || argument == "-h")
        {
            options.help = true;
            continue;
        }
        if (argument == "--fullscreen") { options.fullscreen = true; continue; }
        if (argument == "--borderless") { options.borderless = true; continue; }
        if (argument == "--vsync") { options.vsync = true; continue; }
        if (argument == "--no-vsync") { options.vsync = false; continue; }
        if (argument == "--allow-warp-fallback") { options.allowWarpFallback = true; continue; }
        if (argument == "--show-render-diagnostics") { options.showRenderDiagnostics = true; continue; }
        if (argument == "--background-grid") { options.backgroundGrid = true; continue; }
        if (argument == "--world-axes") { options.worldAxes = true; continue; }
        if (argument == "--recenter-on-start") { options.recenterOnStart = true; continue; }
        if (argument == "--orientation-demo-mode") { options.orientationDemoMode = true; continue; }
        if (argument == "--smoke-test") { options.smokeTest = true; continue; }
        if (argument == "--apply-gyro-bias") { options.applyGyroscopeBias = true; continue; }
        if (argument == "--predict-orientation")
        {
            options.predictOrientation = true;
            predictionOptionExplicit = true;
            continue;
        }

        if (!requiresValue(argument))
        {
            return fail("Unknown renderer option: " + std::string(argument));
        }
        if (index + 1 >= argc)
        {
            return fail(std::string(argument) + " requires a value.");
        }
        const std::string_view value(argv[++index]);
        if (argument.starts_with("--prediction-"))
        {
            predictionOptionExplicit = true;
        }

        if (argument == "--window-width" || argument == "--window-height"
            || argument == "--monitor-index" || argument == "--render-diagnostics-rate"
            || argument == "--smoke-test-frames")
        {
            const auto parsed = parseInteger<unsigned int>(value);
            if (!parsed.has_value()) { return fail(std::string(argument) + " requires an unsigned integer."); }
            if (argument == "--window-width") { options.windowWidth = *parsed; }
            else if (argument == "--window-height") { options.windowHeight = *parsed; }
            else if (argument == "--monitor-index") { options.monitorIndex = *parsed; }
            else if (argument == "--render-diagnostics-rate") { options.renderDiagnosticsRateHz = *parsed; }
            else { options.smokeTestFrames = *parsed; }
            continue;
        }
        if (argument == "--window-x" || argument == "--window-y")
        {
            const auto parsed = parseInteger<int>(value);
            if (!parsed.has_value()) { return fail(std::string(argument) + " requires an integer."); }
            if (argument == "--window-x") { options.windowX = *parsed; }
            else { options.windowY = *parsed; }
            continue;
        }
        if (argument == "--render-orientation-source")
        {
            if (value == "measured") { options.orientationSource = RenderOrientationSource::measured; }
            else if (value == "predicted") { options.orientationSource = RenderOrientationSource::predicted; }
            else { return fail("--render-orientation-source requires measured or predicted."); }
            continue;
        }
        if (argument == "--render-orientation-frame")
        {
            if (value == "absolute") { options.orientationFrame = RenderOrientationFrame::absolute; }
            else if (value == "relative") { options.orientationFrame = RenderOrientationFrame::relative; }
            else { return fail("--render-orientation-frame requires absolute or relative."); }
            continue;
        }
        if (argument == "--prediction-mode")
        {
            if (value == "constant-velocity") { options.predictionMode = sensors::PredictionMode::constantVelocity; }
            else if (value == "constant-acceleration") { options.predictionMode = sensors::PredictionMode::constantAcceleration; }
            else { return fail("--prediction-mode requires constant-velocity or constant-acceleration."); }
            continue;
        }
        if (argument == "--prediction-limit-behavior")
        {
            if (value == "reject") { options.predictionLimitBehavior = sensors::PredictionLimitBehavior::reject; }
            else if (value == "clamp") { options.predictionLimitBehavior = sensors::PredictionLimitBehavior::clamp; }
            else { return fail("--prediction-limit-behavior requires reject or clamp."); }
            continue;
        }
        if (argument == "--fusion-mode")
        {
            if (value != "complementary") { return fail("Only --fusion-mode complementary is supported by the renderer."); }
            continue;
        }
        if (argument == "--fusion-startup")
        {
            if (value == "gravity") { options.fusionStartupGravity = true; }
            else if (value == "identity") { options.fusionStartupGravity = false; }
            else { return fail("--fusion-startup requires gravity or identity."); }
            continue;
        }
        if (argument == "--recenter-key" || argument == "--reset-recenter-key")
        {
            if (value.size() != 1U) { return fail(std::string(argument) + " requires one character."); }
            const char key = static_cast<char>(std::toupper(static_cast<unsigned char>(value.front())));
            if (argument == "--recenter-key") { options.recenterKey = key; }
            else { options.resetRecenterKey = key; }
            continue;
        }
        if (argument == "--accelerometer-profile" || argument == "--render-json-output")
        {
            if (value.empty()) { return fail(std::string(argument) + " requires a non-empty path."); }
            if (argument == "--accelerometer-profile") { options.accelerometerProfilePath = std::string(value); }
            else { options.renderJsonOutputPath = std::string(value); }
            continue;
        }

        const auto parsed = parseDouble(value);
        if (!parsed.has_value()) { return fail(std::string(argument) + " requires a finite number."); }
        if (argument == "--render-duration") { options.renderDurationSeconds = *parsed; }
        else if (argument == "--panel-distance") { options.panelDistance = *parsed; }
        else if (argument == "--panel-width") { options.panelWidth = *parsed; }
        else if (argument == "--panel-height") { options.panelHeight = *parsed; }
        else if (argument == "--field-of-view-degrees") { options.fieldOfViewDegrees = *parsed; }
        else if (argument == "--near-plane") { options.nearPlane = *parsed; }
        else if (argument == "--far-plane") { options.farPlane = *parsed; }
        else if (argument == "--gyro-calibrate-seconds") { options.gyroscopeCalibrationSeconds = *parsed; }
        else if (argument == "--gyro-warmup-seconds") { options.gyroscopeWarmupSeconds = *parsed; }
        else if (argument == "--gyro-scale-raw-per-dps") { options.gyroscopeScaleRawPerDegreePerSecond = *parsed; }
        else if (argument == "--accelerometer-correction-time-constant") { options.accelerometerCorrectionTimeConstantSeconds = *parsed; }
        else if (argument == "--accelerometer-max-correction-dps") { options.accelerometerMaximumCorrectionDegreesPerSecond = *parsed; }
        else if (argument == "--prediction-horizon-ms") { options.predictionHorizonMilliseconds = *parsed; }
        else if (argument == "--prediction-max-horizon-ms") { options.predictionMaximumHorizonMilliseconds = *parsed; }
        else if (argument == "--prediction-angular-velocity-smoothing-seconds") { options.predictionAngularVelocitySmoothingSeconds = *parsed; }
        else if (argument == "--prediction-max-angular-speed-dps") { options.predictionMaximumAngularSpeedDegreesPerSecond = *parsed; }
        else if (argument == "--prediction-max-angle-degrees") { options.predictionMaximumAngleDegrees = *parsed; }
        else if (argument == "--target-fps") { options.targetFramesPerSecond = *parsed; }
    }

    if (options.windowWidth == 0U || options.windowHeight == 0U)
    {
        return fail("Window width and height must be positive.");
    }
    if (options.fullscreen && options.borderless)
    {
        return fail("--fullscreen and --borderless are mutually exclusive.");
    }
    if (options.renderDurationSeconds.has_value() && *options.renderDurationSeconds <= 0.0)
    {
        return fail("--render-duration must be positive.");
    }
    if (options.panelDistance <= 0.0 || options.panelWidth <= 0.0 || options.panelHeight <= 0.0)
    {
        return fail("Panel distance, width and height must be positive.");
    }
    if (options.fieldOfViewDegrees <= 0.0 || options.fieldOfViewDegrees >= 180.0
        || options.nearPlane <= 0.0 || options.farPlane <= options.nearPlane)
    {
        return fail("Projection requires 0 < FOV < 180 and 0 < near < far.");
    }
    if (options.renderDiagnosticsRateHz == 0U || options.renderDiagnosticsRateHz > 60U)
    {
        return fail("--render-diagnostics-rate must be between 1 and 60 Hz.");
    }
    if (options.targetFramesPerSecond.has_value() && *options.targetFramesPerSecond <= 0.0)
    {
        return fail("--target-fps must be positive.");
    }
    if (options.smokeTestFrames == 0U)
    {
        return fail("--smoke-test-frames must be positive.");
    }
    if (options.predictionHorizonMilliseconds < 0.0
        || options.predictionMaximumHorizonMilliseconds <= 0.0
        || options.predictionMaximumAngularSpeedDegreesPerSecond <= 0.0
        || options.predictionMaximumAngleDegrees <= 0.0)
    {
        return fail("Prediction horizons and limits are invalid.");
    }
    if (options.predictionHorizonMilliseconds > options.predictionMaximumHorizonMilliseconds
        && options.predictionLimitBehavior == sensors::PredictionLimitBehavior::reject)
    {
        return fail("Prediction horizon exceeds its maximum; use clamp explicitly to bound it.");
    }
    if (predictionOptionExplicit
        && options.orientationSource != RenderOrientationSource::predicted)
    {
        return fail("Prediction options require --render-orientation-source predicted.");
    }
    if (options.smokeTest)
    {
        options.orientationDemoMode = true;
    }
    if (!options.orientationDemoMode)
    {
        if (!options.applyGyroscopeBias || !options.gyroscopeScaleRawPerDegreePerSecond.has_value()
            || !options.accelerometerProfilePath.has_value())
        {
            return fail("Hardware mode requires --apply-gyro-bias, --gyro-scale-raw-per-dps and --accelerometer-profile.");
        }
        if (options.orientationSource == RenderOrientationSource::predicted
            && !options.predictOrientation)
        {
            return fail("Predicted hardware rendering requires --predict-orientation.");
        }
    }
    if (options.gyroscopeCalibrationSeconds <= 0.0 || options.gyroscopeWarmupSeconds < 0.0
        || options.accelerometerCorrectionTimeConstantSeconds <= 0.0
        || options.accelerometerMaximumCorrectionDegreesPerSecond <= 0.0)
    {
        return fail("Calibration and fusion timing values are invalid.");
    }
    return {options, {}};
}

std::string rendererUsage()
{
    return "xreal-spatial-renderer [--orientation-demo-mode] [--window-width <pixels>] "
           "[--window-height <pixels>] [--window-x <pixels>] [--window-y <pixels>] "
           "[--fullscreen|--borderless] [--monitor-index <index>] [--vsync|--no-vsync] "
           "[--allow-warp-fallback] [--render-duration <seconds>] [--target-fps <fps>] "
           "[--show-render-diagnostics] [--render-diagnostics-rate <hz>] "
           "[--panel-distance <value>] [--panel-width <value>] [--panel-height <value>] "
           "[--field-of-view-degrees <value>] [--near-plane <value>] [--far-plane <value>] "
           "[--background-grid] [--world-axes] "
           "[--render-orientation-source <measured|predicted>] "
           "[--render-orientation-frame <absolute|relative>] [--recenter-on-start] "
           "[--predict-orientation] [--prediction-mode <constant-velocity|constant-acceleration>] "
           "[--prediction-horizon-ms <value>] [--render-json-output <file.json>] "
           "[--smoke-test] [--smoke-test-frames <count>]\n";
}

} // namespace xreal::rendering
