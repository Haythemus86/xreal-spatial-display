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
        || option == "--monitor-index" || option == "--render-monitor-index"
        || option == "--capture-monitor-index"
        || option == "--render-monitor-device-name"
        || option == "--capture-monitor-device-name"
        || option == "--panel-content" || option == "--desktop-fit"
        || option == "--desktop-filter" || option == "--desktop-background"
        || option == "--desktop-stale-threshold-ms"
        || option == "--desktop-capture-timeout-ms"
        || option == "--desktop-capture-retry-ms"
        || option == "--desktop-capture-cross-adapter"
        || option == "--desktop-capture-json-output"
        || option == "--desktop-capture-dump-first-frame"
        || option == "--desktop-debug-readback-upload"
        || option == "--desktop-debug-dump-render-target"
        || option == "--desktop-capture-smoke-test-frames"
        || option == "--render-duration"
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
    unsigned int desktopShaderDebugModeCount{};
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
        if (argument == "--orientation-demo-static")
        {
            options.orientationDemoMode = true;
            options.orientationDemoStatic = true;
            continue;
        }
        if (argument == "--smoke-test") { options.smokeTest = true; continue; }
        if (argument == "--desktop-capture-smoke-test") { options.desktopCaptureSmokeTest = true; continue; }
        if (argument == "--desktop-debug-checkerboard") { options.desktopDebugCheckerboard = true; continue; }
        if (argument == "--desktop-debug-shader-solid-red")
        {
            options.desktopShaderDebugMode = DesktopShaderDebugMode::solidRed;
            ++desktopShaderDebugModeCount;
            continue;
        }
        if (argument == "--desktop-debug-shader-uv")
        {
            options.desktopShaderDebugMode = DesktopShaderDebugMode::ultraviolet;
            ++desktopShaderDebugModeCount;
            continue;
        }
        if (argument == "--desktop-debug-shader-sample")
        {
            options.desktopShaderDebugMode = DesktopShaderDebugMode::sample;
            ++desktopShaderDebugModeCount;
            continue;
        }
        if (argument == "--desktop-debug-shader-sample-no-overlay")
        {
            options.desktopShaderDebugMode = DesktopShaderDebugMode::sampleNoOverlay;
            ++desktopShaderDebugModeCount;
            continue;
        }
        if (argument == "--desktop-debug-opaque-base")
        {
            options.desktopDebugOpaqueBase = true;
            continue;
        }
        if (argument == "--desktop-show-cursor") { options.desktopCapture.showCursor = true; continue; }
        if (argument == "--desktop-hide-cursor") { options.desktopCapture.showCursor = false; continue; }
        if (argument == "--desktop-flip-y") { options.desktopCapture.flipY = true; continue; }
        if (argument == "--allow-desktop-capture-cpu-fallback") { options.desktopCapture.allowCpuFallback = true; continue; }
        if (argument == "--desktop-capture-diagnostics") { options.desktopCapture.diagnostics = true; continue; }
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
            || argument == "--monitor-index" || argument == "--render-monitor-index"
            || argument == "--capture-monitor-index"
            || argument == "--render-diagnostics-rate"
            || argument == "--smoke-test-frames"
            || argument == "--desktop-capture-smoke-test-frames"
            || argument == "--desktop-stale-threshold-ms"
            || argument == "--desktop-capture-timeout-ms"
            || argument == "--desktop-capture-retry-ms")
        {
            const auto parsed = parseInteger<unsigned int>(value);
            if (!parsed.has_value()) { return fail(std::string(argument) + " requires an unsigned integer."); }
            if (argument == "--window-width") { options.windowWidth = *parsed; }
            else if (argument == "--window-height") { options.windowHeight = *parsed; }
            else if (argument == "--monitor-index" || argument == "--render-monitor-index") { options.monitorIndex = *parsed; }
            else if (argument == "--capture-monitor-index") { options.desktopCapture.captureMonitor.index = *parsed; }
            else if (argument == "--render-diagnostics-rate") { options.renderDiagnosticsRateHz = *parsed; }
            else if (argument == "--smoke-test-frames") { options.smokeTestFrames = *parsed; }
            else if (argument == "--desktop-capture-smoke-test-frames") { options.desktopCaptureSmokeTestFrames = *parsed; }
            else if (argument == "--desktop-stale-threshold-ms") { options.desktopCapture.staleThresholdMilliseconds = *parsed; }
            else if (argument == "--desktop-capture-timeout-ms") { options.desktopCapture.timeoutMilliseconds = *parsed; }
            else { options.desktopCapture.retryMilliseconds = *parsed; }
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
        if (argument == "--panel-content")
        {
            if (value == "synthetic") { options.desktopCapture.panelContent = capture::PanelContent::synthetic; }
            else if (value == "desktop") { options.desktopCapture.panelContent = capture::PanelContent::desktop; }
            else { return fail("--panel-content requires synthetic or desktop."); }
            continue;
        }
        if (argument == "--desktop-debug-readback-upload")
        {
            options.desktopDebugReadbackUploadPath = std::string(value);
            continue;
        }
        if (argument == "--desktop-debug-dump-render-target")
        {
            options.desktopDebugRenderTargetPath = std::string(value);
            continue;
        }
        if (argument == "--desktop-fit")
        {
            if (value == "contain") { options.desktopCapture.fit = capture::DesktopFit::contain; }
            else if (value == "cover") { options.desktopCapture.fit = capture::DesktopFit::cover; }
            else if (value == "stretch") { options.desktopCapture.fit = capture::DesktopFit::stretch; }
            else { return fail("--desktop-fit requires contain, cover or stretch."); }
            continue;
        }
        if (argument == "--desktop-filter")
        {
            if (value == "point") { options.desktopCapture.filter = capture::DesktopFilter::point; }
            else if (value == "linear") { options.desktopCapture.filter = capture::DesktopFilter::linear; }
            else { return fail("--desktop-filter requires point or linear."); }
            continue;
        }
        if (argument == "--desktop-background")
        {
            if (value == "black") { options.desktopCapture.background = capture::DesktopBackground::black; }
            else if (value == "grid") { options.desktopCapture.background = capture::DesktopBackground::grid; }
            else { return fail("--desktop-background requires black or grid."); }
            continue;
        }
        if (argument == "--desktop-capture-cross-adapter")
        {
            if (value == "auto") { options.desktopCapture.crossAdapterPolicy = capture::CrossAdapterPolicy::automatic; }
            else if (value == "shared-handle") { options.desktopCapture.crossAdapterPolicy = capture::CrossAdapterPolicy::sharedHandle; }
            else if (value == "cpu-fallback") { options.desktopCapture.crossAdapterPolicy = capture::CrossAdapterPolicy::cpuFallback; }
            else if (value == "reject") { options.desktopCapture.crossAdapterPolicy = capture::CrossAdapterPolicy::reject; }
            else { return fail("--desktop-capture-cross-adapter requires auto, shared-handle, cpu-fallback or reject."); }
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
        if (argument == "--accelerometer-profile" || argument == "--render-json-output"
            || argument == "--render-monitor-device-name"
            || argument == "--capture-monitor-device-name"
            || argument == "--desktop-capture-json-output"
            || argument == "--desktop-capture-dump-first-frame")
        {
            if (value.empty()) { return fail(std::string(argument) + " requires a non-empty path."); }
            if (argument == "--accelerometer-profile") { options.accelerometerProfilePath = std::string(value); }
            else if (argument == "--render-json-output") { options.renderJsonOutputPath = std::string(value); }
            else if (argument == "--render-monitor-device-name") { options.renderMonitorDeviceName = std::string(value); }
            else if (argument == "--capture-monitor-device-name") { options.desktopCapture.captureMonitor.deviceName = std::string(value); }
            else if (argument == "--desktop-capture-json-output") { options.desktopCapture.jsonOutputPath = std::string(value); }
            else { options.desktopCapture.firstFrameBmpPath = std::string(value); }
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
    if (desktopShaderDebugModeCount > 1U)
    {
        return fail("Desktop pixel-shader diagnostic modes are mutually exclusive.");
    }
    if ((options.desktopDebugReadbackUploadPath.has_value()
            && options.desktopDebugReadbackUploadPath->empty())
        || (options.desktopDebugRenderTargetPath.has_value()
            && options.desktopDebugRenderTargetPath->empty()))
    {
        return fail("Desktop debug BMP output paths must not be empty.");
    }
    if (options.desktopCapture.timeoutMilliseconds == 0U
        || options.desktopCapture.retryMilliseconds == 0U
        || options.desktopCapture.staleThresholdMilliseconds == 0U)
    {
        return fail("Desktop capture timeout, retry and stale threshold must be positive.");
    }
    if (options.desktopCaptureSmokeTestFrames == 0U)
    {
        return fail("--desktop-capture-smoke-test-frames must be positive.");
    }
    if (options.desktopCapture.crossAdapterPolicy == capture::CrossAdapterPolicy::cpuFallback
        && !options.desktopCapture.allowCpuFallback)
    {
        return fail("CPU desktop-capture fallback requires --allow-desktop-capture-cpu-fallback.");
    }
    if (options.desktopCapture.panelContent == capture::PanelContent::desktop
        && !options.desktopDebugCheckerboard
        && !options.desktopCapture.captureMonitor.index.has_value()
        && !options.desktopCapture.captureMonitor.deviceName.has_value())
    {
        return fail("Desktop panel mode requires --capture-monitor-index or --capture-monitor-device-name.");
    }
    if (options.desktopCaptureSmokeTest)
    {
        options.desktopCapture.panelContent = capture::PanelContent::desktop;
        options.orientationDemoMode = true;
        options.smokeTest = true;
        options.smokeTestFrames = options.desktopCaptureSmokeTestFrames;
    }
    if (options.desktopDebugCheckerboard)
    {
        options.desktopCapture.panelContent = capture::PanelContent::desktop;
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
           "[--fullscreen|--borderless] [--monitor-index <index>|--render-monitor-index <index>] "
           "[--render-monitor-device-name <name>] [--vsync|--no-vsync] "
           "[--allow-warp-fallback] [--render-duration <seconds>] [--target-fps <fps>] "
           "[--show-render-diagnostics] [--render-diagnostics-rate <hz>] "
           "[--panel-distance <value>] [--panel-width <value>] [--panel-height <value>] "
           "[--field-of-view-degrees <value>] [--near-plane <value>] [--far-plane <value>] "
           "[--background-grid] [--world-axes] "
           "[--orientation-demo-static] "
           "[--render-orientation-source <measured|predicted>] "
           "[--render-orientation-frame <absolute|relative>] [--recenter-on-start] "
           "[--predict-orientation] [--prediction-mode <constant-velocity|constant-acceleration>] "
           "[--prediction-horizon-ms <value>] [--render-json-output <file.json>] "
           "[--panel-content <synthetic|desktop>] [--capture-monitor-index <index>] "
           "[--capture-monitor-device-name <name>] [--desktop-fit <contain|cover|stretch>] "
           "[--desktop-filter <point|linear>] [--desktop-show-cursor|--desktop-hide-cursor] "
           "[--desktop-capture-cross-adapter <auto|shared-handle|cpu-fallback|reject>] "
           "[--desktop-capture-diagnostics] [--desktop-capture-json-output <file.json>] "
           "[--desktop-debug-checkerboard] "
           "[--desktop-debug-shader-solid-red|--desktop-debug-shader-uv|"
           "--desktop-debug-shader-sample|--desktop-debug-shader-sample-no-overlay] "
           "[--desktop-debug-opaque-base] "
           "[--desktop-debug-readback-upload <file.bmp>] "
           "[--desktop-debug-dump-render-target <file.bmp>] "
           "[--desktop-capture-dump-first-frame <file.bmp>] "
           "[--smoke-test] [--smoke-test-frames <count>]\n";
}

} // namespace xreal::rendering
