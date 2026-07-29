#include "rendering/RendererApplication.hpp"

#include "graphics/D3D11Renderer.hpp"
#include "platform/windows/RenderWindow.hpp"
#include "rendering/Camera.hpp"
#include "rendering/DemoOrientationSource.hpp"
#include "rendering/OrientationRenderBridge.hpp"
#include "rendering/RenderDiagnostics.hpp"
#include "rendering/RendererSummary.hpp"
#include "rendering/SensorOrientationService.hpp"

#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <numbers>
#include <optional>
#include <sstream>
#include <thread>

namespace xreal::rendering
{
namespace
{

[[nodiscard]] std::optional<std::string> readTextFile(
    const std::string& path,
    std::string& error)
{
    std::ifstream input(path, std::ios::binary);
    if (!input)
    {
        error = "Failed to open file: " + path;
        return std::nullopt;
    }
    std::ostringstream contents;
    contents << input.rdbuf();
    if (!input.good() && !input.eof())
    {
        error = "Failed while reading file: " + path;
        return std::nullopt;
    }
    return contents.str();
}

[[nodiscard]] std::optional<SensorOrientationConfig> makeSensorConfig(
    const RendererOptions& options,
    std::string& error)
{
    if (!options.gyroscopeScaleRawPerDegreePerSecond.has_value()
        || !options.accelerometerProfilePath.has_value())
    {
        error = "Hardware rendering configuration is incomplete.";
        return std::nullopt;
    }
    SensorOrientationConfig result;
    result.gyroscopeScale = sensors::makeExperimentalGyroscopeScaleProfile(
        *options.gyroscopeScaleRawPerDegreePerSecond);
    result.gyroscopeScale.axisMapping =
        sensors::makeExperimentalXrealAir2UltraGyroscopeAxisMapping();
    const auto gyroValidation = sensors::validateGyroscopeScaleProfile(result.gyroscopeScale);
    if (!gyroValidation.valid)
    {
        error = "Invalid gyroscope scale: " + gyroValidation.explanation;
        return std::nullopt;
    }
    const auto text = readTextFile(*options.accelerometerProfilePath, error);
    if (!text.has_value())
    {
        return std::nullopt;
    }
    const auto loaded = sensors::loadAccelerometerCalibrationProfileJson(
        *text, *options.accelerometerProfilePath);
    if (!loaded.profile.has_value())
    {
        error = "Invalid accelerometer profile: " + loaded.error;
        return std::nullopt;
    }
    result.accelerometerProfile = *loaded.profile;
    result.accelerometerProfile.axisMapping =
        sensors::makeExperimentalXrealAir2UltraAccelerometerAxisMapping();
    if (!sensors::validateAccelerometerCalibrationProfile(result.accelerometerProfile))
    {
        error = "The XREAL Air 2 Ultra accelerometer sensor-to-body mapping is invalid.";
        return std::nullopt;
    }
    result.biasCalibration.calibrationDuration = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(options.gyroscopeCalibrationSeconds));
    result.biasCalibration.warmupDuration = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(options.gyroscopeWarmupSeconds));
    result.fusion.gyroscopeAxisMapping = result.gyroscopeScale.axisMapping;
    result.fusion.correctionTimeConstant = std::chrono::duration<double>(
        options.accelerometerCorrectionTimeConstantSeconds);
    result.fusion.maximumCorrectionDegreesPerSecond =
        options.accelerometerMaximumCorrectionDegreesPerSecond;
    result.fusion.startupMode = options.fusionStartupGravity
        ? sensors::FusionStartupMode::gravity : sensors::FusionStartupMode::identity;
    result.prediction.mode = options.predictionMode;
    result.prediction.horizon = std::chrono::duration<double>(
        options.predictionHorizonMilliseconds * 0.001);
    result.prediction.maximumHorizon = std::chrono::duration<double>(
        options.predictionMaximumHorizonMilliseconds * 0.001);
    if (options.predictionAngularVelocitySmoothingSeconds.has_value())
    {
        result.prediction.angularVelocitySmoothingTimeConstant = std::chrono::duration<double>(
            *options.predictionAngularVelocitySmoothingSeconds);
    }
    result.prediction.maximumAngularSpeedRadiansPerSecond =
        options.predictionMaximumAngularSpeedDegreesPerSecond * std::numbers::pi / 180.0;
    result.prediction.maximumPredictionAngleRadians =
        options.predictionMaximumAngleDegrees * std::numbers::pi / 180.0;
    result.prediction.limitBehavior = options.predictionLimitBehavior;
    result.predictionEnabled = options.predictOrientation;
    result.recenterOnStart = options.recenterOnStart;
    return result;
}

void printMonitors(const std::vector<platform::windows::MonitorInformation>& monitors)
{
    std::cout << "Available monitors:\n";
    for (const auto& monitor : monitors)
    {
        std::cout << "  [" << monitor.index << "] " << monitor.deviceName
                  << " bounds=[" << monitor.left << ',' << monitor.top << ','
                  << monitor.right << ',' << monitor.bottom << "] resolution="
                  << monitor.right - monitor.left << 'x' << monitor.bottom - monitor.top
                  << " work=["
                  << monitor.workLeft << ',' << monitor.workTop << ','
                  << monitor.workRight << ',' << monitor.workBottom << "] primary="
                  << (monitor.primary ? "yes" : "no") << '\n';
        if (monitor.dxgiOutput.has_value())
        {
            const auto& output = *monitor.dxgiOutput;
            std::cout << "      DXGI adapter=" << output.adapterIndex
                      << " output=" << output.outputIndex
                      << " description=\"" << output.adapterDescription << "\""
                      << " LUID="
                      << platform::windows::dxgiAdapterLuidText(output.adapterLuid) << '\n';
        }
        else
        {
            std::cout << "      DXGI output: unmatched\n";
        }
    }
}

void printControls(const RendererOptions& options)
{
    std::cout << "Controls:\n"
              << "  Escape: exit\n"
              << "  " << options.recenterKey << ": recenter relative orientation\n"
              << "  " << options.resetRecenterKey << ": clear recenter reference\n"
              << "  P: toggle measured/predicted\n"
              << "  F: toggle absolute/relative\n"
              << "  G: toggle background grid\n"
              << "  X: toggle world axes\n"
              << "  V: toggle vsync\n"
              << "  D: toggle console diagnostics\n"
              << "  Demo: arrows=yaw/pitch, Q/E=roll, Space=reset\n";
}

void printFrameDiagnostics(
    const FrameTimingStatistics& statistics,
    RendererStartupState state,
    RenderOrientationSource source,
    RenderOrientationFrame frame)
{
    std::cout << std::fixed << std::setprecision(2)
              << "render state=" << rendererStartupStateText(state)
              << " fps=" << statistics.averageFramesPerSecond
              << " frame_ms=" << statistics.averageFrameTimeMilliseconds
              << " max_frame_ms=" << statistics.maximumFrameTimeMilliseconds
              << " snapshot_age_ms=" << statistics.averageSnapshotAgeMilliseconds
              << " effective_lead_ms=" << statistics.averageApproximateEffectiveLeadMilliseconds
              << " source=" << (source == RenderOrientationSource::predicted ? "predicted" : "measured")
              << " frame=" << (frame == RenderOrientationFrame::relative ? "relative" : "absolute")
              << " repeated=" << statistics.repeatedOrientationSnapshots
              << " invalid=" << statistics.invalidOrientationSnapshots
              << " fallbacks=" << statistics.predictionFallbacks << '\n';
}

} // namespace

RendererApplication::RendererApplication(RendererOptions options) : options_(std::move(options)) {}

int RendererApplication::run()
{
    const auto topology = platform::windows::enumerateDisplayTopology();
    const auto& monitors = topology.monitors;
    if (!topology.error.empty())
    {
        std::cerr << "Display topology warning: " << topology.error << '\n';
    }
    printMonitors(monitors);
    if (options_.monitorIndex >= monitors.size())
    {
        std::cerr << "Monitor index " << options_.monitorIndex << " does not exist.\n";
        return 1;
    }
    const auto& selectedMonitor = monitors[options_.monitorIndex];
    std::cout << "Selected monitor [" << selectedMonitor.index << "]: "
              << selectedMonitor.deviceName << '\n';
    if (!selectedMonitor.dxgiOutput.has_value())
    {
        std::cerr << "WARNING: selected monitor " << selectedMonitor.deviceName
                  << " has no matching active DXGI output. The default adapter will be used, "
                     "and monitor/adapter correspondence cannot be guaranteed.\n";
    }
    const OrientationToRenderMapping mapping;
    if (!validateRenderMapping(mapping))
    {
        std::cerr << "The configured sensor-to-render mapping is not a proper rotation.\n";
        return 1;
    }
    std::cout << "Render mapping: " << renderMappingText(mapping) << '\n'
              << "Orientation source: "
              << (options_.orientationSource == RenderOrientationSource::predicted
                      ? "predicted" : "measured") << '\n'
              << "Orientation frame: "
              << (options_.orientationFrame == RenderOrientationFrame::relative
                      ? "relative" : "absolute") << '\n';
    printControls(options_);

    bool exitRequested{};
    bool toggleSource{};
    bool toggleFrame{};
    bool toggleGrid{};
    bool toggleAxes{};
    bool toggleVsync{};
    bool toggleDiagnostics{};
    int demoYawSteps{};
    int demoPitchSteps{};
    int demoRollSteps{};
    bool demoReset{};
    OrientationRenderBridge bridge;
    platform::windows::RenderWindow window;
    platform::windows::RenderWindowConfig windowConfig;
    windowConfig.width = options_.windowWidth;
    windowConfig.height = options_.windowHeight;
    windowConfig.x = options_.windowX;
    windowConfig.y = options_.windowY;
    windowConfig.monitorIndex = options_.monitorIndex;
    windowConfig.monitorDeviceName = selectedMonitor.deviceName;
    windowConfig.fullscreen = options_.fullscreen;
    windowConfig.borderless = options_.borderless;
    windowConfig.hidden = options_.smokeTest;
    const bool created = window.create(windowConfig, [&](unsigned int key) {
        constexpr unsigned int escape = 0x1BU;
        constexpr unsigned int space = 0x20U;
        constexpr unsigned int left = 0x25U;
        constexpr unsigned int up = 0x26U;
        constexpr unsigned int right = 0x27U;
        constexpr unsigned int down = 0x28U;
        if (key == escape) { exitRequested = true; }
        else if (key == static_cast<unsigned int>(options_.recenterKey)) { bridge.requestRecenter(); }
        else if (key == static_cast<unsigned int>(options_.resetRecenterKey)) { bridge.requestClearRecenter(); }
        else if (key == 'P') { toggleSource = true; }
        else if (key == 'F') { toggleFrame = true; }
        else if (key == 'G') { toggleGrid = true; }
        else if (key == 'X') { toggleAxes = true; }
        else if (key == 'V') { toggleVsync = true; }
        else if (key == 'D') { toggleDiagnostics = true; }
        else if (key == left) { --demoYawSteps; }
        else if (key == right) { ++demoYawSteps; }
        else if (key == up) { ++demoPitchSteps; }
        else if (key == down) { --demoPitchSteps; }
        else if (key == 'Q') { --demoRollSteps; }
        else if (key == 'E') { ++demoRollSteps; }
        else if (key == space) { demoReset = true; }
    });
    if (!created)
    {
        std::cerr << window.error() << '\n';
        return 1;
    }

    const auto initialClientSize = window.takePendingResize();
    if (!initialClientSize.has_value()
        || initialClientSize->width == 0U || initialClientSize->height == 0U)
    {
        std::cerr << "The renderer window did not provide valid initial client dimensions.\n";
        return 1;
    }

    graphics::D3D11Renderer renderer;
    graphics::D3D11RendererConfig rendererConfig;
    rendererConfig.width = initialClientSize->width;
    rendererConfig.height = initialClientSize->height;
    rendererConfig.allowWarpFallback = options_.allowWarpFallback;
    rendererConfig.fullscreen = options_.fullscreen;
    rendererConfig.panelDistance = options_.panelDistance;
    rendererConfig.panelWidth = options_.panelWidth;
    rendererConfig.panelHeight = options_.panelHeight;
    if (selectedMonitor.dxgiOutput.has_value())
    {
        rendererConfig.selectedAdapterLuid = selectedMonitor.dxgiOutput->adapterLuid;
        rendererConfig.selectedOutputDeviceName = selectedMonitor.dxgiOutput->deviceName;
    }
    if (!renderer.initialize(window.nativeHandle(), rendererConfig))
    {
        std::cerr << renderer.error() << '\n';
        return 1;
    }
    const auto& graphicsInfo = renderer.information();
    std::cout << "D3D11 adapter: " << graphicsInfo.adapterName << '\n'
              << "Adapter LUID: "
              << (graphicsInfo.adapterLuid.has_value()
                      ? platform::windows::dxgiAdapterLuidText(*graphicsInfo.adapterLuid)
                      : "unavailable") << '\n'
              << "DXGI output: "
              << (graphicsInfo.outputDeviceName.empty()
                      ? "unmatched/default" : graphicsInfo.outputDeviceName) << '\n'
              << "Feature level: " << graphicsInfo.featureLevel << '\n'
              << "Device: " << (graphicsInfo.warp ? "WARP" : "hardware") << '\n'
              << "Swap chain: " << graphicsInfo.swapChainFormat
              << ", flip-discard, max latency 1, refresh="
              << (options_.vsync ? "vsync" : "immediate") << '\n';
    if (selectedMonitor.dxgiOutput.has_value() && !graphicsInfo.selectedAdapterMatched)
    {
        std::cerr << "WARNING: selected monitor adapter and active D3D11 adapter do not "
                     "correspond. This is expected only for an explicitly enabled WARP fallback.\n";
    }
    if (selectedMonitor.dxgiOutput.has_value() && !graphicsInfo.selectedOutputMatched)
    {
        std::cerr << "WARNING: selected monitor DXGI output was not selected by the renderer.\n";
    }
    if (!graphicsInfo.debugLayer)
    {
        std::cout << "D3D11 debug layer unavailable or disabled; rendering continues without it.\n";
    }

    RendererStartupState state = RendererStartupState::initializingRenderer;
    std::unique_ptr<DemoOrientationSource> demo;
    std::unique_ptr<SensorOrientationService> sensor;
    std::string startupError;
    if (options_.orientationDemoMode)
    {
        demo = std::make_unique<DemoOrientationSource>(bridge, options_.predictionHorizonMilliseconds);
        state = RendererStartupState::ready;
        if (options_.recenterOnStart)
        {
            bridge.requestRecenter();
        }
    }
    else
    {
        const auto sensorConfig = makeSensorConfig(options_, startupError);
        if (!sensorConfig.has_value())
        {
            std::cerr << startupError << '\n';
            return 1;
        }
        sensor = std::make_unique<SensorOrientationService>(bridge, *sensorConfig);
        std::cout << "XREAL Air 2 Ultra sensor-to-body mapping (experimental): "
                  << "gyro X -> -X, gyro Y -> +Y, gyro Z -> +Z; "
                  << "accel X -> +X, accel Y -> -Y, accel Z -> +Z\n";
        if (!sensor->start())
        {
            std::cerr << sensor->error() << '\n';
            return 1;
        }
        state = sensor->state();
    }

    FrameTimingTracker timing(options_.targetFramesPerSecond);
    RenderOrientationSelector selector;
    PerspectiveProjection projection{
        options_.fieldOfViewDegrees,
        static_cast<double>(initialClientSize->width)
            / static_cast<double>(initialClientSize->height),
        options_.nearPlane,
        options_.farPlane,
    };
    const auto start = std::chrono::steady_clock::now();
    auto nextDiagnostics = start;
    auto nextPacedFrame = start;
    std::uint64_t previousSnapshotSequence{};
    std::uint64_t renderedFrames{};
    std::string shutdownReason{"window_closed"};
    std::string runtimeError;
    ImuHealthCounters finalImu;
    std::uint64_t finalRecenterGeneration{};

    while (!exitRequested && window.processMessages())
    {
        if (window.minimized())
        {
            window.waitForMessageWhenMinimized();
            continue;
        }
        if (const auto resize = window.takePendingResize(); resize.has_value())
        {
            if (!renderer.resize(resize->width, resize->height))
            {
                runtimeError = renderer.error();
                shutdownReason = "resize_error";
                break;
            }
            projection.aspectRatio = static_cast<double>(resize->width)
                / static_cast<double>(resize->height);
        }
        if (toggleSource)
        {
            options_.orientationSource = options_.orientationSource == RenderOrientationSource::predicted
                ? RenderOrientationSource::measured : RenderOrientationSource::predicted;
            selector.reset();
            toggleSource = false;
        }
        if (toggleFrame)
        {
            options_.orientationFrame = options_.orientationFrame == RenderOrientationFrame::relative
                ? RenderOrientationFrame::absolute : RenderOrientationFrame::relative;
            selector.reset();
            toggleFrame = false;
        }
        if (toggleGrid) { options_.backgroundGrid = !options_.backgroundGrid; toggleGrid = false; }
        if (toggleAxes) { options_.worldAxes = !options_.worldAxes; toggleAxes = false; }
        if (toggleVsync) { options_.vsync = !options_.vsync; toggleVsync = false; }
        if (toggleDiagnostics) { options_.showRenderDiagnostics = !options_.showRenderDiagnostics; toggleDiagnostics = false; }

        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - start).count();
        if (demo)
        {
            constexpr double step = 2.0 * std::numbers::pi / 180.0;
            demo->adjustYaw(static_cast<double>(demoYawSteps) * step);
            demo->adjustPitch(static_cast<double>(demoPitchSteps) * step);
            demo->adjustRoll(static_cast<double>(demoRollSteps) * step);
            demoYawSteps = 0;
            demoPitchSteps = 0;
            demoRollSteps = 0;
            if (demoReset) { demo->reset(); demoReset = false; }
            demo->update(elapsed);
        }
        else if (sensor)
        {
            state = sensor->state();
            if (state == RendererStartupState::error)
            {
                runtimeError = sensor->error();
                shutdownReason = "imu_error";
                break;
            }
        }
        window.setTitle("XREAL Spatial Renderer - " + rendererStartupStateText(state));

        const auto snapshot = bridge.latest();
        SelectedRenderOrientation selected;
        if (snapshot.has_value())
        {
            selected = selector.select(*snapshot, options_.orientationSource,
                options_.orientationFrame, mapping);
            finalImu = snapshot->imu;
            finalRecenterGeneration = snapshot->recenterGeneration;
        }
        const bool repeated = snapshot.has_value()
            && snapshot->sequence == previousSnapshotSequence;
        if (snapshot.has_value()) { previousSnapshotSequence = snapshot->sequence; }
        const bool ready = state == RendererStartupState::ready && selected.valid;
        const auto viewProjection = makeViewProjection(
            ready ? selected.orientation : sensors::Quaternion::identity(), projection);
        if (!viewProjection.matrix.has_value())
        {
            runtimeError = viewProjection.error;
            shutdownReason = "camera_error";
            break;
        }

        timing.beginFrame();
        timing.recordSnapshot(snapshot.has_value() ? &*snapshot : nullptr,
            repeated, !selected.valid, selected.preservingLastValid);
        const bool rendered = renderer.render(*viewProjection.matrix,
            options_.backgroundGrid, options_.worldAxes, ready);
        const bool presented = rendered && renderer.present(options_.vsync);
        timing.endFrame(presented);
        if (!rendered || !presented)
        {
            runtimeError = renderer.error();
            shutdownReason = rendered ? "present_error" : "render_error";
            break;
        }
        ++renderedFrames;

        if (options_.showRenderDiagnostics && now >= nextDiagnostics)
        {
            printFrameDiagnostics(timing.statistics(), state,
                options_.orientationSource, options_.orientationFrame);
            nextDiagnostics = now + std::chrono::milliseconds(
                1000 / options_.renderDiagnosticsRateHz);
        }
        if (options_.smokeTest && renderedFrames >= options_.smokeTestFrames)
        {
            shutdownReason = "smoke_test_complete";
            break;
        }
        if (options_.renderDurationSeconds.has_value()
            && elapsed >= *options_.renderDurationSeconds)
        {
            shutdownReason = "duration_complete";
            break;
        }
        if (!options_.vsync && options_.targetFramesPerSecond.has_value())
        {
            nextPacedFrame += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(1.0 / *options_.targetFramesPerSecond));
            std::this_thread::sleep_until(nextPacedFrame);
        }
    }

    if (exitRequested)
    {
        shutdownReason = "escape_key";
    }
    const bool calibrationAccepted = demo != nullptr
        || (sensor != nullptr && sensor->calibrationAccepted());
    if (sensor)
    {
        sensor->stop();
    }
    state = runtimeError.empty() ? RendererStartupState::shuttingDown : RendererStartupState::error;
    const auto statistics = timing.statistics();
    std::cout << "Render summary: frames=" << statistics.renderFrameCount
              << " presents=" << statistics.presentCount
              << " average_fps=" << statistics.averageFramesPerSecond
              << " average_snapshot_age_ms=" << statistics.averageSnapshotAgeMilliseconds
              << " average_effective_lead_ms="
              << statistics.averageApproximateEffectiveLeadMilliseconds
              << " shutdown=" << shutdownReason << '\n';
    if (!runtimeError.empty())
    {
        std::cerr << runtimeError << '\n';
    }

    if (options_.renderJsonOutputPath.has_value())
    {
        RendererSummary summary;
        summary.options = options_;
        summary.graphics = renderer.information();
        summary.monitor = monitors[options_.monitorIndex];
        summary.mapping = mapping;
        summary.timing = statistics;
        summary.imu = finalImu;
        summary.finalState = state;
        summary.recenterGeneration = finalRecenterGeneration;
        summary.calibrationAccepted = calibrationAccepted;
        summary.shutdownReason = shutdownReason;
        summary.error = runtimeError;
        std::ofstream output(*options_.renderJsonOutputPath, std::ios::trunc);
        if (!output)
        {
            std::cerr << "Failed to open render JSON output: "
                      << *options_.renderJsonOutputPath << '\n';
            return 1;
        }
        output << serializeRendererSummaryJson(summary);
        if (!output)
        {
            std::cerr << "Failed to write render JSON output.\n";
            return 1;
        }
    }
    return runtimeError.empty() ? 0 : 1;
}

} // namespace xreal::rendering
