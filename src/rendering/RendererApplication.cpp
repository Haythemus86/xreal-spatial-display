#include "rendering/RendererApplication.hpp"

#include "capture/DesktopCaptureDiagnostics.hpp"
#include "capture/DesktopCaptureManager.hpp"
#include "capture/DesktopCpuFrame.hpp"
#include "capture/DesktopDuplicationCapture.hpp"
#include "graphics/D3D11Renderer.hpp"
#include "platform/windows/RenderWindow.hpp"
#include "platform/windows/ProcessMemory.hpp"
#include "rendering/Camera.hpp"
#include "rendering/DemoOrientationSource.hpp"
#include "rendering/OrientationRenderBridge.hpp"
#include "rendering/PanelContentRegistry.hpp"
#include "rendering/PanelLayoutSerialization.hpp"
#include "rendering/PanelPerformance.hpp"
#include "rendering/PanelScene.hpp"
#include "rendering/RenderDiagnostics.hpp"
#include "rendering/RendererSummary.hpp"
#include "rendering/SensorOrientationService.hpp"

#include <algorithm>
#include <array>
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

[[nodiscard]] std::string bgraPixelText(const capture::BgraPixel& pixel)
{
    std::ostringstream output;
    output << std::hex << std::uppercase << std::setfill('0')
           << std::setw(2) << static_cast<unsigned int>(pixel.blue) << ' '
           << std::setw(2) << static_cast<unsigned int>(pixel.green) << ' '
           << std::setw(2) << static_cast<unsigned int>(pixel.red) << ' '
           << std::setw(2) << static_cast<unsigned int>(pixel.alpha);
    return output.str();
}

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
              << "  1/2/3: select panel; arrows: move selected panel\n"
              << "  Tab: select next panel; U: reset selected panel\n"
              << "  [/]: resize; I/O: distance; Q/E: yaw; W/S: pitch\n"
              << "  ,/.: gap; H: hide/show; L: layout; Z: reset layout\n"
              << "  K: save layout; Space: reset demo orientation\n";
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
    PanelScene scene = options_.panelScene;
    if (options_.panelLayoutFilePath.has_value())
    {
        std::string loadError;
        const auto text = readTextFile(*options_.panelLayoutFilePath, loadError);
        if (!text.has_value())
        {
            std::cerr << "Panel layout load failed: " << loadError << '\n';
            return 1;
        }
        const auto loaded = loadPanelLayoutJson(*text);
        if (!loaded.scene.has_value())
        {
            std::cerr << "Panel layout load failed: " << loaded.error << '\n';
            return 1;
        }
        scene = *loaded.scene;
    }
    PanelContentRegistry contentRegistry;
    const auto registryResult = contentRegistry.rebuild(scene);
    if (!registryResult.success)
    {
        std::cerr << "Panel source registry failed: " << registryResult.error << '\n';
        return 1;
    }
    std::size_t desktopSourceCount{};
    for (std::size_t slot = 0; slot < contentRegistry.sourceCount(); ++slot)
    {
        if (contentRegistry.source(slot).key.kind == PanelContentKind::desktop)
        {
            ++desktopSourceCount;
        }
    }
    if (desktopSourceCount > capture::maximumDesktopCaptureSources)
    {
        std::cerr << "The scene exceeds the fixed capacity of three desktop sources.\n";
        return 1;
    }
    if (desktopSourceCount > 0U)
    {
        options_.desktopCapture.panelContent = capture::PanelContent::desktop;
    }
    std::cout << "Panel scene: count=" << scene.panelCount
              << " layout=" << panelLayoutPresetText(scene.layout)
              << " unique_sources=" << contentRegistry.sourceCount()
              << " desktop_sources=" << desktopSourceCount
              << " performance_profile="
              << performanceProfileText(scene.performanceProfile) << '\n';
#ifndef NDEBUG
    if (options_.multiPanelBenchmark)
    {
        std::cerr << "WARNING: Debug benchmark results are diagnostic only. "
                     "Use the Release build for performance conclusions.\n";
    }
#endif
    for (std::size_t index = 0; index < scene.panelCount; ++index)
    {
        const auto& panel = scene.panels[index];
        std::cout << "  panel=" << index + 1U
                  << " id=" << static_cast<unsigned int>(panel.id.value)
                  << " enabled=" << (panel.enabled ? "yes" : "no")
                  << " name=\"" << panel.displayName << '"'
                  << " content=" << panelContentKindText(panel.content.kind)
                  << " source_slot=" << scene.runtime[index].sourceSlot
                  << " position_m=[" << panel.transform.position.x << ','
                  << panel.transform.position.y << ',' << panel.transform.position.z << ']'
                  << " size_m=" << panel.dimensions.width << 'x' << panel.dimensions.height
                  << " yaw=" << panel.transform.yawDegrees
                  << " pitch=" << panel.transform.pitchDegrees
                  << " fit=" << panelFitModeText(panel.fit)
                  << " filter=" << panelFilterModeText(panel.filter)
                  << " overlay=" << (panel.overlay.enabled ? "yes" : "no")
                  << " target_fps=" << panel.targetFramesPerSecond
                  << " target_fps_explicit="
                  << (panel.targetFramesPerSecondExplicit ? "yes" : "no")
                  << " source_target=" << panel.content.requestedWidth << 'x'
                  << panel.content.requestedHeight
                  << " source_scale=" << panel.content.requestedScale
                  << " transfer_policy="
                  << panelTransferPolicyText(panel.content.transferPolicy) << '\n';
    }
    const auto topology = platform::windows::enumerateDisplayTopology();
    const auto& monitors = topology.monitors;
    if (!topology.error.empty())
    {
        std::cerr << "Display topology warning: " << topology.error << '\n';
    }
    printMonitors(monitors);
    capture::MonitorSelector renderSelector;
    renderSelector.index = options_.monitorIndex;
    renderSelector.deviceName = options_.renderMonitorDeviceName;
    const auto renderSelection = capture::resolveMonitor(monitors, renderSelector, "Render");
    if (renderSelection.monitor == nullptr)
    {
        std::cerr << renderSelection.error << '\n';
        return 1;
    }
    const auto& selectedMonitor = *renderSelection.monitor;
    options_.monitorIndex = selectedMonitor.index;
    std::cout << "Selected render monitor [" << selectedMonitor.index << "]: "
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
    int selectPanel{-1};
    bool cyclePanelSelection{};
    double panelWidthDelta{};
    double panelHeightDelta{};
    Vector3 panelMoveDelta{};
    double panelYawDelta{};
    double panelPitchDelta{};
    double panelGapDelta{};
    bool togglePanelVisibility{};
    bool cyclePanelLayout{};
    bool resetPanelLayout{};
    bool resetSelectedPanel{};
    bool savePanelLayout{};
    PanelSceneController sceneController(scene);
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
    windowConfig.hidden = options_.smokeTest || options_.multiPanelBenchmark;
    const bool created = window.create(windowConfig, [&](unsigned int key) {
        constexpr unsigned int escape = 0x1BU;
        constexpr unsigned int space = 0x20U;
        constexpr unsigned int tab = 0x09U;
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
        else if (key >= '1' && key <= '3') { selectPanel = static_cast<int>(key - '1'); }
        else if (key == tab) { cyclePanelSelection = true; }
        else if (key == left) { panelMoveDelta.x -= 0.05; }
        else if (key == right) { panelMoveDelta.x += 0.05; }
        else if (key == up) { panelMoveDelta.y += 0.05; }
        else if (key == down) { panelMoveDelta.y -= 0.05; }
        else if (key == 'Q') { panelYawDelta -= 2.0; }
        else if (key == 'E') { panelYawDelta += 2.0; }
        else if (key == 'W') { panelPitchDelta += 2.0; }
        else if (key == 'S') { panelPitchDelta -= 2.0; }
        else if (key == 'I') { panelMoveDelta.z -= 0.05; }
        else if (key == 'O') { panelMoveDelta.z += 0.05; }
        else if (key == 0xDBU) { panelWidthDelta -= 0.05; }
        else if (key == 0xDDU) { panelWidthDelta += 0.05; }
        else if (key == 0xBCU) { panelGapDelta -= 0.02; }
        else if (key == 0xBEU) { panelGapDelta += 0.02; }
        else if (key == 'H') { togglePanelVisibility = true; }
        else if (key == 'L') { cyclePanelLayout = true; }
        else if (key == 'Z') { resetPanelLayout = true; }
        else if (key == 'U') { resetSelectedPanel = true; }
        else if (key == 'K') { savePanelLayout = true; }
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
    rendererConfig.desktopShaderDebugMode = options_.desktopShaderDebugMode;
    rendererConfig.desktopDebugOpaqueBase = options_.desktopDebugOpaqueBase;
    rendererConfig.desktopDebugReadbackUploadPath = options_.desktopDebugReadbackUploadPath;
    rendererConfig.desktopDebugRenderTargetPath = options_.desktopDebugRenderTargetPath;
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

    capture::DesktopCaptureBridge desktopBridge;
    capture::DesktopCaptureDiagnostics desktopDiagnostics;
    capture::DesktopCaptureManager desktopCaptureManager;
    std::array<std::optional<PanelSourceKey>,
        capture::maximumDesktopCaptureSources> startedDesktopSourceKeys{};
    std::array<capture::DesktopCaptureDiagnostics,
        capture::maximumDesktopCaptureSources> desktopSourceDiagnostics{};
    std::array<const platform::windows::MonitorInformation*,
        capture::maximumDesktopCaptureSources> captureMonitors{};
    capture::DesktopCaptureStatistics checkerboardCaptureStatistics;
    const platform::windows::MonitorInformation* captureMonitor{};
    std::optional<std::size_t> firstDesktopSourceSlot;
    if (options_.desktopCapture.panelContent == capture::PanelContent::desktop)
    {
        if (options_.desktopDebugCheckerboard)
        {
            if (!graphicsInfo.adapterLuid.has_value())
            {
                std::cerr << "The render adapter LUID is unavailable for checkerboard metadata.\n";
                return 1;
            }
            captureMonitor = &selectedMonitor;
            firstDesktopSourceSlot = 0U;
            auto checkerboard = capture::makeDesktopCheckerboardFrame(640U, 360U);
            if (!checkerboard.has_value())
            {
                std::cerr << "Failed to create the desktop debug checkerboard.\n";
                return 1;
            }
            const auto checkerboardContent = capture::analyzeDesktopFrameContent(*checkerboard);
            if (!checkerboardContent.has_value() || checkerboardContent->allZero
                || !checkerboardContent->allOpaque
                || checkerboardContent->distinctColorCount < 4U)
            {
                std::cerr << "Desktop debug checkerboard failed opacity or color validation.\n";
                return 1;
            }
            std::cout << "Desktop checkerboard source: dimensions=640x360"
                      << " stride=" << checkerboard->cpuRowPitch
                      << " format=" << checkerboard->sourceFormat
                      << " valid=yes sequence=" << checkerboard->sequence
                      << " bytes=" << checkerboardContent->byteCount
                      << " first_bgra=" << bgraPixelText(checkerboardContent->firstPixel)
                      << " center_bgra=" << bgraPixelText(checkerboardContent->centerPixel)
                      << " last_bgra=" << bgraPixelText(checkerboardContent->lastPixel)
                      << " distinct_colors=" << checkerboardContent->distinctColorCount
                      << " opaque=yes all_zero=no checksum=0x"
                      << std::hex << std::uppercase << checkerboardContent->checksum
                      << std::dec << '\n';
            checkerboard->sourceAdapterLuid = *graphicsInfo.adapterLuid;
            const std::uint64_t sequence = desktopBridge.publish(std::move(*checkerboard));
            if (sequence == 0U)
            {
                std::cerr << "Failed to publish the desktop debug checkerboard.\n";
                return 1;
            }
            checkerboardCaptureStatistics.state = capture::DesktopCaptureStatus::active;
            checkerboardCaptureStatistics.transferMode = "debug_cpu_checkerboard";
            checkerboardCaptureStatistics.acquiredFrames = 1U;
            checkerboardCaptureStatistics.stagingCopies = 1U;
            checkerboardCaptureStatistics.stagingMaps = 1U;
            checkerboardCaptureStatistics.stagingMapSuccesses = 1U;
            checkerboardCaptureStatistics.cpuBuffersCreated = 1U;
            checkerboardCaptureStatistics.cpuFramesPublished = 1U;
            checkerboardCaptureStatistics.latestPublishedSequence = sequence;
            checkerboardCaptureStatistics.sourceWidth = 640U;
            checkerboardCaptureStatistics.sourceHeight = 360U;
            checkerboardCaptureStatistics.sourceFormat = 87U;
            std::cout << "Desktop debug checkerboard published through CPU bridge.\n"
                      << "desktop-first-frame stage=acquired capture_candidate=1\n"
                      << "desktop-first-frame stage=cpu_staging_map_succeeded capture_candidate=1\n"
                      << "desktop-first-frame stage=cpu_frame_published sequence="
                      << sequence << '\n';
        }
        else
        {
            if (!graphicsInfo.adapterLuid.has_value())
            {
                std::cerr << "The render adapter LUID is unavailable; desktop transfer topology "
                             "cannot be validated.\n";
                return 1;
            }
            double aggregateEstimatedMebibytesPerSecond{};
            for (std::size_t sourceSlot = 0U;
                 sourceSlot < contentRegistry.sourceCount(); ++sourceSlot)
            {
                const auto& source = contentRegistry.source(sourceSlot);
                if (source.key.kind != PanelContentKind::desktop)
                {
                    continue;
                }
                const bool syntheticCapture = options_.captureBenchmark
                    && source.key.captureBenchmarkInstance > 0U;
                capture::MonitorSelector selectorForSource;
                selectorForSource.index = source.key.monitorIndex;
                if (!source.key.monitorDeviceName.empty())
                {
                    selectorForSource.deviceName = source.key.monitorDeviceName;
                }
                const auto captureSelection = capture::resolveMonitor(
                    monitors, selectorForSource, "Capture source "
                        + std::to_string(sourceSlot));
                if (captureSelection.monitor == nullptr)
                {
                    std::cerr << captureSelection.error << '\n';
                    return 1;
                }
                captureMonitors[sourceSlot] = captureSelection.monitor;
                if (!firstDesktopSourceSlot.has_value())
                {
                    firstDesktopSourceSlot = sourceSlot;
                }
                if (captureMonitor == nullptr)
                {
                    captureMonitor = captureSelection.monitor;
                }
                const bool captureSameAdapter = capture::sameAdapter(
                    selectedMonitor, *captureSelection.monitor);
                capture::DesktopDuplicationConfig captureConfig;
                captureConfig.captureMonitor = *captureSelection.monitor;
                captureConfig.renderAdapterLuid = *graphicsInfo.adapterLuid;
                captureConfig.options = options_.desktopCapture;
                captureConfig.options.captureMonitor = selectorForSource;
                captureConfig.options.crossAdapterPolicy =
                    source.key.transferPolicy == PanelTransferPolicy::sharedHandle
                    ? capture::CrossAdapterPolicy::sharedHandle
                    : source.key.transferPolicy == PanelTransferPolicy::cpuFallback
                        ? capture::CrossAdapterPolicy::cpuFallback
                        : source.key.transferPolicy == PanelTransferPolicy::reject
                            ? capture::CrossAdapterPolicy::reject
                            : capture::CrossAdapterPolicy::automatic;
                captureConfig.maximumFramesPerSecond = source.requestedFramesPerSecond;
                captureConfig.scaling.resolutionPolicy = source.key.resolutionPolicy;
                captureConfig.scaling.cropMode = source.key.cropMode;
                captureConfig.scaling.customRegion = source.key.customRegion;
                captureConfig.scaling.targetWidth = source.key.requestedWidth;
                captureConfig.scaling.targetHeight = source.key.requestedHeight;
                captureConfig.scaling.scale = source.key.requestedScale;
                captureConfig.scaling.fit = source.key.scaleFit;
                captureConfig.scaling.filter = source.key.scaleFilter;
                captureConfig.scaling.allowUpscale = source.key.allowUpscale;
                captureConfig.scaling.panelAwareSafetyFactor = source.key.safetyFactor;
                for (std::size_t panelIndex = 0U; panelIndex < scene.panelCount; ++panelIndex)
                {
                    if ((source.consumerMask & (1U << panelIndex)) == 0U)
                    {
                        continue;
                    }
                    const auto& panel = scene.panels[panelIndex];
                    const double distance = std::sqrt(
                        panel.transform.position.x * panel.transform.position.x
                        + panel.transform.position.y * panel.transform.position.y
                        + panel.transform.position.z * panel.transform.position.z);
                    const double angularWidth = distance > 0.0
                        ? 2.0 * std::atan(panel.dimensions.width / (2.0 * distance))
                            * 180.0 / std::numbers::pi : 0.0;
                    const auto projectedWidth = static_cast<std::uint32_t>(std::max(1.0,
                        std::clamp(angularWidth / options_.fieldOfViewDegrees, 0.0, 1.0)
                            * (selectedMonitor.right - selectedMonitor.left)));
                    const auto projectedHeight = static_cast<std::uint32_t>(std::max(1.0,
                        projectedWidth * panel.dimensions.height / panel.dimensions.width));
                    captureConfig.scaling.projectedWidth = std::max(
                        captureConfig.scaling.projectedWidth, projectedWidth);
                    captureConfig.scaling.projectedHeight = std::max(
                        captureConfig.scaling.projectedHeight, projectedHeight);
                }
                const std::uint32_t plannedSourceWidth = syntheticCapture
                    ? 3840U : static_cast<std::uint32_t>(
                        captureSelection.monitor->right - captureSelection.monitor->left);
                const std::uint32_t plannedSourceHeight = syntheticCapture
                    ? 2160U : static_cast<std::uint32_t>(
                        captureSelection.monitor->bottom - captureSelection.monitor->top);
                const auto planned = capture::resolveDesktopScalePlan(
                    plannedSourceWidth, plannedSourceHeight, captureConfig.scaling);
                if (!planned.plan.has_value())
                {
                    std::cerr << "Desktop source " << sourceSlot
                              << " scaling configuration failed: " << planned.error << '\n';
                    return 1;
                }
                const double estimatedMebibytesPerSecond =
                    static_cast<double>(planned.plan->targetBytesPerFrame)
                    * captureConfig.maximumFramesPerSecond / (1024.0 * 1024.0);
                aggregateEstimatedMebibytesPerSecond += estimatedMebibytesPerSecond;
                std::cout << "Desktop source " << sourceSlot
                          << ": monitor="
                          << (syntheticCapture ? "synthetic-benchmark"
                                  : captureSelection.monitor->deviceName)
                          << " adapter="
                          << (syntheticCapture ? "default D3D11 benchmark adapter"
                                  : captureSelection.monitor->dxgiOutput->adapterDescription)
                          << " topology=" << (captureSameAdapter ? "same" : "cross")
                          << " policy=" << capture::crossAdapterPolicyText(
                                 captureConfig.options.crossAdapterPolicy)
                          << " original=" << planned.plan->sourceWidth << 'x'
                          << planned.plan->sourceHeight
                          << " crop=[" << planned.plan->crop.x << ','
                          << planned.plan->crop.y << ',' << planned.plan->crop.width << ','
                          << planned.plan->crop.height << ']'
                          << " transfer=" << planned.plan->targetWidth << 'x'
                          << planned.plan->targetHeight
                          << " gpu_downscale="
                          << (planned.plan->gpuScaleRequired ? "yes" : "no")
                          << " capture_fps=" << captureConfig.maximumFramesPerSecond
                          << " estimated_readback_mib_s="
                          << estimatedMebibytesPerSecond << '\n';
                capture::DesktopCaptureSourceConfig managerConfig;
                managerConfig.sourceSlot = sourceSlot;
                managerConfig.synthetic = syntheticCapture;
                managerConfig.duplication = std::move(captureConfig);
                managerConfig.syntheticConfig.sourceOrdinal =
                    source.key.captureBenchmarkInstance;
                managerConfig.syntheticConfig.maximumFramesPerSecond =
                    source.requestedFramesPerSecond;
                managerConfig.syntheticConfig.scaling =
                    managerConfig.duplication.scaling;
                if (!desktopCaptureManager.startSource(std::move(managerConfig)))
                {
                    std::cerr << "Desktop capture source " << sourceSlot
                              << " failed to start: "
                              << desktopCaptureManager.error(sourceSlot) << '\n';
                    return 1;
                }
                startedDesktopSourceKeys[sourceSlot] = source.key;
            }
            if (aggregateEstimatedMebibytesPerSecond
                > options_.desktopBandwidthWarningMebibytesPerSecond)
            {
                std::cerr << "WARNING: aggregate configured desktop readback is "
                          << aggregateEstimatedMebibytesPerSecond
                          << " MiB/s, above the informational threshold of "
                          << options_.desktopBandwidthWarningMebibytesPerSecond
                          << " MiB/s.\n";
            }
        }
    }

    RendererStartupState state = RendererStartupState::initializingRenderer;
    std::unique_ptr<DemoOrientationSource> demo;
    std::unique_ptr<SensorOrientationService> sensor;
    std::string startupError;
    if (options_.orientationDemoMode)
    {
        demo = std::make_unique<DemoOrientationSource>(bridge,
            options_.orientationDemoStatic ? 0.0 : options_.predictionHorizonMilliseconds);
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
    const auto benchmarkMeasurementStart = start
        + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(options_.multiPanelBenchmark
                ? options_.multiPanelBenchmarkWarmupSeconds : 0.0));
    auto nextDiagnostics = start;
    auto nextPacedFrame = start;
    std::uint64_t previousSnapshotSequence{};
    std::uint64_t renderedFrames{};
    std::array<std::optional<capture::DesktopCaptureFrame>,
        capture::maximumDesktopCaptureSources> latestDesktopFrames{};
    std::array<std::chrono::steady_clock::time_point,
        capture::maximumDesktopCaptureSources> lastDesktopUploadTimes{};
    std::string shutdownReason{"window_closed"};
    std::string runtimeError;
    ImuHealthCounters finalImu;
    std::uint64_t finalRecenterGeneration{};
    MultiPanelPerformanceCounters panelPerformance;
    const auto initialMemory = platform::windows::currentProcessMemoryUsage();
    if (initialMemory.available)
    {
        panelPerformance.initialWorkingSetBytes = initialMemory.workingSetBytes;
        panelPerformance.peakWorkingSetBytes = initialMemory.workingSetBytes;
        panelPerformance.initialPrivateBytes = initialMemory.privateBytes;
        panelPerformance.peakPrivateBytes = initialMemory.privateBytes;
    }
    graphics::D3D11SceneStatistics benchmarkSceneBaseline;
    std::array<capture::DesktopCaptureStatistics,
        capture::maximumDesktopCaptureSources> benchmarkCaptureBaseline{};
    std::array<capture::DesktopRenderStageStatistics,
        capture::maximumDesktopCaptureSources> benchmarkUploadBaseline{};
    bool benchmarkSceneBaselineCaptured{};
    std::optional<RendererStartupState> lastWindowTitleState;

    while (!exitRequested && window.processMessages())
    {
        if (window.minimized())
        {
            window.waitForMessageWhenMinimized();
            continue;
        }
        const auto panelFrameStart = std::chrono::steady_clock::now();
        const bool recordPanelFrame = panelFrameStart >= benchmarkMeasurementStart;
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
        bool sceneChanged{};
        if (selectPanel >= 0)
        {
            sceneChanged = sceneController.select(
                static_cast<std::size_t>(selectPanel)) || sceneChanged;
            selectPanel = -1;
        }
        if (cyclePanelSelection)
        {
            (void)sceneController.select((scene.selectedPanel + 1U) % scene.panelCount);
            cyclePanelSelection = false;
            sceneChanged = true;
        }
        if (panelWidthDelta != 0.0 || panelHeightDelta != 0.0)
        {
            sceneChanged = sceneController.resizeSelected(
                panelWidthDelta, panelHeightDelta) || sceneChanged;
            panelWidthDelta = 0.0;
            panelHeightDelta = 0.0;
        }
        if (panelMoveDelta.x != 0.0 || panelMoveDelta.y != 0.0
            || panelMoveDelta.z != 0.0)
        {
            sceneChanged = sceneController.moveSelected(panelMoveDelta) || sceneChanged;
            panelMoveDelta = {};
        }
        if (panelYawDelta != 0.0 || panelPitchDelta != 0.0)
        {
            sceneChanged = sceneController.rotateSelected(
                panelYawDelta, panelPitchDelta) || sceneChanged;
            panelYawDelta = 0.0;
            panelPitchDelta = 0.0;
        }
        if (panelGapDelta != 0.0)
        {
            scene.gap = std::max(0.0, scene.gap + panelGapDelta);
            if (scene.layout != PanelLayoutPreset::custom)
            {
                (void)applyPanelLayout(scene, scene.layout);
            }
            panelGapDelta = 0.0;
            sceneChanged = true;
        }
        if (togglePanelVisibility)
        {
            (void)sceneController.toggleSelected();
            togglePanelVisibility = false;
            sceneChanged = true;
        }
        if (cyclePanelLayout)
        {
            PanelLayoutPreset next = PanelLayoutPreset::single;
            if (scene.panelCount == 2U)
            {
                next = PanelLayoutPreset::dualFlat;
            }
            else if (scene.panelCount == 3U)
            {
                next = scene.layout == PanelLayoutPreset::tripleAngled
                    ? PanelLayoutPreset::tripleFlat : PanelLayoutPreset::tripleAngled;
            }
            (void)sceneController.applyPreset(next);
            cyclePanelLayout = false;
            sceneChanged = true;
        }
        if (resetPanelLayout)
        {
            sceneController.reset();
            resetPanelLayout = false;
            sceneChanged = true;
        }
        if (resetSelectedPanel)
        {
            sceneChanged = sceneController.resetSelected() || sceneChanged;
            resetSelectedPanel = false;
        }
        if (sceneChanged)
        {
            const auto rebuilt = contentRegistry.rebuild(scene);
            if (!rebuilt.success)
            {
                runtimeError = rebuilt.error;
                shutdownReason = "panel_registry_error";
                break;
            }
            for (std::size_t sourceSlot = 0U;
                 sourceSlot < capture::maximumDesktopCaptureSources; ++sourceSlot)
            {
                const bool sourceStillPresent = sourceSlot < contentRegistry.sourceCount()
                    && contentRegistry.source(sourceSlot).key.kind
                        == PanelContentKind::desktop;
                if (!sourceStillPresent)
                {
                    if (desktopCaptureManager.active(sourceSlot))
                    {
                        desktopCaptureManager.setMaximumFramesPerSecond(sourceSlot, 0.0);
                    }
                    continue;
                }
                const auto& source = contentRegistry.source(sourceSlot);
                if (desktopCaptureManager.active(sourceSlot)
                    && startedDesktopSourceKeys[sourceSlot].has_value()
                    && *startedDesktopSourceKeys[sourceSlot] == source.key)
                {
                    desktopCaptureManager.setMaximumFramesPerSecond(
                        sourceSlot, source.requestedFramesPerSecond);
                }
                else if (!options_.desktopDebugCheckerboard)
                {
                    runtimeError = "Interactive capture source identity changed; restart "
                        "the renderer to create the new Desktop Duplication topology safely.";
                    shutdownReason = "desktop_source_topology_changed";
                    break;
                }
            }
            if (!runtimeError.empty())
            {
                break;
            }
        }
        if (savePanelLayout)
        {
            if (!options_.panelLayoutSavePath.has_value())
            {
                std::cerr << "Panel layout save requested, but --panel-layout-save-file was not provided.\n";
            }
            else
            {
                std::string saveError;
                if (!savePanelLayoutFileAtomic(*options_.panelLayoutSavePath, scene,
                        options_.overwritePanelLayout, saveError))
                {
                    std::cerr << "Panel layout save failed: " << saveError << '\n';
                }
                else
                {
                    std::cout << "Panel layout saved: "
                              << *options_.panelLayoutSavePath << '\n';
                }
            }
            savePanelLayout = false;
        }

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
            demo->update(options_.orientationDemoStatic ? 0.0 : elapsed);
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
        if (!lastWindowTitleState.has_value() || *lastWindowTitleState != state)
        {
            window.setTitle("XREAL Spatial Renderer - " + rendererStartupStateText(state));
            lastWindowTitleState = state;
        }

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

        if (options_.desktopCapture.panelContent == capture::PanelContent::desktop)
        {
            for (std::size_t sourceSlot = 0U;
                 sourceSlot < contentRegistry.sourceCount(); ++sourceSlot)
            {
                const auto& source = contentRegistry.source(sourceSlot);
                if (source.key.kind != PanelContentKind::desktop
                    || !desktopCaptureManager.active(sourceSlot))
                {
                    continue;
                }
                const auto captureState = desktopCaptureManager.statistics(sourceSlot);
                if (captureState.state == capture::DesktopCaptureStatus::fatalError
                    || captureState.state == capture::DesktopCaptureStatus::unsupported
                    || captureState.state == capture::DesktopCaptureStatus::outputMissing)
                {
                    runtimeError = "Desktop source " + std::to_string(sourceSlot)
                        + ": " + desktopCaptureManager.error(sourceSlot);
                    shutdownReason = "desktop_capture_error";
                    break;
                }
                auto frame = desktopCaptureManager.tryLatest(sourceSlot);
                if (!frame.has_value())
                {
                    continue;
                }
                latestDesktopFrames[sourceSlot] = std::move(frame);
                const double requestedUploadRate = source.requestedUploadFramesPerSecond;
                const bool uploadDue = requestedUploadRate > 0.0
                    && (lastDesktopUploadTimes[sourceSlot]
                            == std::chrono::steady_clock::time_point{}
                        || now - lastDesktopUploadTimes[sourceSlot]
                            >= std::chrono::duration_cast<
                                std::chrono::steady_clock::duration>(
                                std::chrono::duration<double>(
                                    1.0 / requestedUploadRate)));
                if (!uploadDue)
                {
                    continue;
                }
                if (!renderer.updateDesktopFrame(sourceSlot,
                        *latestDesktopFrames[sourceSlot], source.key.scaleFit,
                        source.key.scaleFilter, options_.desktopCapture.flipY))
                {
                    runtimeError = renderer.error();
                    shutdownReason = "desktop_transfer_error";
                    break;
                }
                lastDesktopUploadTimes[sourceSlot] = now;
                desktopSourceDiagnostics[sourceSlot].recordRendered(
                    *latestDesktopFrames[sourceSlot], now);
            }
            if (!runtimeError.empty())
            {
                break;
            }
            if (options_.desktopDebugCheckerboard)
            {
                if (auto frame = desktopBridge.tryLatest(); frame.has_value())
                {
                    latestDesktopFrames[0] = std::move(frame);
                    if (!renderer.updateDesktopFrame(0U, *latestDesktopFrames[0],
                            options_.desktopCapture.fit, options_.desktopCapture.filter,
                            options_.desktopCapture.flipY))
                    {
                        runtimeError = renderer.error();
                        shutdownReason = "desktop_transfer_error";
                        break;
                    }
                    desktopDiagnostics.recordRendered(*latestDesktopFrames[0], now);
                    desktopSourceDiagnostics[0].recordRendered(
                        *latestDesktopFrames[0], now);
                }
            }
        }

        timing.beginFrame();
        timing.recordSnapshot(snapshot.has_value() ? &*snapshot : nullptr,
            repeated, !selected.valid, selected.preservingLastValid);
        std::array<bool, capture::maximumDesktopCaptureSources> desktopStale{};
        for (std::size_t sourceSlot = 0U; sourceSlot < desktopStale.size(); ++sourceSlot)
        {
            desktopStale[sourceSlot] = !options_.desktopDebugCheckerboard
                && latestDesktopFrames[sourceSlot].has_value()
                && now - latestDesktopFrames[sourceSlot]->captureHostTimestamp
                    > std::chrono::milliseconds(
                        options_.desktopCapture.staleThresholdMilliseconds);
        }
        std::array<PanelRenderInstance, maximumPanelCount> panelInstances{};
        for (std::size_t panelIndex = 0; panelIndex < scene.panelCount; ++panelIndex)
        {
            const auto& panel = scene.panels[panelIndex];
            auto& runtime = scene.runtime[panelIndex];
            auto& instance = panelInstances[panelIndex];
            instance.id = panel.id;
            instance.dimensions = panel.dimensions;
            instance.content = panel.content.kind;
            instance.fit = panel.fit;
            instance.filter = panel.filter;
            instance.overlay = panel.overlay;
            instance.sourceSlot = runtime.sourceSlot;
            instance.visible = panelPotentiallyVisible(panel);
            const std::size_t sourceSlot = runtime.sourceSlot;
            const auto desktopUploadState = renderer.desktopStatistics(sourceSlot);
            instance.stale = panel.content.kind == PanelContentKind::desktop
                && sourceSlot < desktopStale.size() && desktopStale[sourceSlot];
            instance.selected = panelIndex == scene.selectedPanel;
            instance.worldViewProjection = *viewProjection.matrix
                * runtime.worldTransform;
            runtime.visible = instance.visible;
            runtime.culled = !instance.visible;
            runtime.stale = instance.stale;
            runtime.contentAvailable = panel.content.kind != PanelContentKind::unavailable
                && (panel.content.kind != PanelContentKind::desktop
                    || desktopUploadState.desktopSrvValid);
            runtime.effectiveContent = panel.content.kind == PanelContentKind::desktop
                && !runtime.contentAvailable
                ? PanelContentKind::unavailable : panel.content.kind;
            runtime.latestContentSequence = sourceSlot < latestDesktopFrames.size()
                && latestDesktopFrames[sourceSlot].has_value()
                && panel.content.kind == PanelContentKind::desktop
                ? latestDesktopFrames[sourceSlot]->sequence : 0U;
            runtime.latestUploadSequence = panel.content.kind == PanelContentKind::desktop
                ? desktopUploadState.latestUploadedSequence : 0U;
            runtime.frameAgeMilliseconds = sourceSlot < latestDesktopFrames.size()
                && latestDesktopFrames[sourceSlot].has_value()
                && panel.content.kind == PanelContentKind::desktop
                ? std::chrono::duration<double, std::milli>(
                    now - latestDesktopFrames[sourceSlot]->captureHostTimestamp).count()
                : 0.0;
        }
        const auto drawStart = std::chrono::steady_clock::now();
        if (recordPanelFrame && !benchmarkSceneBaselineCaptured)
        {
            benchmarkSceneBaseline = renderer.sceneStatistics();
            for (std::size_t sourceSlot = 0U;
                 sourceSlot < capture::maximumDesktopCaptureSources; ++sourceSlot)
            {
                benchmarkCaptureBaseline[sourceSlot] =
                    desktopCaptureManager.statistics(sourceSlot);
                benchmarkUploadBaseline[sourceSlot] =
                    renderer.desktopStatistics(sourceSlot);
            }
            benchmarkSceneBaselineCaptured = true;
        }
        // Preserve the validated one-panel checkerboard diagnostic pipeline, including
        // its ordered binding trace and optional GPU readback BMP. Normal single- and
        // multi-panel rendering use the bounded scene path below.
        const bool legacyCheckerboardDiagnostic = scene.panelCount == 1U
            && options_.desktopDebugCheckerboard;
        const bool rendered = legacyCheckerboardDiagnostic
            ? renderer.render(*viewProjection.matrix,
                  options_.backgroundGrid, options_.worldAxes, ready, true,
                  options_.desktopCapture.background, desktopStale[0])
            : renderer.renderScene(
                  std::span<const PanelRenderInstance>(
                      panelInstances.data(), scene.panelCount),
                  options_.backgroundGrid, options_.worldAxes, ready,
                  options_.desktopCapture.background);
        const auto drawEnd = std::chrono::steady_clock::now();
        const auto presentStart = drawEnd;
        const bool presented = rendered && renderer.present(options_.vsync);
        const auto panelFrameEnd = std::chrono::steady_clock::now();
        timing.endFrame(presented);
        if (!rendered || !presented)
        {
            runtimeError = renderer.error();
            shutdownReason = rendered ? "present_error" : "render_error";
            break;
        }
        ++renderedFrames;
        for (std::size_t panelIndex = 0; panelIndex < scene.panelCount; ++panelIndex)
        {
            if (panelInstances[panelIndex].visible)
            {
                ++scene.runtime[panelIndex].renderedFrames;
            }
        }
        if (recordPanelFrame)
        {
            const auto milliseconds = [](auto duration) {
                return std::chrono::duration<double, std::milli>(duration).count();
            };
            panelPerformance.frameTimes.add(milliseconds(panelFrameEnd - panelFrameStart));
            panelPerformance.cpuUpdateTimes.add(milliseconds(drawStart - panelFrameStart));
            panelPerformance.drawSubmissionTimes.add(milliseconds(drawEnd - drawStart));
            panelPerformance.presentTimes.add(milliseconds(panelFrameEnd - presentStart));
            ++panelPerformance.frames;
            if (presented)
            {
                ++panelPerformance.presents;
            }
            if ((panelPerformance.frames % 60U) == 0U)
            {
                double readbackBytesPerSecond{};
                double cpuTransferBytesPerSecond{};
                double uploadBytesPerSecond{};
                for (std::size_t sourceSlot = 0U;
                     sourceSlot < capture::maximumDesktopCaptureSources; ++sourceSlot)
                {
                    if (!desktopCaptureManager.active(sourceSlot))
                    {
                        continue;
                    }
                    const auto captureStatistics =
                        desktopCaptureManager.statistics(sourceSlot);
                    const auto uploadStatistics = renderer.desktopStatistics(sourceSlot);
                    const double targetBytes = static_cast<double>(
                        captureStatistics.transferWidth)
                        * captureStatistics.transferHeight * 4.0;
                    readbackBytesPerSecond += targetBytes
                        * captureStatistics.capturedFramesPerSecond;
                    cpuTransferBytesPerSecond += targetBytes
                        * captureStatistics.capturedFramesPerSecond;
                    uploadBytesPerSecond += static_cast<double>(
                        uploadStatistics.latestUploadBytes)
                        * uploadStatistics.uploadFramesPerSecond;
                }
                constexpr double bytesPerMebibyte = 1024.0 * 1024.0;
                panelPerformance.maximumObservedReadbackMebibytesPerSecond =
                    std::max(panelPerformance.maximumObservedReadbackMebibytesPerSecond,
                        readbackBytesPerSecond / bytesPerMebibyte);
                panelPerformance.maximumObservedCpuTransferMebibytesPerSecond =
                    std::max(panelPerformance.maximumObservedCpuTransferMebibytesPerSecond,
                        cpuTransferBytesPerSecond / bytesPerMebibyte);
                panelPerformance.maximumObservedUploadMebibytesPerSecond =
                    std::max(panelPerformance.maximumObservedUploadMebibytesPerSecond,
                        uploadBytesPerSecond / bytesPerMebibyte);
            }
        }

        if (options_.showRenderDiagnostics && now >= nextDiagnostics)
        {
            const auto diagnosticMemory = platform::windows::currentProcessMemoryUsage();
            if (diagnosticMemory.available)
            {
                panelPerformance.peakWorkingSetBytes = std::max(
                    panelPerformance.peakWorkingSetBytes,
                    diagnosticMemory.workingSetBytes);
                panelPerformance.peakPrivateBytes = std::max(
                    panelPerformance.peakPrivateBytes,
                    diagnosticMemory.privateBytes);
            }
            printFrameDiagnostics(timing.statistics(), state,
                options_.orientationSource, options_.orientationFrame);
            const auto multiPanelTiming = panelPerformance.frameTimes.statistics();
            const auto sceneStatistics = renderer.sceneStatistics();
            std::size_t currentVisiblePanels{};
            for (std::size_t panelIndex = 0; panelIndex < scene.panelCount; ++panelIndex)
            {
                currentVisiblePanels += static_cast<std::size_t>(
                    panelPotentiallyVisible(scene.panels[panelIndex]));
            }
            std::cout << "multi-panel panels=" << scene.panelCount
                      << " sources=" << contentRegistry.sourceCount()
                      << " selected=" << scene.selectedPanel + 1U
                      << " visible=" << currentVisiblePanels
                      << " culled=" << scene.panelCount - currentVisiblePanels
                      << " frame_p50_ms=" << multiPanelTiming.p50Milliseconds
                      << " frame_p95_ms=" << multiPanelTiming.p95Milliseconds
                      << " frame_p99_ms=" << multiPanelTiming.p99Milliseconds
                      << " frame_max_ms=" << multiPanelTiming.maximumMilliseconds
                      << " draws=" << sceneStatistics.drawCalls
                      << " base_draws=" << sceneStatistics.baseDrawCalls
                      << " overlay_draws=" << sceneStatistics.overlayDrawCalls
                      << " auxiliary_draws=" << sceneStatistics.auxiliaryDrawCalls
                      << " cb_updates=" << sceneStatistics.constantBufferUpdates
                      << " instance_updates=" << sceneStatistics.instanceBufferUpdates
                      << " state_sets=" << sceneStatistics.stateSetCalls
                      << " srv_binds=" << sceneStatistics.shaderResourceBindCalls
                      << " sampler_binds=" << sceneStatistics.samplerBindCalls
                      << " uploads=" << sceneStatistics.textureUploads
                      << " presents=" << sceneStatistics.presents << '\n';
            for (std::size_t panelIndex = 0; panelIndex < scene.panelCount; ++panelIndex)
            {
                const auto& panel = scene.panels[panelIndex];
                const std::size_t sourceSlot = scene.runtime[panelIndex].sourceSlot;
                const auto uploadStatistics = renderer.desktopStatistics(sourceSlot);
                const auto sourceCaptureStatistics = options_.desktopDebugCheckerboard
                    ? checkerboardCaptureStatistics
                    : desktopCaptureManager.statistics(sourceSlot);
                const double distance = std::sqrt(
                    panel.transform.position.x * panel.transform.position.x
                    + panel.transform.position.y * panel.transform.position.y
                    + panel.transform.position.z * panel.transform.position.z);
                const double angularWidthDegrees = distance > 0.0
                    ? 2.0 * std::atan(panel.dimensions.width / (2.0 * distance))
                        * 180.0 / std::numbers::pi
                    : 0.0;
                const double approximateCoveragePixels = std::clamp(
                    angularWidthDegrees / options_.fieldOfViewDegrees, 0.0, 1.0)
                    * static_cast<double>(selectedMonitor.right - selectedMonitor.left);
                const bool visible = panelPotentiallyVisible(panel);
                std::cout << "  panel=" << panelIndex + 1U
                          << " id=" << static_cast<unsigned int>(panel.id.value)
                          << " selected="
                          << (panelIndex == scene.selectedPanel ? "yes" : "no")
                          << " content=" << panelContentKindText(panel.content.kind)
                          << " source=" << scene.runtime[panelIndex].sourceSlot
                          << " enabled=" << (panel.enabled ? "yes" : "no")
                          << " draw_visible=" << (visible ? "yes" : "no")
                          << " size_m=" << panel.dimensions.width << 'x'
                          << panel.dimensions.height
                          << " distance_m=" << distance
                          << " position_m=[" << panel.transform.position.x << ','
                          << panel.transform.position.y << ','
                          << panel.transform.position.z << ']'
                          << " yaw_deg=" << panel.transform.yawDegrees
                          << " pitch_deg=" << panel.transform.pitchDegrees
                          << " angular_width_deg=" << angularWidthDegrees
                          << " approximate_output_width_px="
                          << approximateCoveragePixels
                          << " requested_fps=" << panel.targetFramesPerSecond
                          << " effective_fps="
                          << (panel.content.kind == PanelContentKind::desktop
                                  ? sourceCaptureStatistics.capturedFramesPerSecond : 0.0)
                          << " source_resolution="
                          << (panel.content.kind == PanelContentKind::desktop
                                  ? std::to_string(uploadStatistics.latestUploadWidth)
                                      + "x" + std::to_string(
                                          uploadStatistics.latestUploadHeight)
                                  : "procedural")
                          << " latest_sequence="
                          << scene.runtime[panelIndex].latestContentSequence
                          << " latest_upload_sequence="
                          << scene.runtime[panelIndex].latestUploadSequence
                          << " frame_age_ms="
                          << scene.runtime[panelIndex].frameAgeMilliseconds
                          << " stale="
                          << (panel.content.kind == PanelContentKind::desktop
                                  && sourceSlot < desktopStale.size()
                                  && desktopStale[sourceSlot] ? "yes" : "no") << '\n';
            }
            for (std::size_t sourceSlot = 0;
                 sourceSlot < contentRegistry.sourceCount(); ++sourceSlot)
            {
                const auto& source = contentRegistry.source(sourceSlot);
                std::cout << "  source=" << sourceSlot
                          << " kind=" << panelContentKindText(source.key.kind)
                          << " consumers=0x" << std::hex
                          << static_cast<unsigned int>(source.consumerMask) << std::dec
                          << " requested_fps=" << source.requestedFramesPerSecond;
                if (source.key.kind == PanelContentKind::desktop)
                {
                    const auto sourceCaptureStatistics = options_.desktopDebugCheckerboard
                        ? checkerboardCaptureStatistics
                        : desktopCaptureManager.statistics(sourceSlot);
                    const auto uploadStatistics = renderer.desktopStatistics(sourceSlot);
                    const auto sourceBridgeStatistics = options_.desktopDebugCheckerboard
                        ? desktopBridge.statistics()
                        : desktopCaptureManager.bridgeStatistics(sourceSlot);
                    const auto sourceRenderStatistics =
                        desktopSourceDiagnostics[sourceSlot].snapshot();
                    std::cout << " capture_sequence="
                              << sourceCaptureStatistics.latestPublishedSequence
                              << " upload_sequence="
                              << uploadStatistics.latestUploadedSequence
                              << " capture_fps="
                              << sourceCaptureStatistics.capturedFramesPerSecond
                              << " original_dimensions="
                              << sourceCaptureStatistics.sourceWidth << 'x'
                              << sourceCaptureStatistics.sourceHeight
                              << " crop=[" << sourceCaptureStatistics.cropX << ','
                              << sourceCaptureStatistics.cropY << ','
                              << sourceCaptureStatistics.cropWidth << ','
                              << sourceCaptureStatistics.cropHeight << ']'
                              << " transfer_dimensions="
                              << sourceCaptureStatistics.transferWidth << 'x'
                              << sourceCaptureStatistics.transferHeight
                              << " upload_dimensions="
                              << uploadStatistics.latestUploadWidth << 'x'
                              << uploadStatistics.latestUploadHeight
                              << " transferred_bytes="
                              << sourceCaptureStatistics.stagingBytesMapped
                              << " upload_bytes="
                              << uploadStatistics.uploadBytesSubmitted
                              << " upload_fps="
                              << uploadStatistics.uploadFramesPerSecond
                              << " availability="
                              << capture::desktopFrameAvailabilityText(
                                  sourceCaptureStatistics.availability)
                              << " map_wait_avg_ms="
                              << sourceCaptureStatistics.averageMapWaitMilliseconds
                              << " map_wait_max_ms="
                              << sourceCaptureStatistics.maximumMapWaitMilliseconds
                              << " update_avg_ms="
                              << uploadStatistics.averageUpdateSubresourceMilliseconds
                              << " repeated="
                              << sourceRenderStatistics.repeatedCaptureFrames
                              << " dropped="
                              << sourceBridgeStatistics.droppedPublications
                              << " contended_reads="
                              << sourceBridgeStatistics.contendedReads
                              << " stale="
                              << (sourceSlot < desktopStale.size()
                                      && desktopStale[sourceSlot] ? "yes" : "no");
                }
                std::cout << '\n';
            }
            if (options_.desktopCapture.panelContent == capture::PanelContent::desktop
                && options_.desktopCapture.diagnostics)
            {
                for (std::size_t sourceSlot = 0U;
                     sourceSlot < contentRegistry.sourceCount(); ++sourceSlot)
                {
                    if (contentRegistry.source(sourceSlot).key.kind
                        != PanelContentKind::desktop)
                    {
                        continue;
                    }
                    const auto captureStatistics = options_.desktopDebugCheckerboard
                        ? checkerboardCaptureStatistics
                        : desktopCaptureManager.statistics(sourceSlot);
                    const auto bridgeStatistics = options_.desktopDebugCheckerboard
                        ? desktopBridge.statistics()
                        : desktopCaptureManager.bridgeStatistics(sourceSlot);
                    const auto renderCaptureStatistics =
                        desktopSourceDiagnostics[sourceSlot].snapshot();
                    const auto upload = renderer.desktopStatistics(sourceSlot);
                    const auto* sourceMonitor = captureMonitors[sourceSlot] != nullptr
                        ? captureMonitors[sourceSlot] : captureMonitor;
                    std::cout << "capture source=" << sourceSlot << " status="
                          << capture::desktopCaptureStatusText(captureStatistics.state)
                          << " capture_monitor="
                          << (sourceMonitor != nullptr
                                  ? sourceMonitor->deviceName : "synthetic")
                          << " transfer_mode=" << captureStatistics.transferMode
                          << " capture_frame_sequence="
                          << (latestDesktopFrames[sourceSlot].has_value()
                                  ? latestDesktopFrames[sourceSlot]->sequence : 0U)
                          << " capture_fps=" << captureStatistics.capturedFramesPerSecond
                          << " source=" << captureStatistics.sourceWidth << 'x'
                          << captureStatistics.sourceHeight
                          << " format=" << captureStatistics.sourceFormat
                          << " rotation="
                          << static_cast<unsigned int>(captureStatistics.sourceRotation)
                          << " source_frame_age_ms="
                          << renderCaptureStatistics.averageSourceFrameAgeMilliseconds
                          << " repeated=" << renderCaptureStatistics.repeatedCaptureFrames
                          << " dropped=" << bridgeStatistics.droppedPublications
                          << " access_loss_count=" << captureStatistics.accessLossEvents
                          << " recovery_generation=" << captureStatistics.recoveryGeneration
                          << " frames_acquired=" << captureStatistics.acquiredFrames
                          << " staging_copies=" << captureStatistics.stagingCopies
                          << " staging_maps=" << captureStatistics.stagingMaps
                          << " staging_map_successes=" << captureStatistics.stagingMapSuccesses
                          << " cpu_buffers_created=" << captureStatistics.cpuBuffersCreated
                          << " cpu_frames_published=" << captureStatistics.cpuFramesPublished
                          << " latest_published_sequence=" << captureStatistics.latestPublishedSequence
                          << " cpu_frames_seen=" << upload.cpuFramesSeen
                          << " cpu_frames_consumed=" << upload.cpuFramesConsumed
                          << " cpu_frames_skipped_same_sequence=" << upload.cpuFramesSkippedSameSequence
                          << " latest_consumed_sequence=" << upload.latestConsumedSequence
                          << " upload_texture_creations=" << upload.uploadTextureCreations
                          << " upload_texture_recreations=" << upload.uploadTextureRecreations
                          << " update_subresource_calls=" << upload.updateSubresourceCalls
                          << " update_subresource_failures=" << upload.updateSubresourceFailures
                          << " latest_uploaded_sequence=" << upload.latestUploadedSequence
                          << " latest_upload=" << upload.latestUploadWidth << 'x'
                          << upload.latestUploadHeight << ':' << upload.latestUploadFormat
                          << " upload_texture_valid=" << (upload.uploadTextureValid ? "yes" : "no")
                          << " desktop_srv_creations=" << upload.desktopSrvCreations
                          << " desktop_srv_failures=" << upload.desktopSrvFailures
                          << " desktop_srv_bind_count=" << upload.desktopSrvBindCount
                          << " latest_bound_sequence=" << upload.latestBoundSequence
                          << " desktop_srv_valid=" << (upload.desktopSrvValid ? "yes" : "no")
                          << " panel_content_requested="
                          << capture::panelContentText(upload.panelContentRequested)
                          << " panel_content_effective="
                          << capture::desktopPanelEffectiveModeText(upload.panelContentEffective)
                          << " desktop_texture_available="
                          << (upload.desktopTextureAvailable ? "yes" : "no")
                          << " rendered_desktop_frames=" << upload.renderedDesktopFrames
                          << " rendered_unavailable_frames=" << upload.renderedUnavailableFrames
                          << " rendered_synthetic_frames=" << upload.renderedSyntheticFrames
                          << '\n';
                }
            }
            nextDiagnostics = now + std::chrono::milliseconds(
                1000 / options_.renderDiagnosticsRateHz);
        }
        if (options_.desktopCaptureSmokeTest
            && desktopCaptureManager.activeSourceCount() > 0U)
        {
            const auto captureStatistics = desktopCaptureManager.statistics(
                firstDesktopSourceSlot.value_or(0U));
            if (captureStatistics.acquiredFrames >= options_.desktopCaptureSmokeTestFrames)
            {
                shutdownReason = "desktop_capture_smoke_test_complete";
                break;
            }
            if (elapsed >= 10.0)
            {
                runtimeError = "Desktop capture smoke test timed out before acquiring the requested frames.";
                shutdownReason = "desktop_capture_smoke_test_timeout";
                break;
            }
        }
        else if (options_.smokeTest && renderedFrames >= options_.smokeTestFrames)
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
    std::array<capture::DesktopCaptureStatistics,
        capture::maximumDesktopCaptureSources> finalCaptureStatistics{};
    std::array<capture::DesktopCaptureBridgeStatistics,
        capture::maximumDesktopCaptureSources> finalBridgeStatistics{};
    for (std::size_t sourceSlot = 0U;
         sourceSlot < capture::maximumDesktopCaptureSources; ++sourceSlot)
    {
        if (desktopCaptureManager.active(sourceSlot))
        {
            finalCaptureStatistics[sourceSlot] =
                desktopCaptureManager.detailedStatistics(sourceSlot);
            finalBridgeStatistics[sourceSlot] =
                desktopCaptureManager.bridgeStatistics(sourceSlot);
        }
    }
    if (options_.desktopDebugCheckerboard)
    {
        finalCaptureStatistics[0] = checkerboardCaptureStatistics;
        finalBridgeStatistics[0] = desktopBridge.statistics();
    }
    desktopCaptureManager.stop();
    state = runtimeError.empty() ? RendererStartupState::shuttingDown : RendererStartupState::error;
    const auto statistics = timing.statistics();
    const auto finalSceneStatistics = renderer.sceneStatistics();
    panelPerformance.drawCalls = finalSceneStatistics.drawCalls
        - benchmarkSceneBaseline.drawCalls;
    panelPerformance.baseDrawCalls = finalSceneStatistics.baseDrawCalls
        - benchmarkSceneBaseline.baseDrawCalls;
    panelPerformance.overlayDrawCalls = finalSceneStatistics.overlayDrawCalls
        - benchmarkSceneBaseline.overlayDrawCalls;
    panelPerformance.auxiliaryDrawCalls = finalSceneStatistics.auxiliaryDrawCalls
        - benchmarkSceneBaseline.auxiliaryDrawCalls;
    panelPerformance.stateSetCalls = finalSceneStatistics.stateSetCalls
        - benchmarkSceneBaseline.stateSetCalls;
    panelPerformance.shaderResourceBindCalls =
        finalSceneStatistics.shaderResourceBindCalls
        - benchmarkSceneBaseline.shaderResourceBindCalls;
    panelPerformance.samplerBindCalls = finalSceneStatistics.samplerBindCalls
        - benchmarkSceneBaseline.samplerBindCalls;
    panelPerformance.constantBufferUpdates = finalSceneStatistics.constantBufferUpdates
        - benchmarkSceneBaseline.constantBufferUpdates;
    panelPerformance.instanceBufferUpdates = finalSceneStatistics.instanceBufferUpdates
        - benchmarkSceneBaseline.instanceBufferUpdates;
    panelPerformance.textureUploads = finalSceneStatistics.textureUploads
        - benchmarkSceneBaseline.textureUploads;
    panelPerformance.resourcesCreatedAtStartup =
        finalSceneStatistics.resourcesCreatedAtStartup;
    panelPerformance.resourcesCreatedSteadyState =
        finalSceneStatistics.resourcesCreatedSteadyState
        - benchmarkSceneBaseline.resourcesCreatedSteadyState;
    panelPerformance.flushCalls = finalSceneStatistics.flushCalls
        - benchmarkSceneBaseline.flushCalls;
    for (std::size_t sourceSlot = 0U;
         sourceSlot < capture::maximumDesktopCaptureSources; ++sourceSlot)
    {
        const auto& captureStatistics = finalCaptureStatistics[sourceSlot];
        panelPerformance.captureFrames += captureStatistics.acquiredFrames
            - std::min(captureStatistics.acquiredFrames,
                benchmarkCaptureBaseline[sourceSlot].acquiredFrames);
        panelPerformance.readbackBytes += captureStatistics.stagingBytesMapped
            - std::min(captureStatistics.stagingBytesMapped,
                benchmarkCaptureBaseline[sourceSlot].stagingBytesMapped);
        panelPerformance.cpuTransferBytes += captureStatistics.cpuBytesCopied
            - std::min(captureStatistics.cpuBytesCopied,
                benchmarkCaptureBaseline[sourceSlot].cpuBytesCopied);
        const auto finalUpload = renderer.desktopStatistics(sourceSlot);
        panelPerformance.uploadBytes += finalUpload.uploadBytesSubmitted
            - std::min(finalUpload.uploadBytesSubmitted,
                benchmarkUploadBaseline[sourceSlot].uploadBytesSubmitted);
        panelPerformance.droppedFrames +=
            finalBridgeStatistics[sourceSlot].droppedPublications;
        panelPerformance.repeatedFrames +=
            desktopSourceDiagnostics[sourceSlot].snapshot().repeatedCaptureFrames;
    }
    const auto memory = platform::windows::currentProcessMemoryUsage();
    if (memory.available)
    {
        panelPerformance.workingSetBytes = memory.workingSetBytes;
        panelPerformance.privateBytes = memory.privateBytes;
        panelPerformance.peakWorkingSetBytes = std::max(
            panelPerformance.peakWorkingSetBytes, memory.workingSetBytes);
        panelPerformance.peakPrivateBytes = std::max(
            panelPerformance.peakPrivateBytes, memory.privateBytes);
    }
    const auto panelFrameStatistics = panelPerformance.frameTimes.statistics();
    std::cout << "Render summary: frames=" << statistics.renderFrameCount
              << " presents=" << statistics.presentCount
              << " average_fps=" << statistics.averageFramesPerSecond
              << " average_snapshot_age_ms=" << statistics.averageSnapshotAgeMilliseconds
              << " average_effective_lead_ms="
              << statistics.averageApproximateEffectiveLeadMilliseconds
              << " shutdown=" << shutdownReason << '\n'
              << "Multi-panel performance: measured_frames=" << panelPerformance.frames
              << " frame_avg_ms=" << panelFrameStatistics.averageMilliseconds
              << " p50_ms=" << panelFrameStatistics.p50Milliseconds
              << " p95_ms=" << panelFrameStatistics.p95Milliseconds
              << " p99_ms=" << panelFrameStatistics.p99Milliseconds
              << " max_ms=" << panelFrameStatistics.maximumMilliseconds
              << " over_33_ms=" << panelFrameStatistics.overBudget33Milliseconds
              << " draws=" << panelPerformance.drawCalls
              << " base_draws=" << panelPerformance.baseDrawCalls
              << " overlay_draws=" << panelPerformance.overlayDrawCalls
              << " auxiliary_draws=" << panelPerformance.auxiliaryDrawCalls
              << " presents=" << panelPerformance.presents
              << " cb_updates=" << panelPerformance.constantBufferUpdates
              << " instance_updates=" << panelPerformance.instanceBufferUpdates
              << " state_sets=" << panelPerformance.stateSetCalls
              << " srv_binds=" << panelPerformance.shaderResourceBindCalls
              << " sampler_binds=" << panelPerformance.samplerBindCalls
              << " uploads=" << panelPerformance.textureUploads
              << " steady_resource_creations="
              << panelPerformance.resourcesCreatedSteadyState
              << " flushes=" << panelPerformance.flushCalls
              << " working_set_bytes=" << panelPerformance.workingSetBytes
              << " private_bytes=" << panelPerformance.privateBytes << '\n';
    if (!runtimeError.empty())
    {
        std::cerr << runtimeError << '\n';
    }

    if (options_.multiPanelBenchmarkJsonPath.has_value())
    {
        const double measurementSeconds = std::max(0.0,
            std::chrono::duration<double>(std::chrono::steady_clock::now()
                - benchmarkMeasurementStart).count());
#ifdef NDEBUG
        constexpr std::string_view buildConfiguration = "Release";
#else
        constexpr std::string_view buildConfiguration = "Debug";
#endif
        std::ofstream output(*options_.multiPanelBenchmarkJsonPath, std::ios::trunc);
        if (!output)
        {
            std::cerr << "Failed to open multi-panel benchmark JSON: "
                      << *options_.multiPanelBenchmarkJsonPath << '\n';
            return 1;
        }
        output << serializeMultiPanelPerformanceJson(panelPerformance,
            measurementSeconds, scene.panelCount, contentRegistry.sourceCount(),
            graphicsInfo.adapterName, graphicsInfo.outputDeviceName,
            buildConfiguration);
        if (!output)
        {
            std::cerr << "Failed to write multi-panel benchmark JSON.\n";
            return 1;
        }
    }

    if (options_.savePanelLayoutOnExit)
    {
        if (!options_.panelLayoutSavePath.has_value())
        {
            std::cerr << "--save-panel-layout-on-exit requires --panel-layout-save-file.\n";
            return 1;
        }
        std::string saveError;
        if (!savePanelLayoutFileAtomic(*options_.panelLayoutSavePath, scene,
                options_.overwritePanelLayout, saveError))
        {
            std::cerr << "Panel layout save failed: " << saveError << '\n';
            return 1;
        }
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
        summary.desktop = renderer.desktopStatistics();
        if (firstDesktopSourceSlot.has_value())
        {
            summary.capture = finalCaptureStatistics[*firstDesktopSourceSlot];
        }
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
    if (options_.desktopCapture.panelContent == capture::PanelContent::desktop
        && options_.desktopCapture.jsonOutputPath.has_value()
        && captureMonitor != nullptr)
    {
        capture::DesktopCaptureSummary summary;
        summary.renderMonitor = selectedMonitor;
        summary.captureMonitor = *captureMonitor;
        summary.options = options_.desktopCapture;
        const std::size_t sourceSlot = firstDesktopSourceSlot.value_or(0U);
        summary.capture = finalCaptureStatistics[sourceSlot];
        summary.bridge = finalBridgeStatistics[sourceSlot];
        summary.render = desktopSourceDiagnostics[sourceSlot].snapshot();
        summary.stages = renderer.desktopStatistics(sourceSlot);
        summary.sameAdapter = capture::sameAdapter(selectedMonitor, *captureMonitor);
        summary.sharedHandleSupported = summary.capture.gpuCopies > 0U;
        summary.cpuFallbackUsed = summary.capture.cpuFallbackCopies > 0U;
        summary.finalError = runtimeError;
        std::ofstream output(*options_.desktopCapture.jsonOutputPath, std::ios::trunc);
        if (!output)
        {
            std::cerr << "Failed to open desktop capture JSON output: "
                      << *options_.desktopCapture.jsonOutputPath << '\n';
            return 1;
        }
        output << capture::serializeDesktopCaptureSummaryJson(summary);
        if (!output)
        {
            std::cerr << "Failed to write desktop capture JSON output.\n";
            return 1;
        }
    }
    return runtimeError.empty() ? 0 : 1;
}

} // namespace xreal::rendering
