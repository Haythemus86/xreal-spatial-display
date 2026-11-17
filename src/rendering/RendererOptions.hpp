#pragma once

#include "capture/DesktopCaptureOptions.hpp"
#include "rendering/DesktopPanelDiagnostics.hpp"
#include "rendering/OrientationRenderBridge.hpp"
#include "rendering/PanelScene.hpp"
#include "sensors/OrientationPrediction.hpp"

#include <optional>
#include <string>

namespace xreal::rendering
{

struct RendererOptions
{
    unsigned int windowWidth{1280};
    unsigned int windowHeight{720};
    std::optional<int> windowX;
    std::optional<int> windowY;
    bool fullscreen{};
    bool borderless{};
    unsigned int monitorIndex{};
    std::optional<std::string> renderMonitorDeviceName;
    capture::DesktopCaptureOptions desktopCapture;
    bool vsync{true};
    bool allowWarpFallback{};
    std::optional<double> renderDurationSeconds;
    bool showRenderDiagnostics{};
    unsigned int renderDiagnosticsRateHz{2};
    std::optional<double> targetFramesPerSecond;
    bool backgroundGrid{};
    bool worldAxes{};
    double panelDistance{2.0};
    double panelWidth{1.6};
    double panelHeight{0.9};
    PanelScene panelScene{makeDefaultPanelScene()};
    capture::DesktopResolutionPolicy desktopResolutionPolicy{
        capture::DesktopResolutionPolicy::native};
    double desktopResolutionSafetyFactor{1.25};
    double desktopBandwidthWarningMebibytesPerSecond{1000.0};
    std::optional<std::string> panelLayoutFilePath;
    std::optional<std::string> panelLayoutSavePath;
    bool savePanelLayoutOnExit{};
    bool overwritePanelLayout{};
    bool multiPanelBenchmark{};
    double multiPanelBenchmarkSeconds{10.0};
    double multiPanelBenchmarkWarmupSeconds{2.0};
    std::size_t multiPanelBenchmarkPanels{3U};
    PanelContentKind multiPanelBenchmarkContent{PanelContentKind::synthetic};
    std::optional<std::string> multiPanelBenchmarkJsonPath;
    bool captureBenchmark{};
    std::size_t captureBenchmarkSources{1U};
    double captureBenchmarkSeconds{10.0};
    unsigned int captureBenchmarkTargetWidth{};
    unsigned int captureBenchmarkTargetHeight{};
    double captureBenchmarkFramesPerSecond{30.0};
    std::optional<std::string> captureBenchmarkJsonPath;
    double fieldOfViewDegrees{60.0};
    double nearPlane{0.05};
    double farPlane{100.0};
    RenderOrientationSource orientationSource{RenderOrientationSource::predicted};
    RenderOrientationFrame orientationFrame{RenderOrientationFrame::relative};
    bool recenterOnStart{};
    char recenterKey{'R'};
    char resetRecenterKey{'C'};
    bool orientationDemoMode{};
    bool orientationDemoStatic{};
    bool xrealSdkPose{};
    bool smokeTest{};
    unsigned int smokeTestFrames{10};
    bool desktopCaptureSmokeTest{};
    unsigned int desktopCaptureSmokeTestFrames{30};
    bool desktopDebugCheckerboard{};
    DesktopShaderDebugMode desktopShaderDebugMode{DesktopShaderDebugMode::normal};
    bool desktopDebugOpaqueBase{};
    std::optional<std::string> desktopDebugReadbackUploadPath;
    std::optional<std::string> desktopDebugRenderTargetPath;
    bool applyGyroscopeBias{};
    double gyroscopeCalibrationSeconds{2.0};
    double gyroscopeWarmupSeconds{1.0};
    std::optional<double> gyroscopeScaleRawPerDegreePerSecond;
    std::optional<std::string> accelerometerProfilePath;
    double accelerometerCorrectionTimeConstantSeconds{2.0};
    double accelerometerMaximumCorrectionDegreesPerSecond{10.0};
    bool fusionStartupGravity{true};
    bool predictOrientation{};
    sensors::PredictionMode predictionMode{sensors::PredictionMode::constantVelocity};
    double predictionHorizonMilliseconds{15.0};
    double predictionMaximumHorizonMilliseconds{50.0};
    std::optional<double> predictionAngularVelocitySmoothingSeconds{0.01};
    double predictionMaximumAngularSpeedDegreesPerSecond{1000.0};
    double predictionMaximumAngleDegrees{30.0};
    sensors::PredictionLimitBehavior predictionLimitBehavior{
        sensors::PredictionLimitBehavior::reject};
    std::optional<std::string> renderJsonOutputPath;
    bool help{};
};

struct RendererOptionResult
{
    std::optional<RendererOptions> options;
    std::string error;
};

[[nodiscard]] RendererOptionResult parseRendererOptions(int argc, char* argv[]);
[[nodiscard]] std::string rendererUsage();

} // namespace xreal::rendering
