#include "rendering/RenderDiagnostics.hpp"
#include "rendering/RendererSummary.hpp"
#include "JsonSyntaxParser.hpp"

#include <chrono>
#include <iostream>
#include <limits>

int main()
{
    using namespace xreal::rendering;
    FrameTimingTracker tracker(60.0);
    tracker.beginFrame();
    RenderOrientationSnapshot snapshot;
    snapshot.sequence = 1;
    snapshot.hostPublishTimestampNanoseconds = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
    snapshot.predictionHorizonMilliseconds = 15.0;
    snapshot.deviceTimestampNanoseconds = 42;
    tracker.recordSnapshot(&snapshot, false, false, false);
    tracker.endFrame(true);
    const auto statistics = tracker.statistics();
    if (statistics.renderFrameCount != 1U || statistics.presentCount != 1U
        || !std::isfinite(statistics.averageSnapshotAgeMilliseconds))
    {
        std::cerr << "Frame diagnostics are invalid.\n";
        return 1;
    }
    RendererSummary summary;
    summary.options.orientationDemoMode = true;
    summary.graphics.adapterName = "Synthetic adapter";
    summary.graphics.featureLevel = "11.0";
    summary.monitor.deviceName = "DISPLAY1";
    summary.timing = statistics;
    summary.timing.maximumFrameTimeMilliseconds = std::numeric_limits<double>::infinity();
    const std::string json = serializeRendererSummaryJson(summary);
    if (!JsonSyntaxParser(json).valid() || json.find("inf") != std::string::npos
        || json.find("nan") != std::string::npos)
    {
        std::cerr << "Renderer JSON is invalid or contains non-finite values.\n";
        return 1;
    }
    if (rendererStartupStateText(RendererStartupState::initializingRenderer)
            != "initializing_renderer"
        || rendererStartupStateText(RendererStartupState::shuttingDown) != "shutting_down")
    {
        std::cerr << "Startup state transition text is not deterministic.\n";
        return 1;
    }
    std::cout << "All render diagnostic tests passed.\n";
    return 0;
}
