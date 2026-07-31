#include "rendering/PanelContentRegistry.hpp"
#include "rendering/PanelLayoutSerialization.hpp"
#include "rendering/PanelPerformance.hpp"
#include "rendering/PanelScene.hpp"
#include "JsonSyntaxParser.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>

namespace
{

int failures{};
int checks{};

static_assert(xreal::rendering::maximumPanelCount == 3U);
static_assert(std::tuple_size_v<decltype(xreal::rendering::PanelScene::panels)> == 3U);

void expect(bool condition, const char* message)
{
    ++checks;
    if (!condition)
    {
        std::cerr << "FAILED: " << message << '\n';
        ++failures;
    }
}

void testSceneAndLayouts()
{
    using namespace xreal::rendering;
    const auto defaultScene = makeDefaultPanelScene();
    expect(defaultScene.panelCount == 1U,
        "default panel scene contains exactly one panel");
    for (std::size_t count = 1U; count <= maximumPanelCount; ++count)
    {
        expect(validatePanelScene(makeDefaultPanelScene(count)).valid,
            "panel counts one through three validate");
    }
    auto scene = makeDefaultPanelScene(3U);
    expect(scene.panelCount == 3U, "three-panel scene has requested count");
    expect(scene.layout == PanelLayoutPreset::tripleAngled,
        "three-panel default is angled");
    expect(validatePanelScene(scene).valid, "default scene validates");
    expect(scene.panels[0].id == PanelId{0U}, "first panel identifier is stable");
    expect(scene.panels[1].id == PanelId{1U}, "second panel identifier is stable");
    expect(scene.panels[2].id == PanelId{2U}, "third panel identifier is stable");
    expect(scene.panels[0].displayName == "Panel 1"
        && scene.panels[2].displayName == "Panel 3",
        "default display names are deterministic");
    expect(scene.panels[0].transform.position.x < 0.0,
        "left panel is left of center");
    expect(scene.panels[1].transform.position.x == 0.0,
        "middle panel is centered");
    expect(scene.panels[2].transform.position.x > 0.0,
        "right panel is right of center");
    expect(std::abs(scene.panels[0].transform.position.x
        + scene.panels[2].transform.position.x) < 1.0e-12,
        "triple layout is horizontally symmetric");
    expect(std::abs((scene.panels[1].transform.position.x
        - scene.panels[0].transform.position.x)
        - (scene.defaultWidth + scene.gap)) < 1.0e-12,
        "flat projected centers preserve the configured edge gap");
    expect(std::abs(scene.panels[0].transform.yawDegrees
        + scene.panels[2].transform.yawDegrees) < 1.0e-12,
        "angled layout yaw is symmetric");
    expect(panelWorldMatrix(scene.panels[0]).finite(), "panel world matrix is finite");
    expect(scene.runtime[0].worldTransform.finite(),
        "layout caches a finite world transform");
    expect(panelPotentiallyVisible(scene.panels[0]), "valid front panel is visible");

    expect(applyPanelLayout(scene, PanelLayoutPreset::tripleFlat),
        "triple-flat layout applies");
    expect(scene.panels[0].transform.yawDegrees == 0.0
        && scene.panels[2].transform.yawDegrees == 0.0,
        "triple-flat clears side yaw");
    scene.panelCount = 2U;
    expect(applyPanelLayout(scene, PanelLayoutPreset::dualFlat), "dual-flat applies");
    expect(scene.panels[0].transform.position.x == -scene.panels[1].transform.position.x,
        "dual-flat is symmetric");
    scene.panelCount = 1U;
    expect(applyPanelLayout(scene, PanelLayoutPreset::single), "single layout applies");
    expect(scene.panels[0].transform.position.x == 0.0,
        "single panel is centered");

    PanelSceneController controller(scene);
    expect(!controller.select(1U), "selection rejects inactive panel");
    expect(controller.select(0U), "selection accepts active panel");
    expect(controller.resizeSelected(0.1, 0.1), "selected panel resizes");
    expect(!controller.resizeSelected(-100.0, 0.0), "degenerate resize is rejected");
    const auto worldBeforeMove = scene.runtime[0].worldTransform;
    expect(controller.moveSelected({0.1, 0.0, -0.1}), "selected panel moves");
    expect(scene.runtime[0].worldTransform.values != worldBeforeMove.values,
        "transform change refreshes the cached world matrix");
    expect(!controller.moveSelected({0.0, 0.0, 100.0}), "panel behind camera is rejected");
    expect(controller.rotateSelected(10.0, 5.0), "selected panel rotates");
    expect(controller.resetSelected()
        && scene.panels[0].transform.yawDegrees == 0.0
        && scene.panels[0].dimensions.width == scene.defaultWidth,
        "selected-panel reset restores deterministic preset geometry");
    expect(!controller.toggleSelected(), "selected panel can be hidden");
    expect(!panelPotentiallyVisible(scene.panels[0]), "hidden panel is culled");
    expect(controller.toggleSelected(), "selected panel can be shown");
    scene.panels[0].content.kind = PanelContentKind::checkerboard;
    controller.reset();
    expect(scene.panelCount == 1U && scene.layout == PanelLayoutPreset::single,
        "controller reset restores preset");
    expect(scene.panels[0].content.kind == PanelContentKind::checkerboard,
        "layout reset preserves panel content configuration");

    scene.panelCount = 0U;
    expect(!validatePanelScene(scene).valid, "zero panels is rejected");
    scene.panelCount = maximumPanelCount + 1U;
    expect(!validatePanelScene(scene).valid, "more than three panels is rejected");
    scene = makeDefaultPanelScene(1U);
    scene.panels[0].dimensions.width = std::numeric_limits<double>::quiet_NaN();
    expect(!validatePanelScene(scene).valid, "NaN dimensions are rejected");
    scene = makeDefaultPanelScene(1U);
    scene.panels[0].dimensions.width = 0.01;
    expect(!validatePanelScene(scene).valid, "near-zero width is rejected");
    scene = makeDefaultPanelScene(1U);
    scene.panels[0].dimensions.height = 0.01;
    expect(!validatePanelScene(scene).valid, "near-zero height is rejected");
    scene = makeDefaultPanelScene(1U);
    scene.panels[0].transform.position.z = 1.0;
    expect(!panelPotentiallyVisible(scene.panels[0]), "panel behind camera is culled");
    scene = makeDefaultPanelScene(1U);
    scene.gap = -0.01;
    expect(!validatePanelScene(scene).valid, "negative global gap is rejected");
    scene = makeDefaultPanelScene(1U);
    scene.defaultDistance = std::numeric_limits<double>::infinity();
    expect(!validatePanelScene(scene).valid, "non-finite global distance is rejected");
    expect(parsePanelContentKind("checkerboard") == PanelContentKind::checkerboard,
        "checkerboard content parses");
    expect(!parsePanelContentKind("video").has_value(), "unknown content is rejected");
    expect(parsePanelLayoutPreset("triple-angled") == PanelLayoutPreset::tripleAngled,
        "triple-angled preset parses");
    expect(!parsePanelLayoutPreset("circle").has_value(), "unknown layout is rejected");
}

void testSourceRegistryAndRates()
{
    using namespace xreal::rendering;
    auto scene = makeDefaultPanelScene(3U);
    for (auto& panel : scene.panels)
    {
        panel.content.kind = PanelContentKind::desktop;
        panel.content.captureMonitorIndex = 0U;
        panel.targetFramesPerSecond = 60.0;
    }
    PanelContentRegistry registry;
    expect(registry.rebuild(scene).success, "source registry builds");
    expect(registry.sourceCount() == 1U, "identical desktop sources deduplicate");
    expect(registry.source(0U).consumerMask == 0x07U,
        "deduplicated source tracks all consumers");
    expect(registry.source(0U).requestedFramesPerSecond == 30.0,
        "selected panel drives shared source at highest policy rate");
    expect(scene.runtime[0].sourceSlot == scene.runtime[1].sourceSlot
        && scene.runtime[1].sourceSlot == scene.runtime[2].sourceSlot,
        "shared panels reference one stable source slot");

    scene.panels[1].content.transferPolicy = PanelTransferPolicy::cpuFallback;
    expect(registry.rebuild(scene).success && registry.sourceCount() == 2U,
        "different transfer policies are distinct desktop source identities");
    scene.panels[1].content.transferPolicy = PanelTransferPolicy::automatic;
    expect(registry.rebuild(scene).success && registry.sourceCount() == 1U,
        "restoring an identical policy deduplicates the source again");
    scene.panels[0].enabled = false;
    expect(registry.rebuild(scene).success && registry.sourceCount() == 1U
        && registry.source(0U).consumerMask == 0x06U,
        "removing one consumer preserves a source used by other panels");
    scene.panels[0].enabled = true;

    scene.panels[1].content.captureMonitorIndex = 1U;
    expect(registry.rebuild(scene).success && registry.sourceCount() == 2U,
        "different capture monitor creates another source");
    scene.panels[2].content.captureMonitorIndex = 2U;
    expect(registry.rebuild(scene).success && registry.sourceCount() == 3U,
        "three unique desktop configurations create three fixed source slots");
    scene.panels[2].content.captureMonitorIndex = 0U;
    scene.panels[2].content.scaling.cropMode =
        xreal::capture::DesktopCropMode::center16x9;
    expect(registry.rebuild(scene).success && registry.sourceCount() == 3U,
        "different crop configuration is a distinct source identity");
    scene.panels[2].content.scaling.cropMode =
        xreal::capture::DesktopCropMode::full;
    scene.panels[2].content.requestedWidth = 1920U;
    scene.panels[2].content.requestedHeight = 540U;
    expect(registry.rebuild(scene).success && registry.sourceCount() == 3U,
        "different target dimensions are a distinct source identity");
    scene.panels[2].content.requestedWidth = 0U;
    scene.panels[2].content.requestedHeight = 0U;
    scene.panels[2].content.kind = PanelContentKind::checkerboard;
    expect(registry.rebuild(scene).success && registry.sourceCount() == 3U,
        "mixed content uses fixed source slots");
    scene.panels[2].enabled = false;
    expect(registry.rebuild(scene).success && registry.sourceCount() == 2U,
        "hidden source with no consumers is paused");
    expect(defaultPanelSourceRate(true, true, 60.0) == 30.0,
        "selected source defaults to 30 Hz");
    expect(defaultPanelSourceRate(false, true, 60.0) == 20.0,
        "unselected visible source defaults to 20 Hz");
    expect(defaultPanelSourceRate(false, false, 60.0) == 0.0,
        "hidden source rate is zero");
    expect(defaultPanelSourceRate(true, true, 15.0) == 15.0,
        "lower explicit source rate is preserved");
    expect(defaultPanelSourceRate(PerformanceProfile::quality, false, true, 60.0) == 30.0,
        "quality profile keeps visible background sources at 30 Hz");
    expect(defaultPanelSourceRate(PerformanceProfile::performance, false, true, 60.0) == 15.0,
        "performance profile caps background sources at 15 Hz");
    expect(defaultPanelSourceRate(
        PerformanceProfile::performance, false, true, 48.0, true) == 48.0,
        "explicit panel rate overrides the selected performance profile");
    scene = makeDefaultPanelScene(3U);
    for (auto& panel : scene.panels)
    {
        panel.content.kind = PanelContentKind::desktop;
        panel.content.captureMonitorIndex = 0U;
        panel.targetFramesPerSecond = 60.0;
    }
    scene.panels[0].targetFramesPerSecond = 15.0;
    scene.panels[0].targetFramesPerSecondExplicit = true;
    scene.panels[1].targetFramesPerSecond = 24.0;
    scene.panels[1].targetFramesPerSecondExplicit = true;
    scene.panels[2].targetFramesPerSecond = 48.0;
    scene.panels[2].targetFramesPerSecondExplicit = true;
    expect(registry.rebuild(scene).success
            && registry.source(0U).requestedFramesPerSecond == 48.0,
        "shared source arbitration uses the maximum explicit consumer rate");
    expect(defaultPanelSourceRate(true, true,
        std::numeric_limits<double>::quiet_NaN()) == 0.0,
        "non-finite source rate is rejected");
}

void testPerformance()
{
    using namespace xreal::rendering;
    FrameTimeHistory history;
    for (int value = 1; value <= 100; ++value)
    {
        history.add(static_cast<double>(value));
    }
    const auto statistics = history.statistics();
    expect(statistics.sampleCount == 100U, "history records all initial samples");
    expect(std::abs(statistics.averageMilliseconds - 50.5) < 1.0e-12,
        "history mean is correct");
    expect(std::abs(statistics.p50Milliseconds - 50.5) < 1.0e-12,
        "history p50 is interpolated");
    expect(std::abs(statistics.p95Milliseconds - 95.05) < 1.0e-9,
        "history p95 is interpolated");
    expect(std::abs(statistics.p99Milliseconds - 99.01) < 1.0e-9,
        "history p99 is interpolated");
    expect(statistics.maximumMilliseconds == 100.0, "history maximum is correct");
    expect(statistics.overBudget33Milliseconds == 67U,
        "over-budget frame count is correct");
    history.add(-1.0);
    history.add(std::numeric_limits<double>::quiet_NaN());
    expect(history.size() == 100U, "invalid frame times are ignored");
    for (std::size_t index = 0; index < frameHistoryCapacity + 10U; ++index)
    {
        history.add(5.0);
    }
    expect(history.size() == frameHistoryCapacity, "history capacity is fixed");
    expect(history.statistics().maximumMilliseconds == 5.0,
        "ring history overwrites oldest samples");
    history.clear();
    expect(history.size() == 0U && history.statistics().sampleCount == 0U,
        "history clears deterministically");

    const auto bandwidth = estimateBgraBandwidth(1920U, 1080U, 30.0, 1U);
    expect(bandwidth.valid && bandwidth.bytesPerFrame == 8294400U,
        "BGRA byte estimate is exact");
    expect(bandwidth.mebibytesPerSecond > 237.0
        && bandwidth.mebibytesPerSecond < 238.0,
        "BGRA bandwidth estimate uses MiB/s");
    expect(!bandwidth.warning, "one 1080p30 source stays below warning threshold");
    expect(estimateBgraBandwidth(5120U, 1440U, 60.0, 3U).warning,
        "three 5K-wide sources trigger bandwidth warning");
    expect(!estimateBgraBandwidth(0U, 1080U, 30.0, 1U).valid,
        "zero width estimate is rejected");
    expect(!estimateBgraBandwidth(1920U, 1080U, -1.0, 1U).valid,
        "negative frame rate estimate is rejected");
    expect(!estimateBgraBandwidth(
        std::numeric_limits<std::uint32_t>::max(),
        std::numeric_limits<std::uint32_t>::max(), 60.0, 3U).valid,
        "BGRA bandwidth arithmetic rejects integer overflow");

    const auto onePanelDraws = makePanelDrawPlan(1U, true, false, false);
    expect(onePanelDraws.baseDrawCalls == 1U
        && onePanelDraws.overlayDrawCalls == 1U
        && onePanelDraws.totalDrawCalls == 2U,
        "one visible panel uses one base and one overlay draw");
    const auto threePanelDraws = makePanelDrawPlan(3U, true, false, false);
    expect(threePanelDraws.baseDrawCalls == 3U
        && threePanelDraws.overlayDrawCalls == 3U
        && threePanelDraws.presentCalls == 1U,
        "three visible panels stay within draw bounds and one Present");
    const auto culledPanelDraws = makePanelDrawPlan(2U, true, true, true);
    expect(culledPanelDraws.baseDrawCalls == 2U
        && culledPanelDraws.auxiliaryDrawCalls == 4U,
        "culled panels produce no base, overlay, grid, or axes draw");

    MultiPanelPerformanceCounters counters;
    counters.frameTimes.add(16.0);
    counters.presentTimes.add(1.0);
    counters.frames = 10U;
    counters.presents = 10U;
    counters.drawCalls = 30U;
    counters.baseDrawCalls = 15U;
    counters.overlayDrawCalls = 15U;
    const auto json = serializeMultiPanelPerformanceJson(
        counters, 1.0, 3U, 2U, "adapter\"name", "\\\\.\\DISPLAY5", "Release");
    expect(json.find("\"p99_ms\"") != std::string::npos,
        "performance JSON contains percentiles");
    expect(json.find("\"draw_calls\": 30") != std::string::npos,
        "performance JSON contains draw count");
    expect(json.find("\"base_draw_calls\": 15") != std::string::npos
        && json.find("\"overlay_draw_calls\": 15") != std::string::npos,
        "performance JSON separates base and overlay draws");
    expect(json.find("\"build\":\"Release\"") != std::string::npos,
        "performance JSON identifies build configuration");
    expect(json.find("\"average_readback_mib_s\"") != std::string::npos
            && json.find("\"average_upload_mib_s\"") != std::string::npos
            && json.find("\"maximum_observed_readback_mib_s\"")
                != std::string::npos,
        "performance JSON exposes aggregate readback and upload bandwidth");
    expect(json.find("adapter\\\"name") != std::string::npos
        && json.find("\\\\\\\\.\\\\DISPLAY5") != std::string::npos,
        "performance JSON escapes quotes and Win32 device paths");
    expect(JsonSyntaxParser(json).valid(),
        "performance telemetry is syntactically valid JSON");
}

void testPersistence()
{
    using namespace xreal::rendering;
    auto scene = makeDefaultPanelScene(3U);
    scene.performanceProfile = PerformanceProfile::performance;
    scene.selectedPanel = 2U;
    scene.panels[0].content.kind = PanelContentKind::desktop;
    scene.panels[0].content.captureMonitorIndex = 0U;
    scene.panels[0].content.transferPolicy = PanelTransferPolicy::cpuFallback;
    scene.panels[1].content.kind = PanelContentKind::checkerboard;
    scene.panels[2].content.kind = PanelContentKind::unavailable;
    scene.panels[2].transform.pitchDegrees = 4.5;
    scene.panels[2].displayName = "Diagnostics \"width\":999";
    scene.panels[2].fit = PanelFitMode::cover;
    scene.panels[2].filter = PanelFilterMode::point;
    scene.panels[2].overlay.enabled = false;
    scene.panels[2].targetFramesPerSecond = 48.0;
    scene.panels[2].targetFramesPerSecondExplicit = true;
    const auto json = serializePanelLayoutJson(scene);
    expect(JsonSyntaxParser(json).valid(), "serialized panel layout is valid JSON");
    const auto loaded = loadPanelLayoutJson(json);
    expect(loaded.scene.has_value(), "serialized layout loads");
    expect(loaded.scene.has_value() && loaded.scene->panelCount == 3U,
        "roundtrip preserves panel count");
    expect(loaded.scene.has_value() && loaded.scene->selectedPanel == 2U,
        "roundtrip preserves selection");
    expect(loaded.scene.has_value()
        && loaded.scene->performanceProfile == PerformanceProfile::performance,
        "roundtrip preserves performance profile");
    expect(loaded.scene.has_value()
        && loaded.scene->panels[0].content.captureMonitorIndex == 0U,
        "roundtrip preserves capture monitor");
    expect(loaded.scene.has_value()
        && loaded.scene->panels[0].content.transferPolicy
            == PanelTransferPolicy::cpuFallback,
        "roundtrip preserves the desktop transfer policy");
    expect(loaded.scene.has_value()
        && loaded.scene->panels[1].content.kind == PanelContentKind::checkerboard,
        "roundtrip preserves mixed content");
    expect(loaded.scene.has_value()
        && loaded.scene->panels[2].transform.pitchDegrees == 4.5,
        "roundtrip preserves transform");
    expect(loaded.scene.has_value()
        && loaded.scene->panels[2].displayName == "Diagnostics \"width\":999"
        && loaded.scene->panels[2].fit == PanelFitMode::cover
        && loaded.scene->panels[2].filter == PanelFilterMode::point
        && !loaded.scene->panels[2].overlay.enabled,
        "roundtrip preserves name, fit, filter, and overlay settings");
    expect(loaded.scene.has_value()
        && loaded.scene->panels[2].targetFramesPerSecondExplicit
        && loaded.scene->panels[2].targetFramesPerSecond == 48.0,
        "roundtrip preserves explicit source-rate override");
    expect(!loadPanelLayoutJson("{}").scene.has_value(),
        "missing schema is rejected");
    std::string wrongSchema = json;
    const auto version = wrongSchema.find("\"schema_version\": 1");
    wrongSchema.replace(version, std::string("\"schema_version\": 1").size(),
        "\"schema_version\": 99");
    expect(!loadPanelLayoutJson(wrongSchema).scene.has_value(),
        "unknown schema is rejected");
    std::string unstableId = json;
    const auto identifier = unstableId.find("\"id\":0");
    unstableId.replace(identifier, std::string("\"id\":0").size(), "\"id\":2");
    expect(!loadPanelLayoutJson(unstableId).scene.has_value(),
        "unstable panel identifier is rejected");
    std::string nonFinite = json;
    const auto width = nonFinite.find("\"width\":1.6");
    nonFinite.replace(width, std::string("\"width\":1.6").size(), "\"width\":nan");
    expect(!loadPanelLayoutJson(nonFinite).scene.has_value(),
        "non-finite JSON geometry is rejected");
    auto invalidScene = scene;
    invalidScene.panels[0].dimensions.width =
        std::numeric_limits<double>::quiet_NaN();
    expect(serializePanelLayoutJson(invalidScene).empty(),
        "layout serialization never emits NaN or infinity");

    const std::filesystem::path path{"multi-panel-layout-atomic-test.json"};
    std::error_code removeError;
    std::filesystem::remove(path, removeError);
    std::string saveError;
    expect(savePanelLayoutFileAtomic(path.string(), scene, false, saveError),
        "layout writes through the atomic save path");
    saveError.clear();
    expect(!savePanelLayoutFileAtomic(path.string(), scene, false, saveError),
        "layout save never silently overwrites an existing file");
    saveError.clear();
    expect(savePanelLayoutFileAtomic(path.string(), scene, true, saveError),
        "explicit overwrite replaces an existing layout");
    std::ifstream saved(path, std::ios::binary);
    const std::string savedJson{
        std::istreambuf_iterator<char>(saved), std::istreambuf_iterator<char>()};
    expect(loadPanelLayoutJson(savedJson).scene.has_value(),
        "atomically saved layout is parseable");
    saved.close();
    std::filesystem::remove(path, removeError);
    saveError.clear();
    expect(!savePanelLayoutFileAtomic(
        path.string(), invalidScene, false, saveError)
        && !std::filesystem::exists(path),
        "invalid layout is rejected before a file is created");
}

} // namespace

int main()
{
    testSceneAndLayouts();
    testSourceRegistryAndRates();
    testPerformance();
    testPersistence();
    if (failures != 0)
    {
        std::cerr << failures << " of " << checks << " multi-panel checks failed.\n";
        return 1;
    }
    std::cout << "All " << checks << " multi-panel checks passed.\n";
    return 0;
}
