#include "capture/DesktopCaptureBridge.hpp"
#include "capture/DesktopCaptureDiagnostics.hpp"
#include "capture/DesktopCpuFrame.hpp"
#include "capture/DesktopCaptureOptions.hpp"
#include "capture/DesktopCaptureScaler.hpp"
#include "capture/DesktopCaptureScaling.hpp"
#include "capture/DesktopFrameRetention.hpp"
#include "capture/DesktopTextureLayout.hpp"
#include "capture/DesktopRenderStages.hpp"
#include "JsonSyntaxParser.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <thread>
#include <vector>

namespace
{

int failures{};

void expect(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << "FAILED: " << message << '\n';
        ++failures;
    }
}

[[nodiscard]] bool close(double first, double second)
{
    return std::abs(first - second) < 1.0e-9;
}

std::array<xreal::platform::windows::MonitorInformation, 3> monitors()
{
    using namespace xreal::platform::windows;
    return {
        MonitorInformation{0U, R"(\\.\DISPLAY1)", 0, 0, 2560, 1440,
            0, 0, 2560, 1400, true,
            DxgiOutputInformation{0U, 0U, R"(\\.\DISPLAY1)", 0, 0, 2560, 1440,
                true, "NVIDIA", {1U, 0}}},
        MonitorInformation{1U, R"(\\.\DISPLAY5)", 2560, 0, 4480, 1080,
            2560, 0, 4480, 1040, false,
            DxgiOutputInformation{1U, 0U, R"(\\.\DISPLAY5)", 2560, 0, 4480, 1080,
                true, "Intel", {2U, 0}}},
        MonitorInformation{2U, R"(\\.\DISPLAY7)", 0, 0, 1, 1,
            0, 0, 1, 1, false, std::nullopt},
    };
}

void testSelection()
{
    using namespace xreal::capture;
    const auto topology = monitors();
    MonitorSelector selector;
    selector.index = 0U;
    selector.deviceName = R"(\\.\DISPLAY5)";
    const auto named = resolveMonitor(topology, selector, "Capture");
    expect(named.monitor != nullptr && named.monitor->index == 1U,
        "device-name selection overrides index");
    selector.deviceName = R"(\\.\MISSING)";
    expect(resolveMonitor(topology, selector, "Capture").monitor == nullptr,
        "missing named monitor never falls back to another index");
    selector.deviceName.reset();
    selector.index = 2U;
    expect(resolveMonitor(topology, selector, "Capture").monitor == nullptr,
        "monitor without DXGI output is rejected");
    expect(!sameAdapter(topology[0], topology[1]), "different LUIDs are cross-adapter");
    auto same = topology[1];
    same.dxgiOutput->adapterLuid = topology[0].dxgiOutput->adapterLuid;
    expect(sameAdapter(topology[0], same), "matching LUIDs are same-adapter");
}

void testLayout()
{
    using namespace xreal::capture;
    const auto contain = calculateDesktopTextureLayout(
        1920U, 1080U, 4.0 / 3.0, DesktopFit::contain, DesktopRotation::identity, false);
    expect(close(contain.contentMinimum[1], 0.125)
            && close(contain.contentMaximum[1], 0.875),
        "contain adds vertical letterboxing without distortion");
    const auto portraitContain = calculateDesktopTextureLayout(
        1080U, 1920U, 16.0 / 9.0, DesktopFit::contain, DesktopRotation::identity, false);
    expect(portraitContain.contentMinimum[0] > 0.0,
        "contain adds horizontal pillarboxing for portrait sources");
    const auto cover = calculateDesktopTextureLayout(
        1920U, 1080U, 4.0 / 3.0, DesktopFit::cover, DesktopRotation::identity, false);
    expect(cover.cropWidth < 1.0 && close(cover.cropHeight, 1.0),
        "cover crops horizontally and preserves source aspect");
    const auto stretch = calculateDesktopTextureLayout(
        1920U, 1080U, 4.0 / 3.0, DesktopFit::stretch, DesktopRotation::identity, false);
    expect(close(stretch.contentMinimum[0], 0.0) && close(stretch.contentMaximum[1], 1.0)
            && close(stretch.cropWidth, 1.0),
        "stretch explicitly fills the panel");

    const std::array rotations{
        DesktopRotation::identity, DesktopRotation::rotate90,
        DesktopRotation::rotate180, DesktopRotation::rotate270};
    for (const auto rotation : rotations)
    {
        const auto layout = calculateDesktopTextureLayout(
            1920U, 1080U, 16.0 / 9.0, DesktopFit::stretch, rotation, false);
        const std::array source{0.2, 0.7};
        const auto panel = mapSourceToPanel(layout, source);
        const auto roundTrip = mapPanelToSource(layout, panel);
        expect(close(source[0], roundTrip[0]) && close(source[1], roundTrip[1]),
            "rotation transform round-trips cursor coordinates");
    }
    const auto normal = calculateDesktopTextureLayout(
        1U, 1U, 1.0, DesktopFit::stretch, DesktopRotation::identity, false);
    const auto flipped = calculateDesktopTextureLayout(
        1U, 1U, 1.0, DesktopFit::stretch, DesktopRotation::identity, true);
    expect(close(mapPanelToSource(normal, {0.25, 0.2})[1], 0.2)
            && close(mapPanelToSource(flipped, {0.25, 0.2})[1], 0.8),
        "Y flip exists only when explicitly requested");
}

xreal::capture::DesktopCaptureFrame frame(std::uint64_t generation = 0U)
{
    xreal::capture::DesktopCaptureFrame value;
    value.valid = true;
    value.sourceWidth = 100U;
    value.sourceHeight = 50U;
    value.recoveryGeneration = generation;
    value.captureHostTimestamp = std::chrono::steady_clock::now();
    return value;
}

void testBridge()
{
    using namespace xreal::capture;
    DesktopCaptureBridge bridge;
    expect(!bridge.publish({}), "invalid frames are not published");
    expect(bridge.publish(frame(3U)) && bridge.publish(frame(4U)),
        "latest frame replaces the stale pending frame");
    auto latest = bridge.latest();
    expect(latest.has_value() && latest->sequence == 2U
            && latest->recoveryGeneration == 4U,
        "sequence and recovery generation survive publication");
    const auto nonBlockingLatest = bridge.tryLatest();
    expect(nonBlockingLatest.has_value() && nonBlockingLatest->sequence == 2U,
        "non-blocking latest-frame access preserves the bounded snapshot");
    (void)bridge.latest();
    const auto statistics = bridge.statistics();
    expect(statistics.publications == 2U && statistics.droppedPublications == 1U
            && statistics.repeatedReads == 2U && statistics.contendedReads == 0U,
        "bounded bridge reports drops and repeats deterministically");

    DesktopCaptureBridge concurrent;
    std::atomic<bool> finished{};
    std::thread producer([&] {
        for (int index = 0; index < 2000; ++index)
        {
            (void)concurrent.publish(frame(static_cast<std::uint64_t>(index)));
        }
        finished = true;
    });
    std::uint64_t previous{};
    while (!finished || concurrent.statistics().publications < 2000U)
    {
        if (const auto value = concurrent.latest(); value.has_value())
        {
            expect(value->sequence >= previous, "concurrent snapshots remain coherent");
            previous = value->sequence;
        }
    }
    producer.join();
    expect(concurrent.statistics().publications == 2000U,
        "stress test keeps a bounded coherent latest frame");
}

void testJson()
{
    using namespace xreal::capture;
    const auto topology = monitors();
    DesktopCaptureSummary summary;
    summary.renderMonitor = topology[1];
    summary.captureMonitor = topology[0];
    summary.capture.state = DesktopCaptureStatus::active;
    summary.capture.transferMode = "shared_nt_handle_cross_adapter";
    summary.capture.capturedFramesPerSecond = 60.0;
    summary.sameAdapter = false;
    summary.sharedHandleSupported = true;
    const auto json = serializeDesktopCaptureSummaryJson(summary);
    expect(JsonSyntaxParser(json).valid(), "desktop capture JSON parses");
    expect(json.find("motion_to_photon_latency_measured\":false") != std::string::npos,
        "JSON does not claim motion-to-photon measurement");
    expect(json.find("00000000:00000001") != std::string::npos
            && json.find("00000000:00000002") != std::string::npos,
        "JSON preserves both adapter LUIDs");
    constexpr std::array stageFields{
        "frames_acquired", "staging_copies", "staging_maps",
        "staging_map_successes", "cpu_buffers_created", "cpu_frames_published",
        "latest_published_sequence", "cpu_frames_seen", "cpu_frames_consumed",
        "cpu_frames_skipped_same_sequence", "latest_consumed_sequence",
        "upload_texture_creations", "upload_texture_recreations",
        "update_subresource_calls", "update_subresource_failures",
        "latest_uploaded_sequence", "latest_upload_width", "latest_upload_height",
        "latest_upload_format", "upload_texture_valid", "desktop_srv_creations",
        "desktop_srv_failures", "desktop_srv_bind_count", "latest_bound_sequence",
        "desktop_srv_valid", "panel_content_requested", "panel_content_effective",
        "desktop_texture_available", "rendered_desktop_frames",
        "rendered_unavailable_frames", "rendered_synthetic_frames"};
    for (const char* field : stageFields)
    {
        expect(json.find(field) != std::string::npos,
            "capture JSON exposes every frame-path stage counter");
    }
}

void testExitStateRegression()
{
    using namespace xreal::capture;
    expect(desktopCaptureLoopExitStatus(DesktopCaptureStatus::fatalError, false)
            == DesktopCaptureStatus::fatalError,
        "capture-loop exit preserves a fatal error for diagnostics");
    expect(desktopCaptureLoopExitStatus(DesktopCaptureStatus::active, true)
            == DesktopCaptureStatus::shuttingDown,
        "an explicit stop transitions capture to shutting down");
    expect(desktopCaptureLoopExitStatus(DesktopCaptureStatus::waitTimeout, false)
            == DesktopCaptureStatus::waitTimeout,
        "a non-fatal wait timeout is not converted into a false shutdown");
}

void testCpuLayoutAndOwnership()
{
    using namespace xreal::capture;
    const auto layout = calculatePackedBgraLayout(5120U, 1440U);
    expect(layout.has_value() && layout->stride == 20480U
            && layout->bufferSize == static_cast<std::size_t>(20480U) * 1440U,
        "5120-pixel BGRA rows use an overflow-checked packed stride");
    expect(!calculatePackedBgraLayout(0xFFFFFFFFU, 2U).has_value(),
        "packed BGRA stride overflow is rejected");

    constexpr std::uint32_t width = 3U;
    constexpr std::uint32_t height = 2U;
    constexpr std::uint32_t paddedPitch = 16U;
    std::array<std::byte, paddedPitch * height> padded{};
    for (std::size_t index = 0; index < padded.size(); ++index)
    {
        padded[index] = static_cast<std::byte>(index);
    }
    std::array<std::byte, width * 4U * height> packed{};
    expect(repackBgraRows(padded.data(), padded.size(), paddedPitch,
            packed, width, height),
        "padded GPU rows repack into tightly packed CPU rows");
    expect(packed[11] == padded[11] && packed[12] == padded[16]
            && packed[23] == padded[27],
        "repacking drops source RowPitch padding without mixing strides");

    DesktopCaptureBridge bridge;
    auto checkerboard = makeDesktopCheckerboardFrame(128U, 64U);
    expect(checkerboard.has_value() && checkerboard->valid
            && checkerboard->cpuRowPitch == 512U,
        "checkerboard is a valid packed CPU desktop frame");
    const auto originalPixels = checkerboard->cpuPixels;
    const auto sequence = bridge.publish(std::move(*checkerboard));
    auto published = bridge.latest();
    expect(sequence == 1U && published.has_value() && published->valid
            && published->sequence == sequence && published->cpuPixels == originalPixels
            && published->cpuPixels->size() == 128U * 64U * 4U,
        "publication preserves validity, sequence, pixel ownership, dimensions and stride");

    const std::filesystem::path bmpPath{"desktop-capture-test-frame.bmp"};
    const auto bmp = writeDesktopFrameBmp(*published, bmpPath.string());
    std::ifstream bmpInput(bmpPath, std::ios::binary);
    std::array<char, 2> signature{};
    bmpInput.read(signature.data(), signature.size());
    expect(bmp.success && signature[0] == 'B' && signature[1] == 'M'
            && std::filesystem::file_size(bmpPath) == 54U + 128U * 64U * 4U,
        "top-down BGRA BMP dump preserves packed rows and emits a valid header");
    bmpInput.close();
    std::filesystem::remove(bmpPath);
}

void testDiagnosticCheckerboard()
{
    using namespace xreal::capture;
    const auto checkerboard = makeDesktopCheckerboardFrame(640U, 360U);
    expect(checkerboard.has_value() && checkerboard->valid
            && checkerboard->sequence != 0U
            && checkerboard->sourceWidth == 640U && checkerboard->sourceHeight == 360U
            && checkerboard->cpuRowPitch == 2560U
            && checkerboard->sourceFormat == 87U,
        "diagnostic checkerboard publishes a complete packed BGRA frame contract");
    if (!checkerboard.has_value())
    {
        return;
    }
    const auto summary = analyzeDesktopFrameContent(*checkerboard);
    expect(summary.has_value() && !summary->allZero && summary->allOpaque
            && summary->distinctColorCount == 5U,
        "diagnostic checkerboard is opaque, non-zero and contains five distinct colors");
    if (!summary.has_value())
    {
        return;
    }
    expect(summary->firstPixel == BgraPixel{0x00U, 0x00U, 0xFFU, 0xFFU},
        "checkerboard top-left marker is red BGRA");
    expect(summary->centerPixel == BgraPixel{0xFFU, 0xFFU, 0xFFU, 0xFFU},
        "checkerboard center marker is white BGRA");
    expect(summary->lastPixel == BgraPixel{0x00U, 0xFFU, 0x00U, 0xFFU},
        "checkerboard last pixel is green BGRA");
    const auto pixel = [&checkerboard](std::uint32_t x, std::uint32_t y) {
        const std::size_t offset = static_cast<std::size_t>(y)
            * checkerboard->cpuRowPitch + static_cast<std::size_t>(x) * 4U;
        return BgraPixel{
            static_cast<std::uint8_t>((*checkerboard->cpuPixels)[offset + 0U]),
            static_cast<std::uint8_t>((*checkerboard->cpuPixels)[offset + 1U]),
            static_cast<std::uint8_t>((*checkerboard->cpuPixels)[offset + 2U]),
            static_cast<std::uint8_t>((*checkerboard->cpuPixels)[offset + 3U]),
        };
    };
    expect(pixel(639U, 0U) == BgraPixel{0xFFU, 0x00U, 0x00U, 0xFFU},
        "checkerboard top-right marker is blue BGRA");
    expect(summary->checksum == 0xF104DB9F3DA2C325ULL,
        "checkerboard checksum remains stable across builds");

    constexpr std::uint32_t paddedPitch = 2576U;
    auto paddedPixels = std::make_shared<std::vector<std::byte>>(
        static_cast<std::size_t>(paddedPitch) * 360U, std::byte{0xA5U});
    for (std::uint32_t row = 0U; row < 360U; ++row)
    {
        std::copy_n(checkerboard->cpuPixels->data()
                + static_cast<std::size_t>(row) * checkerboard->cpuRowPitch,
            checkerboard->cpuRowPitch,
            paddedPixels->data() + static_cast<std::size_t>(row) * paddedPitch);
    }
    auto padded = *checkerboard;
    padded.cpuPixels = std::move(paddedPixels);
    padded.cpuRowPitch = paddedPitch;
    const auto readbackSummary = analyzeDesktopFrameContent(padded);
    expect(readbackSummary.has_value()
            && readbackSummary->checksum == summary->checksum,
        "padded GPU readback rows repack to the exact CPU-source checksum");
    const std::filesystem::path bmpPath{"desktop-padded-readback-test.bmp"};
    const auto bmp = writeDesktopFrameBmp(padded, bmpPath.string());
    expect(bmp.success && std::filesystem::file_size(bmpPath)
            == 54U + static_cast<std::uintmax_t>(640U * 360U * 4U),
        "render-target BMP writer omits padded RowPitch bytes");
    std::filesystem::remove(bmpPath);
}

void testUploadDecisionsAndPanelMode()
{
    using namespace xreal::capture;
    DesktopUploadState state;
    expect(decideDesktopUpload(state, 1U, 100U, 50U, 87U, true)
            == DesktopUploadAction::create,
        "first CPU frame creates an upload texture");
    state = {1U, 100U, 50U, 87U, true};
    expect(decideDesktopUpload(state, 1U, 100U, 50U, 87U, true)
            == DesktopUploadAction::skipSameSequence,
        "identical sequence is not re-uploaded");
    expect(decideDesktopUpload(state, 2U, 100U, 50U, 87U, true)
            == DesktopUploadAction::update,
        "new sequence updates the existing texture");
    expect(decideDesktopUpload(state, 2U, 101U, 50U, 87U, true)
            == DesktopUploadAction::recreate,
        "width change recreates the upload texture");
    expect(decideDesktopUpload(state, 2U, 100U, 51U, 87U, true)
            == DesktopUploadAction::recreate,
        "height change recreates the upload texture");
    expect(decideDesktopUpload(state, 2U, 100U, 50U, 91U, true)
            == DesktopUploadAction::recreate,
        "format change recreates the upload texture");
    expect(effectiveDesktopPanelMode(PanelContent::desktop, true, false)
            == DesktopPanelEffectiveMode::desktop,
        "desktop requested with valid SRV selects the desktop shader path");
    expect(effectiveDesktopPanelMode(PanelContent::desktop, true, true)
            == DesktopPanelEffectiveMode::desktop,
        "stale diagnostic state retains the last valid desktop SRV");
    expect(effectiveDesktopPanelMode(PanelContent::desktop, false, false)
            == DesktopPanelEffectiveMode::unavailable,
        "desktop requested without SRV selects unavailable, never synthetic");
    expect(effectiveDesktopPanelMode(PanelContent::synthetic, true, false)
            == DesktopPanelEffectiveMode::synthetic,
        "synthetic request remains synthetic");

    const auto contain = calculateDesktopTextureLayout(5120U, 1440U, 16.0 / 9.0,
        DesktopFit::contain, DesktopRotation::identity, false);
    expect(contain.contentMaximum[0] > contain.contentMinimum[0]
            && contain.contentMaximum[1] > contain.contentMinimum[1]
            && contain.cropWidth > 0.0 && contain.cropHeight > 0.0,
        "5120x1440 contain transform into 16:9 remains finite and non-zero");
}

void testGpuScalePlanning()
{
    using namespace xreal::capture;
    DesktopScaleRequest request;
    const auto native = resolveDesktopScalePlan(5120U, 1440U, request);
    expect(native.plan.has_value() && native.plan->targetWidth == 5120U
            && native.plan->targetHeight == 1440U && !native.plan->gpuScaleRequired,
        "native 5120x1440 planning preserves the source");
    expect(native.plan.has_value()
            && native.plan->sourceBytesPerFrame == 29'491'200U,
        "native ultrawide byte accounting is exact");

    request.scale = 0.5;
    const auto half = resolveDesktopScalePlan(5120U, 1440U, request);
    expect(half.plan.has_value() && half.plan->targetWidth == 2560U
            && half.plan->targetHeight == 720U && half.plan->gpuScaleRequired,
        "scale 0.5 resolves to 2560x720 GPU scaling");

    request = {};
    request.targetWidth = 1920U;
    const auto widthOnly = resolveDesktopScalePlan(5120U, 1440U, request);
    expect(widthOnly.plan.has_value() && widthOnly.plan->targetHeight == 540U,
        "one-dimensional target preserves source aspect ratio");

    request = {};
    request.cropMode = DesktopCropMode::center16x9;
    const auto centerCrop = resolveDesktopScalePlan(5120U, 1440U, request);
    expect(centerCrop.plan.has_value()
            && centerCrop.plan->crop == DesktopCaptureRegion{1280U, 0U, 2560U, 1440U},
        "center-16x9 crop is a symmetric 2560x1440 region");

    request.targetWidth = 1920U;
    request.targetHeight = 1080U;
    const auto centerScaled = resolveDesktopScalePlan(5120U, 1440U, request);
    expect(centerScaled.plan.has_value()
            && centerScaled.plan->targetWidth == 1920U
            && centerScaled.plan->targetHeight == 1080U
            && centerScaled.plan->targetBytesPerFrame == 8'294'400U,
        "center crop scales to exact 1920x1080 readback bytes");

    request = {};
    request.targetWidth = 1920U;
    request.targetHeight = 540U;
    const auto fullUltrawide = resolveDesktopScalePlan(5120U, 1440U, request);
    expect(fullUltrawide.plan.has_value()
            && fullUltrawide.plan->targetWidth == 1920U
            && fullUltrawide.plan->targetHeight == 540U
            && fullUltrawide.plan->targetBytesPerFrame == 4'147'200U,
        "full ultrawide downscale preserves 32:9 without distortion");

    request.targetHeight = 1080U;
    request.fit = DesktopFit::cover;
    const auto covered = resolveDesktopScalePlan(5120U, 1440U, request);
    expect(covered.plan.has_value()
            && covered.plan->crop == DesktopCaptureRegion{1280U, 0U, 2560U, 1440U}
            && covered.plan->targetWidth == 1920U
            && covered.plan->targetHeight == 1080U,
        "cover applies one correctly offset centered crop before exact scaling");

    request = {};
    request.cropMode = DesktopCropMode::custom;
    request.customRegion = {5000U, 0U, 200U, 100U};
    expect(!resolveDesktopScalePlan(5120U, 1440U, request).plan.has_value(),
        "crop outside the source is rejected");
    expect(!resolveDesktopScalePlan(0U, 1440U, {}).plan.has_value(),
        "zero source dimension is rejected");

    request = {};
    request.targetWidth = 6000U;
    expect(!resolveDesktopScalePlan(5120U, 1440U, request).plan.has_value(),
        "upscale is rejected by default");
    request.allowUpscale = true;
    expect(resolveDesktopScalePlan(5120U, 1440U, request).plan.has_value(),
        "upscale is accepted only when explicitly enabled");

    request = {};
    request.resolutionPolicy = DesktopResolutionPolicy::panelAware;
    request.projectedWidth = 900U;
    request.projectedHeight = 500U;
    const auto panelAware = resolveDesktopScalePlan(5120U, 1440U, request);
    expect(panelAware.plan.has_value() && panelAware.plan->targetWidth <= 1125U
            && panelAware.plan->targetHeight <= 625U,
        "panel-aware planning honors projected coverage and safety factor");
    if (panelAware.plan.has_value() && half.plan.has_value())
    {
        const auto now = std::chrono::steady_clock::now();
        expect(!shouldTransitionDesktopScalePlan(*half.plan, *panelAware.plan,
                now, now - std::chrono::milliseconds(100)),
            "resolution transition rate limit rejects rapid changes");
        expect(shouldTransitionDesktopScalePlan(*half.plan, *panelAware.plan,
                now, now - std::chrono::seconds(2)),
            "large panel-aware change passes hysteresis after the rate limit");
        expect(!shouldTransitionDesktopScalePlan(*half.plan, *half.plan,
                now, now - std::chrono::seconds(2)),
            "static coverage never requests resource recreation");
    }
    expect(desktopStagingRingCapacity == 3U,
        "capture staging ring has fixed capacity three");
}

void testRetainLastValidFrame()
{
    using namespace xreal::capture;
    DesktopFrameRetention retention;
    const auto start = std::chrono::steady_clock::now();
    expect(retention.availability() == DesktopFrameAvailability::noFrameEver
            && !retention.hasValidFrame(),
        "no-frame-ever starts unavailable without a retained texture");
    retention.recordWaitTimeout(start, std::chrono::milliseconds(500));
    expect(retention.availability() == DesktopFrameAvailability::noFrameEver,
        "WAIT_TIMEOUT before the first frame remains no-frame-ever");
    retention.recordValidFrame(start);
    retention.recordWaitTimeout(start + std::chrono::milliseconds(100),
        std::chrono::milliseconds(500));
    expect(retention.availability() == DesktopFrameAvailability::activeUnchanged
            && retention.hasValidFrame(),
        "WAIT_TIMEOUT after a valid frame retains active unchanged content");
    retention.recordWaitTimeout(start + std::chrono::seconds(2),
        std::chrono::milliseconds(500));
    expect(retention.availability() == DesktopFrameAvailability::staleButValid
            && retention.hasValidFrame(),
        "repeated WAIT_TIMEOUT becomes stale-but-valid without clearing content");
    retention.recordUnavailable();
    expect(retention.availability() == DesktopFrameAvailability::staleButValid,
        "unavailable notification does not discard a retained valid frame");
    retention.recordAccessLost();
    expect(retention.availability() == DesktopFrameAvailability::accessLost
            && retention.hasValidFrame(),
        "access loss is distinct while retained content remains owned");
    retention.recordValidFrame(start + std::chrono::seconds(3));
    expect(retention.availability() == DesktopFrameAvailability::activeUnchanged,
        "a recovered valid frame resumes the same retention state machine");
}

void testCaptureTimingHistory()
{
    using namespace xreal::capture;
    DesktopCaptureTimingHistory history;
    for (int value = 1; value <= 100; ++value)
    {
        history.add(static_cast<double>(value));
    }
    const auto statistics = history.statistics();
    expect(statistics.sampleCount == 100U
            && close(statistics.p50Milliseconds, 50.5)
            && close(statistics.p95Milliseconds, 95.05)
            && close(statistics.p99Milliseconds, 99.01),
        "capture timing history calculates p50/p95/p99 deterministically");
    history.add(-1.0);
    history.add(std::numeric_limits<double>::quiet_NaN());
    expect(history.size() == 100U, "capture timing history rejects invalid samples");
    for (std::size_t index = 0U;
         index < desktopCaptureTimingHistoryCapacity + 10U; ++index)
    {
        history.add(5.0);
    }
    expect(history.size() == desktopCaptureTimingHistoryCapacity
            && close(history.statistics().p99Milliseconds, 5.0),
        "capture timing history is a fixed-capacity overwrite ring");
}

} // namespace

int main()
{
    testSelection();
    testLayout();
    testBridge();
    testJson();
    testExitStateRegression();
    testCpuLayoutAndOwnership();
    testDiagnosticCheckerboard();
    testUploadDecisionsAndPanelMode();
    testGpuScalePlanning();
    testRetainLastValidFrame();
    testCaptureTimingHistory();
    if (failures != 0)
    {
        std::cerr << failures << " desktop capture test(s) failed.\n";
        return 1;
    }
    std::cout << "All desktop capture tests passed.\n";
    return 0;
}
