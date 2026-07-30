#include "capture/DesktopCaptureDiagnostics.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace xreal::capture
{
namespace
{

[[nodiscard]] std::string escapeJson(std::string_view text)
{
    std::string result;
    result.reserve(text.size());
    for (const char character : text)
    {
        if (character == '\\' || character == '"')
        {
            result.push_back('\\');
        }
        result.push_back(character);
    }
    return result;
}

void finiteNumber(std::ostringstream& output, double value)
{
    if (std::isfinite(value))
    {
        output << value;
    }
    else
    {
        output << "null";
    }
}

} // namespace

void DesktopCaptureDiagnostics::recordRendered(
    const DesktopCaptureFrame& frame,
    std::chrono::steady_clock::time_point now) noexcept
{
    if (frame.sequence == previousSequence_)
    {
        ++values_.repeatedCaptureFrames;
    }
    previousSequence_ = frame.sequence;
    ++values_.renderedCaptureFrames;
    const double age = std::max(0.0,
        std::chrono::duration<double, std::milli>(now - frame.captureHostTimestamp).count());
    totalAgeMilliseconds_ += age;
    values_.averageSourceFrameAgeMilliseconds = totalAgeMilliseconds_
        / static_cast<double>(values_.renderedCaptureFrames);
    values_.maximumSourceFrameAgeMilliseconds = std::max(
        values_.maximumSourceFrameAgeMilliseconds, age);
}

DesktopRenderCaptureStatistics DesktopCaptureDiagnostics::snapshot() const noexcept
{
    return values_;
}

std::string serializeDesktopCaptureSummaryJson(const DesktopCaptureSummary& summary)
{
    const auto adapter = [](const platform::windows::MonitorInformation& monitor) {
        return monitor.dxgiOutput.has_value() ? monitor.dxgiOutput->adapterDescription : "unavailable";
    };
    const auto luid = [](const platform::windows::MonitorInformation& monitor) {
        return monitor.dxgiOutput.has_value()
            ? platform::windows::dxgiAdapterLuidText(monitor.dxgiOutput->adapterLuid)
            : "unavailable";
    };
    std::ostringstream output;
    output << std::setprecision(12)
           << "{\n  \"schema_version\":1,\n"
           << "  \"type\":\"xreal_desktop_duplication_diagnostic\",\n"
           << "  \"experimental\":true,\n"
           << "  \"render_monitor\":\"" << escapeJson(summary.renderMonitor.deviceName) << "\",\n"
           << "  \"capture_monitor\":\"" << escapeJson(summary.captureMonitor.deviceName) << "\",\n"
           << "  \"render_adapter\":\"" << escapeJson(adapter(summary.renderMonitor)) << "\",\n"
           << "  \"capture_adapter\":\"" << escapeJson(adapter(summary.captureMonitor)) << "\",\n"
           << "  \"render_adapter_luid\":\"" << luid(summary.renderMonitor) << "\",\n"
           << "  \"capture_adapter_luid\":\"" << luid(summary.captureMonitor) << "\",\n"
           << "  \"same_adapter\":" << (summary.sameAdapter ? "true" : "false") << ",\n"
           << "  \"transfer_mode\":\"" << escapeJson(summary.capture.transferMode) << "\",\n"
           << "  \"shared_handle_supported\":" << (summary.sharedHandleSupported ? "true" : "false") << ",\n"
           << "  \"cpu_fallback_enabled\":" << (summary.options.allowCpuFallback ? "true" : "false") << ",\n"
           << "  \"cpu_fallback_used\":" << (summary.cpuFallbackUsed ? "true" : "false") << ",\n"
           << "  \"duplication_state\":\"" << desktopCaptureStatusText(summary.capture.state) << "\",\n"
           << "  \"source_width\":" << summary.capture.sourceWidth << ",\n"
           << "  \"source_height\":" << summary.capture.sourceHeight << ",\n"
           << "  \"source_format\":" << summary.capture.sourceFormat << ",\n"
           << "  \"source_rotation_degrees\":" << static_cast<unsigned int>(summary.capture.sourceRotation) << ",\n"
           << "  \"acquired_frames\":" << summary.capture.acquiredFrames << ",\n"
           << "  \"frames_acquired\":" << summary.capture.acquiredFrames << ",\n"
           << "  \"staging_copies\":" << summary.capture.stagingCopies << ",\n"
           << "  \"staging_maps\":" << summary.capture.stagingMaps << ",\n"
           << "  \"staging_map_successes\":" << summary.capture.stagingMapSuccesses << ",\n"
           << "  \"cpu_buffers_created\":" << summary.capture.cpuBuffersCreated << ",\n"
           << "  \"cpu_frames_published\":" << summary.capture.cpuFramesPublished << ",\n"
           << "  \"latest_published_sequence\":" << summary.capture.latestPublishedSequence << ",\n"
           << "  \"wait_timeouts\":" << summary.capture.waitTimeouts << ",\n"
           << "  \"access_loss_count\":" << summary.capture.accessLossEvents << ",\n"
           << "  \"recreation_attempts\":" << summary.capture.recreationAttempts << ",\n"
           << "  \"capture_fps\":";
    finiteNumber(output, summary.capture.capturedFramesPerSecond);
    output << ",\n  \"average_source_frame_age_ms\":";
    finiteNumber(output, summary.render.averageSourceFrameAgeMilliseconds);
    output << ",\n  \"maximum_source_frame_age_ms\":";
    finiteNumber(output, summary.render.maximumSourceFrameAgeMilliseconds);
    output << ",\n  \"repeated_frame_count\":" << summary.render.repeatedCaptureFrames
           << ",\n  \"dropped_publication_count\":" << summary.bridge.droppedPublications
           << ",\n  \"gpu_copy_count\":" << summary.capture.gpuCopies
           << ",\n  \"cpu_fallback_copy_count\":" << summary.capture.cpuFallbackCopies
           << ",\n  \"cpu_fallback_bytes\":" << summary.capture.cpuFallbackBytes
           << ",\n  \"dirty_rect_count\":" << summary.capture.dirtyRectCount
           << ",\n  \"move_rect_count\":" << summary.capture.moveRectCount
           << ",\n  \"cursor_metadata_count\":" << summary.capture.pointerMetadataCount
           << ",\n  \"cpu_frames_seen\":" << summary.stages.cpuFramesSeen
           << ",\n  \"cpu_frames_consumed\":" << summary.stages.cpuFramesConsumed
           << ",\n  \"cpu_frames_skipped_same_sequence\":" << summary.stages.cpuFramesSkippedSameSequence
           << ",\n  \"latest_consumed_sequence\":" << summary.stages.latestConsumedSequence
           << ",\n  \"upload_texture_creations\":" << summary.stages.uploadTextureCreations
           << ",\n  \"upload_texture_recreations\":" << summary.stages.uploadTextureRecreations
           << ",\n  \"update_subresource_calls\":" << summary.stages.updateSubresourceCalls
           << ",\n  \"update_subresource_failures\":" << summary.stages.updateSubresourceFailures
           << ",\n  \"latest_uploaded_sequence\":" << summary.stages.latestUploadedSequence
           << ",\n  \"latest_upload_width\":" << summary.stages.latestUploadWidth
           << ",\n  \"latest_upload_height\":" << summary.stages.latestUploadHeight
           << ",\n  \"latest_upload_format\":" << summary.stages.latestUploadFormat
           << ",\n  \"upload_texture_valid\":" << (summary.stages.uploadTextureValid ? "true" : "false")
           << ",\n  \"desktop_srv_creations\":" << summary.stages.desktopSrvCreations
           << ",\n  \"desktop_srv_failures\":" << summary.stages.desktopSrvFailures
           << ",\n  \"desktop_srv_bind_count\":" << summary.stages.desktopSrvBindCount
           << ",\n  \"latest_bound_sequence\":" << summary.stages.latestBoundSequence
           << ",\n  \"desktop_srv_valid\":" << (summary.stages.desktopSrvValid ? "true" : "false")
           << ",\n  \"panel_content_requested\":\"" << panelContentText(summary.stages.panelContentRequested)
           << "\",\n  \"panel_content_effective\":\""
           << desktopPanelEffectiveModeText(summary.stages.panelContentEffective)
           << "\",\n  \"desktop_texture_available\":"
           << (summary.stages.desktopTextureAvailable ? "true" : "false")
           << ",\n  \"rendered_desktop_frames\":" << summary.stages.renderedDesktopFrames
           << ",\n  \"rendered_unavailable_frames\":" << summary.stages.renderedUnavailableFrames
           << ",\n  \"rendered_synthetic_frames\":" << summary.stages.renderedSyntheticFrames
           << ",\n  \"cursor_rendering\":\"metadata_only_not_rendered\",\n"
           << "  \"hdr_color_accuracy\":\"not_supported\",\n"
           << "  \"motion_to_photon_latency_measured\":false,\n"
           << "  \"last_error\":\"" << escapeJson(
                summary.finalError.empty() ? summary.capture.lastError : summary.finalError)
           << "\"\n}\n";
    return output.str();
}

} // namespace xreal::capture
