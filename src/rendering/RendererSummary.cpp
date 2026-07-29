#include "rendering/RendererSummary.hpp"

#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string_view>

namespace xreal::rendering
{
namespace
{

[[nodiscard]] std::string escapeJson(std::string_view value)
{
    std::string result;
    result.reserve(value.size());
    for (char character : value)
    {
        switch (character)
        {
        case '\\': result += "\\\\"; break;
        case '"': result += "\\\""; break;
        case '\n': result += "\\n"; break;
        case '\r': result += "\\r"; break;
        case '\t': result += "\\t"; break;
        default: result += character; break;
        }
    }
    return result;
}

void numberOrNull(std::ostringstream& output, double value)
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

[[nodiscard]] std::string sourceText(RenderOrientationSource source)
{
    return source == RenderOrientationSource::predicted ? "predicted" : "measured";
}

[[nodiscard]] std::string frameText(RenderOrientationFrame frame)
{
    return frame == RenderOrientationFrame::relative ? "relative" : "absolute";
}

} // namespace

std::string serializeRendererSummaryJson(const RendererSummary& summary)
{
    const auto& timing = summary.timing;
    std::ostringstream output;
    output << std::setprecision(std::numeric_limits<double>::max_digits10)
           << "{\n  \"schema_version\":1,\n"
           << "  \"type\":\"orientation-driven-d3d11-render\",\n"
           << "  \"experimental\":true,\n"
           << "  \"motion_to_photon_latency\":\"not_measured\",\n"
           << "  \"adapter\":{\"name\":\"" << escapeJson(summary.graphics.adapterName)
           << "\",\"feature_level\":\"" << escapeJson(summary.graphics.featureLevel)
           << "\",\"device_type\":\"" << (summary.graphics.warp ? "WARP" : "hardware")
           << "\",\"debug_layer\":" << (summary.graphics.debugLayer ? "true" : "false")
           << ",\"luid\":";
    if (summary.graphics.adapterLuid.has_value())
    {
        output << '"' << platform::windows::dxgiAdapterLuidText(*summary.graphics.adapterLuid)
               << '"';
    }
    else
    {
        output << "null";
    }
    output << ",\"selected_monitor_match\":"
           << (summary.graphics.selectedAdapterMatched ? "true" : "false")
           << ",\"output\":\"" << escapeJson(summary.graphics.outputDeviceName) << '"'
           << "},\n  \"swap_chain\":{\"format\":\""
           << escapeJson(summary.graphics.swapChainFormat)
           << "\",\"effect\":\"flip-discard\",\"maximum_frame_latency\":1,"
           << "\"waitable_object\":"
           << (summary.graphics.frameLatencyWaitableObject ? "true" : "false")
           << ",\"vsync\":" << (summary.options.vsync ? "true" : "false") << "},\n"
           << "  \"window\":{\"width\":" << summary.options.windowWidth
           << ",\"height\":" << summary.options.windowHeight
           << ",\"monitor_index\":" << summary.monitor.index
           << ",\"monitor_device\":\"" << escapeJson(summary.monitor.deviceName)
           << "\",\"monitor_bounds\":[" << summary.monitor.left << ',' << summary.monitor.top
           << ',' << summary.monitor.right << ',' << summary.monitor.bottom << ']'
           << ",\"dxgi_output_matched\":"
           << (summary.monitor.dxgiOutput.has_value() ? "true" : "false") << "},\n"
           << "  \"orientation\":{\"source\":\"" << sourceText(summary.options.orientationSource)
           << "\",\"frame\":\"" << frameText(summary.options.orientationFrame)
           << "\",\"demo_mode\":" << (summary.options.orientationDemoMode ? "true" : "false")
           << ",\"calibration_accepted\":"
           << (summary.calibrationAccepted ? "true" : "false")
           << ",\"recenter_generation\":" << summary.recenterGeneration << "},\n"
           << "  \"fusion\":{\"mode\":\"complementary\",\"startup\":\""
           << (summary.options.fusionStartupGravity ? "gravity" : "identity")
           << "\",\"experimental\":true},\n"
           << "  \"prediction\":{\"enabled\":"
           << ((summary.options.predictOrientation || summary.options.orientationDemoMode) ? "true" : "false")
           << ",\"horizon_ms\":";
    numberOrNull(output, summary.options.predictionHorizonMilliseconds);
    output << ",\"experimental\":true},\n"
           << "  \"render_mapping\":{\"sensor_x\":\""
           << renderAxisText(summary.mapping.axes.sensorX) << "\",\"sensor_y\":\""
           << renderAxisText(summary.mapping.axes.sensorY) << "\",\"sensor_z\":\""
           << renderAxisText(summary.mapping.axes.sensorZ)
           << "\",\"handedness\":\"right-handed\",\"experimental\":"
           << (summary.mapping.experimental ? "true" : "false")
           << ",\"verified\":" << (summary.mapping.verified ? "true" : "false") << "},\n"
           << "  \"frame_statistics\":{\"render_frames\":" << timing.renderFrameCount
           << ",\"presents\":" << timing.presentCount << ",\"elapsed_seconds\":";
    numberOrNull(output, timing.elapsedSeconds);
    output << ",\"average_fps\":"; numberOrNull(output, timing.averageFramesPerSecond);
    output << ",\"minimum_fps\":"; numberOrNull(output, timing.minimumFramesPerSecond);
    output << ",\"maximum_fps\":"; numberOrNull(output, timing.maximumFramesPerSecond);
    output << ",\"average_frame_time_ms\":"; numberOrNull(output, timing.averageFrameTimeMilliseconds);
    output << ",\"maximum_frame_time_ms\":"; numberOrNull(output, timing.maximumFrameTimeMilliseconds);
    output << ",\"slow_frames\":" << timing.slowFrameCount << "},\n"
           << "  \"orientation_snapshot_statistics\":{\"repeated\":"
           << timing.repeatedOrientationSnapshots << ",\"invalid\":"
           << timing.invalidOrientationSnapshots << ",\"prediction_fallbacks\":"
           << timing.predictionFallbacks << ",\"sensor_publish_rate\":";
    numberOrNull(output, timing.sensorPublishRate);
    output << ",\"average_snapshot_age_ms\":"; numberOrNull(output, timing.averageSnapshotAgeMilliseconds);
    output << ",\"maximum_snapshot_age_ms\":"; numberOrNull(output, timing.maximumSnapshotAgeMilliseconds);
    output << ",\"average_approximate_effective_lead_ms\":";
    numberOrNull(output, timing.averageApproximateEffectiveLeadMilliseconds);
    output << ",\"latest_device_timestamp\":" << timing.latestDeviceTimestamp << "},\n"
           << "  \"device_counters\":{\"received\":" << summary.imu.received
           << ",\"dropped\":" << summary.imu.dropped << ",\"invalid\":"
           << summary.imu.invalid << ",\"out_of_sequence\":" << summary.imu.outOfSequence
           << "},\n  \"shutdown\":{\"state\":\""
           << rendererStartupStateText(summary.finalState) << "\",\"reason\":\""
           << escapeJson(summary.shutdownReason) << "\",\"error\":";
    if (summary.error.empty())
    {
        output << "null";
    }
    else
    {
        output << '"' << escapeJson(summary.error) << '"';
    }
    output << "}\n}\n";
    return output.str();
}

} // namespace xreal::rendering
