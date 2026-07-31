#include "rendering/PanelPerformance.hpp"
#include "rendering/PanelScene.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

namespace xreal::rendering
{
namespace
{

[[nodiscard]] std::string escapeJson(std::string_view value)
{
    std::string result;
    result.reserve(value.size());
    for (const unsigned char character : value)
    {
        switch (character)
        {
        case '"': result += "\\\""; break;
        case '\\': result += "\\\\"; break;
        case '\b': result += "\\b"; break;
        case '\f': result += "\\f"; break;
        case '\n': result += "\\n"; break;
        case '\r': result += "\\r"; break;
        case '\t': result += "\\t"; break;
        default:
            if (character < 0x20U)
            {
                constexpr char hexadecimal[] = "0123456789ABCDEF";
                result += "\\u00";
                result.push_back(hexadecimal[(character >> 4U) & 0x0FU]);
                result.push_back(hexadecimal[character & 0x0FU]);
            }
            else
            {
                result.push_back(static_cast<char>(character));
            }
            break;
        }
    }
    return result;
}

[[nodiscard]] double percentile(
    const std::array<double, frameHistoryCapacity>& sorted,
    std::size_t count,
    double fraction) noexcept
{
    if (count == 0U)
    {
        return 0.0;
    }
    const double position = fraction * static_cast<double>(count - 1U);
    const auto lower = static_cast<std::size_t>(std::floor(position));
    const auto upper = static_cast<std::size_t>(std::ceil(position));
    const double weight = position - static_cast<double>(lower);
    return sorted[lower] + (sorted[upper] - sorted[lower]) * weight;
}

void writeTiming(std::ostringstream& output, const FramePercentiles& value)
{
    output << "{\"samples\":" << value.sampleCount
           << ",\"average_ms\":" << value.averageMilliseconds
           << ",\"p50_ms\":" << value.p50Milliseconds
           << ",\"p95_ms\":" << value.p95Milliseconds
           << ",\"p99_ms\":" << value.p99Milliseconds
           << ",\"max_ms\":" << value.maximumMilliseconds
           << ",\"over_33_ms\":" << value.overBudget33Milliseconds << '}';
}

} // namespace

void FrameTimeHistory::add(double milliseconds) noexcept
{
    if (!std::isfinite(milliseconds) || milliseconds < 0.0)
    {
        return;
    }
    values_[next_] = milliseconds;
    next_ = (next_ + 1U) % values_.size();
    size_ = std::min(size_ + 1U, values_.size());
}

void FrameTimeHistory::clear() noexcept
{
    values_ = {};
    next_ = 0U;
    size_ = 0U;
}

std::size_t FrameTimeHistory::size() const noexcept
{
    return size_;
}

FramePercentiles FrameTimeHistory::statistics() const noexcept
{
    FramePercentiles result;
    result.sampleCount = size_;
    if (size_ == 0U)
    {
        return result;
    }
    std::array<double, frameHistoryCapacity> sorted{};
    double sum{};
    for (std::size_t index = 0; index < size_; ++index)
    {
        const std::size_t source = size_ == values_.size()
            ? (next_ + index) % values_.size() : index;
        sorted[index] = values_[source];
        sum += sorted[index];
        if (sorted[index] > 33.0)
        {
            ++result.overBudget33Milliseconds;
        }
    }
    std::sort(sorted.begin(), sorted.begin() + static_cast<std::ptrdiff_t>(size_));
    result.averageMilliseconds = sum / static_cast<double>(size_);
    result.p50Milliseconds = percentile(sorted, size_, 0.50);
    result.p95Milliseconds = percentile(sorted, size_, 0.95);
    result.p99Milliseconds = percentile(sorted, size_, 0.99);
    result.maximumMilliseconds = sorted[size_ - 1U];
    return result;
}

BandwidthEstimate estimateBgraBandwidth(
    std::uint32_t width,
    std::uint32_t height,
    double framesPerSecond,
    std::size_t sourceCount) noexcept
{
    if (width == 0U || height == 0U || sourceCount == 0U
        || !std::isfinite(framesPerSecond) || framesPerSecond < 0.0)
    {
        return {};
    }
    constexpr std::uint64_t bytesPerPixel = 4U;
    const std::uint64_t maximum = std::numeric_limits<std::uint64_t>::max();
    if (static_cast<std::uint64_t>(width) > maximum / height / bytesPerPixel)
    {
        return {};
    }
    const std::uint64_t bytesPerFrame = static_cast<std::uint64_t>(width)
        * static_cast<std::uint64_t>(height) * bytesPerPixel;
    const long double total = static_cast<long double>(bytesPerFrame)
        * static_cast<long double>(framesPerSecond)
        * static_cast<long double>(sourceCount);
    if (!std::isfinite(static_cast<double>(total)))
    {
        return {};
    }
    const double mebibytes = static_cast<double>(total / (1024.0L * 1024.0L));
    return {true, bytesPerFrame, mebibytes, mebibytes > 750.0};
}

PanelDrawPlan makePanelDrawPlan(
    std::size_t visiblePanels,
    bool overlayEnabled,
    bool backgroundGrid,
    bool worldAxes) noexcept
{
    PanelDrawPlan result;
    result.visiblePanels = std::min(visiblePanels, maximumPanelCount);
    result.baseDrawCalls = result.visiblePanels;
    result.overlayDrawCalls = overlayEnabled ? result.visiblePanels : 0U;
    const std::size_t auxiliaryPasses = static_cast<std::size_t>(backgroundGrid)
        + static_cast<std::size_t>(worldAxes);
    result.auxiliaryDrawCalls = overlayEnabled
        ? result.visiblePanels * auxiliaryPasses : 0U;
    result.totalDrawCalls = result.baseDrawCalls + result.overlayDrawCalls
        + result.auxiliaryDrawCalls;
    result.presentCalls = 1U;
    return result;
}

std::string serializeMultiPanelPerformanceJson(
    const MultiPanelPerformanceCounters& counters,
    double elapsedSeconds,
    std::size_t panelCount,
    std::size_t sourceCount,
    std::string_view adapter,
    std::string_view outputName,
    std::string_view buildConfiguration)
{
    std::ostringstream output;
    output << std::setprecision(12)
           << "{\n  \"schema_version\": 1,\n"
           << "  \"configuration\": {\"panels\":" << panelCount
           << ",\"sources\":" << sourceCount
           << ",\"adapter\":\"" << escapeJson(adapter) << "\",\"output\":\""
           << escapeJson(outputName) << "\",\"build\":\""
           << escapeJson(buildConfiguration) << "\"},\n"
           << "  \"elapsed_seconds\": " << elapsedSeconds << ",\n"
           << "  \"frames\": " << counters.frames << ",\n"
           << "  \"presents\": " << counters.presents << ",\n"
           << "  \"frame_timing\": ";
    writeTiming(output, counters.frameTimes.statistics());
    output << ",\n  \"present_timing\": ";
    writeTiming(output, counters.presentTimes.statistics());
    output << ",\n  \"cpu_update_timing\": ";
    writeTiming(output, counters.cpuUpdateTimes.statistics());
    output << ",\n  \"draw_submission_timing\": ";
    writeTiming(output, counters.drawSubmissionTimes.statistics());
    output << ",\n  \"draw_calls\": " << counters.drawCalls
           << ",\n  \"base_draw_calls\": " << counters.baseDrawCalls
           << ",\n  \"overlay_draw_calls\": " << counters.overlayDrawCalls
           << ",\n  \"auxiliary_draw_calls\": " << counters.auxiliaryDrawCalls
           << ",\n  \"state_set_calls\": " << counters.stateSetCalls
           << ",\n  \"shader_resource_bind_calls\": "
           << counters.shaderResourceBindCalls
           << ",\n  \"sampler_bind_calls\": " << counters.samplerBindCalls
           << ",\n  \"constant_buffer_updates\": " << counters.constantBufferUpdates
           << ",\n  \"instance_buffer_updates\": " << counters.instanceBufferUpdates
           << ",\n  \"texture_uploads\": " << counters.textureUploads
           << ",\n  \"capture_frames\": " << counters.captureFrames
           << ",\n  \"repeated_frames\": " << counters.repeatedFrames
           << ",\n  \"dropped_frames\": " << counters.droppedFrames
           << ",\n  \"readback_bytes\": " << counters.readbackBytes
           << ",\n  \"cpu_transfer_bytes\": " << counters.cpuTransferBytes
           << ",\n  \"upload_bytes\": " << counters.uploadBytes
           << ",\n  \"average_readback_mib_s\": "
           << (elapsedSeconds > 0.0
                   ? static_cast<double>(counters.readbackBytes)
                       / (1024.0 * 1024.0 * elapsedSeconds) : 0.0)
           << ",\n  \"average_cpu_transfer_mib_s\": "
           << (elapsedSeconds > 0.0
                   ? static_cast<double>(counters.cpuTransferBytes)
                       / (1024.0 * 1024.0 * elapsedSeconds) : 0.0)
           << ",\n  \"average_upload_mib_s\": "
           << (elapsedSeconds > 0.0
                   ? static_cast<double>(counters.uploadBytes)
                       / (1024.0 * 1024.0 * elapsedSeconds) : 0.0)
           << ",\n  \"maximum_observed_readback_mib_s\": "
           << counters.maximumObservedReadbackMebibytesPerSecond
           << ",\n  \"maximum_observed_cpu_transfer_mib_s\": "
           << counters.maximumObservedCpuTransferMebibytesPerSecond
           << ",\n  \"maximum_observed_upload_mib_s\": "
           << counters.maximumObservedUploadMebibytesPerSecond
           << ",\n  \"resources_created_at_startup\": "
           << counters.resourcesCreatedAtStartup
           << ",\n  \"resources_created_steady_state\": "
           << counters.resourcesCreatedSteadyState
           << ",\n  \"flush_calls\": " << counters.flushCalls
           << ",\n  \"working_set_bytes\": " << counters.workingSetBytes
           << ",\n  \"private_bytes\": " << counters.privateBytes
           << ",\n  \"initial_working_set_bytes\": "
           << counters.initialWorkingSetBytes
           << ",\n  \"peak_working_set_bytes\": " << counters.peakWorkingSetBytes
           << ",\n  \"initial_private_bytes\": " << counters.initialPrivateBytes
           << ",\n  \"peak_private_bytes\": " << counters.peakPrivateBytes << "\n}\n";
    return output.str();
}

} // namespace xreal::rendering
