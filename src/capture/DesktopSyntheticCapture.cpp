#include "capture/DesktopSyntheticCapture.hpp"

#include "capture/DesktopCaptureScaler.hpp"

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

#include <d3d11.h>
#include <wrl/client.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <mutex>
#include <sstream>
#include <stop_token>
#include <thread>
#include <utility>

namespace xreal::capture
{
namespace
{

using Microsoft::WRL::ComPtr;

[[nodiscard]] std::string hresultText(HRESULT value)
{
    std::ostringstream output;
    output << "HRESULT 0x" << std::hex << std::uppercase
           << static_cast<unsigned long>(value);
    return output.str();
}

} // namespace

class DesktopSyntheticCapture::Implementation
{
public:
    explicit Implementation(DesktopCaptureBridge& bridge) : bridge_(bridge) {}

    [[nodiscard]] bool start(DesktopSyntheticCaptureConfig config)
    {
        if (worker_.joinable() || config.sourceWidth == 0U || config.sourceHeight == 0U
            || !std::isfinite(config.maximumFramesPerSecond)
            || config.maximumFramesPerSecond <= 0.0)
        {
            std::scoped_lock lock(mutex_);
            error_ = "Synthetic capture requires positive dimensions and frame rate.";
            return false;
        }
        config_ = std::move(config);
        sourceName_ = "SYNTH" + std::to_string(config_.sourceOrdinal + 1U);
        maximumFramesPerSecond_.store(
            config_.maximumFramesPerSecond, std::memory_order_release);
        {
            std::scoped_lock lock(mutex_);
            tracker_ = {};
            tracker_.values.state = DesktopCaptureStatus::initializing;
            tracker_.values.transferMode = "synthetic_gpu_scale_readback";
            error_.clear();
        }
        worker_ = std::jthread([this](std::stop_token stopToken) { run(stopToken); });
        return true;
    }

    void setMaximumFramesPerSecond(double value) noexcept
    {
        if (std::isfinite(value) && value >= 0.0)
        {
            maximumFramesPerSecond_.store(value, std::memory_order_release);
            cadenceCondition_.notify_all();
        }
    }

    void stop()
    {
        if (worker_.joinable())
        {
            worker_.request_stop();
            cadenceCondition_.notify_all();
            worker_.join();
        }
    }

    [[nodiscard]] DesktopCaptureStatistics statistics() const
    {
        std::scoped_lock lock(mutex_);
        return tracker_.snapshot();
    }

    [[nodiscard]] DesktopCaptureStatistics detailedStatistics() const
    {
        std::scoped_lock lock(mutex_);
        return tracker_.detailedSnapshot();
    }

    [[nodiscard]] std::string error() const
    {
        std::scoped_lock lock(mutex_);
        return error_;
    }

private:
    [[nodiscard]] bool initialize()
    {
        constexpr D3D_FEATURE_LEVEL requestedLevels[]{
            D3D_FEATURE_LEVEL_11_1, D3D_FEATURE_LEVEL_11_0};
        D3D_FEATURE_LEVEL selectedLevel{};
        HRESULT result = D3D11CreateDevice(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr,
            D3D11_CREATE_DEVICE_BGRA_SUPPORT, requestedLevels,
            static_cast<UINT>(std::size(requestedLevels)), D3D11_SDK_VERSION,
            &device_, &selectedLevel, &context_);
        if (result == E_INVALIDARG)
        {
            result = D3D11CreateDevice(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr,
                D3D11_CREATE_DEVICE_BGRA_SUPPORT, requestedLevels + 1, 1U,
                D3D11_SDK_VERSION, &device_, &selectedLevel, &context_);
        }
        if (FAILED(result))
        {
            setError("Creating synthetic benchmark D3D11 device failed: "
                + hresultText(result));
            return false;
        }
        D3D11_TEXTURE2D_DESC description{};
        description.Width = config_.sourceWidth;
        description.Height = config_.sourceHeight;
        description.MipLevels = 1U;
        description.ArraySize = 1U;
        description.Format = DXGI_FORMAT_B8G8R8A8_UNORM;
        description.SampleDesc.Count = 1U;
        description.Usage = D3D11_USAGE_DEFAULT;
        description.BindFlags = D3D11_BIND_RENDER_TARGET
            | D3D11_BIND_SHADER_RESOURCE;
        result = device_->CreateTexture2D(&description, nullptr, &sourceTexture_);
        if (FAILED(result))
        {
            setError("Creating synthetic benchmark source texture failed: "
                + hresultText(result));
            return false;
        }
        ComPtr<ID3D11RenderTargetView> view;
        result = device_->CreateRenderTargetView(sourceTexture_.Get(), nullptr, &view);
        if (FAILED(result))
        {
            setError("Creating synthetic benchmark source RTV failed: "
                + hresultText(result));
            return false;
        }
        const float ordinal = static_cast<float>((config_.sourceOrdinal % 3U) + 1U);
        const float color[4]{0.08F * ordinal, 0.14F * ordinal,
            0.22F * ordinal, 1.0F};
        context_->ClearRenderTargetView(view.Get(), color);
        if (!scaler_.initialize(device_.Get(), context_.Get()))
        {
            setError(scaler_.error());
            return false;
        }
        plan_ = resolveDesktopScalePlan(
            config_.sourceWidth, config_.sourceHeight, config_.scaling);
        if (!plan_.plan.has_value())
        {
            setError("Synthetic benchmark scale planning failed: " + plan_.error);
            return false;
        }
        {
            std::scoped_lock lock(mutex_);
            tracker_.values.state = DesktopCaptureStatus::active;
            tracker_.values.availability = DesktopFrameAvailability::activeUnchanged;
            tracker_.values.sourceWidth = config_.sourceWidth;
            tracker_.values.sourceHeight = config_.sourceHeight;
            tracker_.values.sourceFormat =
                static_cast<std::uint32_t>(DXGI_FORMAT_B8G8R8A8_UNORM);
            tracker_.values.cropX = plan_.plan->crop.x;
            tracker_.values.cropY = plan_.plan->crop.y;
            tracker_.values.cropWidth = plan_.plan->crop.width;
            tracker_.values.cropHeight = plan_.plan->crop.height;
            tracker_.values.transferWidth = plan_.plan->targetWidth;
            tracker_.values.transferHeight = plan_.plan->targetHeight;
        }
        return true;
    }

    void run(std::stop_token stopToken)
    {
        if (!initialize())
        {
            return;
        }
        auto nextCapture = std::chrono::steady_clock::now();
        while (!stopToken.stop_requested())
        {
            const double cadence = maximumFramesPerSecond_.load(
                std::memory_order_acquire);
            if (cadence <= 0.0)
            {
                std::unique_lock lock(cadenceMutex_);
                (void)cadenceCondition_.wait(lock, stopToken, [this] {
                    return maximumFramesPerSecond_.load(std::memory_order_acquire) > 0.0;
                });
                nextCapture = std::chrono::steady_clock::now();
                continue;
            }
            const auto attempt = std::chrono::steady_clock::now();
            {
                std::scoped_lock lock(mutex_);
                ++tracker_.values.captureAttempts;
            }
            const auto readback = scaler_.process(
                sourceTexture_.Get(), *plan_.plan, config_.scaling.filter);
            if (!readback.success)
            {
                setError("Synthetic benchmark scale/readback failed: " + readback.error);
                break;
            }
            if (readback.frameReady)
            {
                DesktopCaptureFrame frame;
                frame.valid = true;
                frame.status = DesktopCaptureStatus::active;
                frame.sourceWidth = readback.width;
                frame.sourceHeight = readback.height;
                frame.originalSourceWidth = config_.sourceWidth;
                frame.originalSourceHeight = config_.sourceHeight;
                frame.cropX = plan_.plan->crop.x;
                frame.cropY = plan_.plan->crop.y;
                frame.cropWidth = plan_.plan->crop.width;
                frame.cropHeight = plan_.plan->crop.height;
                frame.sourceFormat = readback.format;
                frame.cpuPixels = readback.pixels;
                frame.cpuRowPitch = readback.rowPitch;
                frame.captureHostTimestamp = std::chrono::steady_clock::now();
                frame.sourceMonitorDeviceName = sourceName_;
                const std::uint64_t sequence = bridge_.publish(std::move(frame));
                const auto scalerStatistics = scaler_.statistics();
                std::scoped_lock lock(mutex_);
                tracker_.recordAcquired(std::chrono::steady_clock::now());
                tracker_.recordAcquireDuration(std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - attempt).count());
                tracker_.recordScaleSubmission(readback.gpuSubmissionMilliseconds);
                tracker_.recordMapWait(readback.mapWaitMilliseconds);
                tracker_.recordCpuRepack(readback.cpuRepackMilliseconds);
                tracker_.values.stagingCopies = scalerStatistics.stagingCopies;
                tracker_.values.stagingMaps = scalerStatistics.stagingMaps;
                tracker_.values.stagingMapSuccesses = scalerStatistics.stagingMaps;
                tracker_.values.cpuBuffersCreated = desktopStagingRingCapacity;
                tracker_.values.cpuFramesPublished += static_cast<std::uint64_t>(
                    sequence != 0U);
                tracker_.values.latestPublishedSequence = sequence;
                tracker_.values.cpuFallbackCopies += static_cast<std::uint64_t>(
                    sequence != 0U);
                tracker_.values.cpuFallbackBytes += readback.mappedBytes;
                tracker_.values.originalSourceBytes += plan_.plan->sourceBytesPerFrame;
                tracker_.values.cropBytes += plan_.plan->cropBytesPerFrame;
                tracker_.values.scaledTargetBytes += plan_.plan->targetBytesPerFrame;
                tracker_.values.stagingBytesMapped += readback.mappedBytes;
                tracker_.values.cpuBytesCopied += readback.mappedBytes;
                tracker_.values.gpuScaleDraws = scalerStatistics.gpuScaleDraws;
                tracker_.values.scalerResourceRecreations =
                    scalerStatistics.resourceRecreations;
                tracker_.values.stagingRingContentions = scalerStatistics.ringContentions;
            }
            nextCapture += std::chrono::duration_cast<
                std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(1.0 / cadence));
            std::unique_lock lock(cadenceMutex_);
            (void)cadenceCondition_.wait_until(lock, stopToken, nextCapture,
                [this, cadence] {
                    return maximumFramesPerSecond_.load(std::memory_order_acquire)
                        != cadence;
                });
        }
        std::scoped_lock lock(mutex_);
        if (tracker_.values.state != DesktopCaptureStatus::fatalError)
        {
            tracker_.values.state = DesktopCaptureStatus::shuttingDown;
        }
    }

    void setError(std::string value)
    {
        std::scoped_lock lock(mutex_);
        error_ = std::move(value);
        tracker_.values.lastError = error_;
        tracker_.values.state = DesktopCaptureStatus::fatalError;
        ++tracker_.values.captureErrors;
    }

    DesktopCaptureBridge& bridge_;
    DesktopSyntheticCaptureConfig config_;
    mutable std::mutex mutex_;
    std::mutex cadenceMutex_;
    std::condition_variable_any cadenceCondition_;
    DesktopCaptureStatisticsTracker tracker_;
    DesktopScalePlanResult plan_;
    DesktopCaptureScaler scaler_;
    ComPtr<ID3D11Device> device_;
    ComPtr<ID3D11DeviceContext> context_;
    ComPtr<ID3D11Texture2D> sourceTexture_;
    std::atomic<double> maximumFramesPerSecond_{30.0};
    std::jthread worker_;
    std::string sourceName_;
    std::string error_;
};

DesktopSyntheticCapture::DesktopSyntheticCapture(DesktopCaptureBridge& bridge)
    : implementation_(std::make_unique<Implementation>(bridge))
{
}

DesktopSyntheticCapture::~DesktopSyntheticCapture() { stop(); }

bool DesktopSyntheticCapture::start(DesktopSyntheticCaptureConfig config)
{
    return implementation_->start(std::move(config));
}

void DesktopSyntheticCapture::setMaximumFramesPerSecond(double value) noexcept
{
    implementation_->setMaximumFramesPerSecond(value);
}

void DesktopSyntheticCapture::stop() { implementation_->stop(); }

DesktopCaptureStatistics DesktopSyntheticCapture::statistics() const
{
    return implementation_->statistics();
}

DesktopCaptureStatistics DesktopSyntheticCapture::detailedStatistics() const
{
    return implementation_->detailedStatistics();
}

std::string DesktopSyntheticCapture::error() const
{
    return implementation_->error();
}

} // namespace xreal::capture
