#include "capture/DesktopDuplicationCapture.hpp"
#include "capture/DesktopCaptureScaler.hpp"
#include "capture/DesktopCpuFrame.hpp"
#include "capture/DesktopFrameRetention.hpp"

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

#include <d3d11_1.h>
#include <dxgi1_3.h>
#include <wrl/client.h>

#include <algorithm>
#include <atomic>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <sstream>
#include <iostream>
#include <utility>
#include <vector>

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

[[nodiscard]] DesktopRotation rotationFromDxgi(DXGI_MODE_ROTATION rotation) noexcept
{
    switch (rotation)
    {
    case DXGI_MODE_ROTATION_ROTATE90: return DesktopRotation::rotate90;
    case DXGI_MODE_ROTATION_ROTATE180: return DesktopRotation::rotate180;
    case DXGI_MODE_ROTATION_ROTATE270: return DesktopRotation::rotate270;
    default: return DesktopRotation::identity;
    }
}

[[nodiscard]] bool supportedFormat(DXGI_FORMAT format) noexcept
{
    return format == DXGI_FORMAT_B8G8R8A8_UNORM
        || format == DXGI_FORMAT_B8G8R8A8_UNORM_SRGB;
}

} // namespace

class DesktopDuplicationCapture::Implementation
{
public:
    explicit Implementation(DesktopCaptureBridge& bridge) : bridge_(bridge) {}

    ~Implementation() { stop(); }

    [[nodiscard]] bool start(DesktopDuplicationConfig config)
    {
        if (worker_.joinable())
        {
            setError("Desktop capture is already running.");
            return false;
        }
        if (!config.captureMonitor.dxgiOutput.has_value())
        {
            setError("The capture monitor has no matching DXGI output.");
            return false;
        }
        if (!std::isfinite(config.maximumFramesPerSecond)
            || config.maximumFramesPerSecond < 0.0)
        {
            setError("Desktop capture maximum frame rate must be finite and non-negative.");
            return false;
        }
        const bool crossAdapter = config.captureMonitor.dxgiOutput->adapterLuid
            != config.renderAdapterLuid;
        if (crossAdapter && config.options.crossAdapterPolicy == CrossAdapterPolicy::reject)
        {
            setError("Cross-adapter desktop capture was rejected by policy.");
            return false;
        }
        if (config.options.crossAdapterPolicy == CrossAdapterPolicy::cpuFallback
            && !config.options.allowCpuFallback)
        {
            setError("CPU desktop-capture fallback requires explicit permission.");
            return false;
        }
        config_ = std::move(config);
        maximumFramesPerSecond_.store(config_.maximumFramesPerSecond,
            std::memory_order_release);
        cpuFallback_ = config_.options.crossAdapterPolicy == CrossAdapterPolicy::cpuFallback;
        {
            std::scoped_lock lock(mutex_);
            tracker_.values = {};
            tracker_.values.state = DesktopCaptureStatus::initializing;
            tracker_.values.transferMode = cpuFallback_ ? "cpu_staging_explicit"
                : crossAdapter ? "shared_nt_handle_cross_adapter"
                               : "shared_nt_handle_same_adapter";
            error_.clear();
        }
        worker_ = std::jthread([this](std::stop_token stopToken) { run(stopToken); });
        return true;
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

    void setMaximumFramesPerSecond(double value) noexcept
    {
        if (!std::isfinite(value) || value < 0.0)
        {
            return;
        }
        maximumFramesPerSecond_.store(value, std::memory_order_release);
        cadenceCondition_.notify_all();
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
    void setState(DesktopCaptureStatus state)
    {
        std::scoped_lock lock(mutex_);
        tracker_.values.state = state;
    }

    void setError(std::string message)
    {
        std::scoped_lock lock(mutex_);
        error_ = message;
        tracker_.values.lastError = message;
        tracker_.values.state = DesktopCaptureStatus::fatalError;
        ++tracker_.values.captureErrors;
    }

    void resetDuplication()
    {
        duplication_.Reset();
        output_.Reset();
        scaler_.reset();
        device_.Reset();
        context_.Reset();
        sharedSurface_.reset();
        sharedMutex_.Reset();
    }

    [[nodiscard]] bool initializeDuplication(bool recreation)
    {
        resetDuplication();
        if (recreation)
        {
            std::scoped_lock lock(mutex_);
            tracker_.values.state = DesktopCaptureStatus::recreating;
            ++tracker_.values.recreationAttempts;
        }
        const auto& selected = *config_.captureMonitor.dxgiOutput;
        ComPtr<IDXGIFactory1> factory;
        HRESULT result = CreateDXGIFactory1(IID_PPV_ARGS(&factory));
        if (FAILED(result))
        {
            setError("CreateDXGIFactory1 for desktop capture failed: " + hresultText(result));
            return false;
        }
        ComPtr<IDXGIAdapter1> adapter;
        for (UINT index = 0U;; ++index)
        {
            ComPtr<IDXGIAdapter1> candidate;
            result = factory->EnumAdapters1(index, &candidate);
            if (result == DXGI_ERROR_NOT_FOUND)
            {
                break;
            }
            if (FAILED(result))
            {
                setError("IDXGIFactory1::EnumAdapters1 failed: " + hresultText(result));
                return false;
            }
            DXGI_ADAPTER_DESC1 description{};
            result = candidate->GetDesc1(&description);
            if (FAILED(result))
            {
                continue;
            }
            const platform::windows::DxgiAdapterLuid luid{
                description.AdapterLuid.LowPart, description.AdapterLuid.HighPart};
            if (luid == selected.adapterLuid)
            {
                adapter = std::move(candidate);
                break;
            }
        }
        if (!adapter)
        {
            setState(DesktopCaptureStatus::outputMissing);
            setError("Capture adapter "
                + platform::windows::dxgiAdapterLuidText(selected.adapterLuid)
                + " is no longer available; no other adapter was selected.");
            return false;
        }
        for (UINT index = 0U;; ++index)
        {
            ComPtr<IDXGIOutput> candidate;
            result = adapter->EnumOutputs(index, &candidate);
            if (result == DXGI_ERROR_NOT_FOUND)
            {
                break;
            }
            if (FAILED(result))
            {
                setError("IDXGIAdapter::EnumOutputs failed: " + hresultText(result));
                return false;
            }
            DXGI_OUTPUT_DESC description{};
            if (FAILED(candidate->GetDesc(&description)))
            {
                continue;
            }
            char name[64]{};
            const int converted = WideCharToMultiByte(CP_UTF8, 0, description.DeviceName, -1,
                name, static_cast<int>(std::size(name)), nullptr, nullptr);
            if (converted > 0 && selected.deviceName == name)
            {
                output_ = std::move(candidate);
                outputDescription_ = description;
                break;
            }
        }
        if (!output_)
        {
            setState(DesktopCaptureStatus::outputMissing);
            setError("Capture output " + selected.deviceName
                + " is no longer present; no different output was selected.");
            return false;
        }
        constexpr std::array featureLevels{
            D3D_FEATURE_LEVEL_11_1, D3D_FEATURE_LEVEL_11_0,
            D3D_FEATURE_LEVEL_10_1, D3D_FEATURE_LEVEL_10_0};
        D3D_FEATURE_LEVEL selectedFeature{};
        result = D3D11CreateDevice(adapter.Get(), D3D_DRIVER_TYPE_UNKNOWN, nullptr,
            D3D11_CREATE_DEVICE_BGRA_SUPPORT, featureLevels.data(),
            static_cast<UINT>(featureLevels.size()), D3D11_SDK_VERSION,
            &device_, &selectedFeature, &context_);
        if (FAILED(result))
        {
            setError("D3D11CreateDevice on capture adapter " + selected.adapterDescription
                + " failed: " + hresultText(result));
            return false;
        }
        if (cpuFallback_ && !scaler_.initialize(device_.Get(), context_.Get()))
        {
            setError("Initializing capture-adapter GPU scaler failed: " + scaler_.error());
            return false;
        }
        ComPtr<IDXGIOutput1> output1;
        result = output_.As(&output1);
        if (FAILED(result))
        {
            setError("Capture output does not expose IDXGIOutput1: " + hresultText(result));
            return false;
        }
        result = output1->DuplicateOutput(device_.Get(), &duplication_);
        if (FAILED(result))
        {
            const auto status = result == DXGI_ERROR_UNSUPPORTED
                ? DesktopCaptureStatus::unsupported : DesktopCaptureStatus::fatalError;
            setState(status);
            setError("IDXGIOutput1::DuplicateOutput failed for " + selected.deviceName
                + " on " + selected.adapterDescription + ": " + hresultText(result));
            return false;
        }
        {
            std::scoped_lock lock(mutex_);
            tracker_.values.state = DesktopCaptureStatus::active;
            if (recreation)
            {
                ++tracker_.values.successfulRecreations;
                ++tracker_.values.recoveryGeneration;
            }
        }
        return true;
    }

    [[nodiscard]] bool ensureSharedSurface(const D3D11_TEXTURE2D_DESC& source)
    {
        if (sharedSurface_ && source.Width == sharedWidth_ && source.Height == sharedHeight_
            && source.Format == sharedFormat_)
        {
            return true;
        }
        if (!supportedFormat(source.Format))
        {
            setError("Desktop Duplication returned unsupported DXGI format "
                + std::to_string(static_cast<unsigned int>(source.Format))
                + "; HDR conversion is not implemented.");
            return false;
        }
        D3D11_TEXTURE2D_DESC description{};
        description.Width = source.Width;
        description.Height = source.Height;
        description.MipLevels = 1;
        description.ArraySize = 1;
        description.Format = source.Format;
        description.SampleDesc.Count = 1;
        description.Usage = D3D11_USAGE_DEFAULT;
        description.BindFlags = D3D11_BIND_SHADER_RESOURCE;
        description.MiscFlags = D3D11_RESOURCE_MISC_SHARED_NTHANDLE
            | D3D11_RESOURCE_MISC_SHARED_KEYEDMUTEX;
        ComPtr<ID3D11Texture2D> texture;
        HRESULT result = device_->CreateTexture2D(&description, nullptr, &texture);
        if (FAILED(result))
        {
            setError("Creating owned shared desktop texture failed: " + hresultText(result));
            return false;
        }
        ComPtr<IDXGIResource1> resource;
        result = texture.As(&resource);
        if (FAILED(result))
        {
            setError("Owned desktop texture does not expose IDXGIResource1: " + hresultText(result));
            return false;
        }
        HANDLE handle{};
        result = resource->CreateSharedHandle(nullptr,
            DXGI_SHARED_RESOURCE_READ | DXGI_SHARED_RESOURCE_WRITE, nullptr, &handle);
        if (FAILED(result))
        {
            setError("IDXGIResource1::CreateSharedHandle failed: " + hresultText(result));
            return false;
        }
        result = texture.As(&sharedMutex_);
        if (FAILED(result))
        {
            CloseHandle(handle);
            setError("Owned desktop texture does not expose IDXGIKeyedMutex: " + hresultText(result));
            return false;
        }
        sharedSurface_ = std::shared_ptr<DesktopCaptureSurface>(
            new DesktopCaptureSurface(texture.Detach(), handle));
        sharedWidth_ = source.Width;
        sharedHeight_ = source.Height;
        sharedFormat_ = source.Format;
        return true;
    }

    [[nodiscard]] bool copyThroughCpu(
        ID3D11Texture2D* source,
        const D3D11_TEXTURE2D_DESC& description,
        DesktopCaptureFrame& frame)
    {
        const auto resolved = resolveDesktopScalePlan(
            description.Width, description.Height, config_.scaling);
        if (!resolved.plan.has_value())
        {
            setError("Resolving capture-side GPU scale failed: " + resolved.error);
            return false;
        }
        const auto readback = scaler_.process(source, *resolved.plan, config_.scaling.filter);
        if (!readback.success)
        {
            setError("Capture-side GPU scale/readback failed: " + readback.error);
            return false;
        }
        const auto scalerStatistics = scaler_.statistics();
        {
            std::scoped_lock lock(mutex_);
            tracker_.values.stagingCopies = scalerStatistics.stagingCopies;
            tracker_.values.stagingMaps = scalerStatistics.stagingMaps;
            tracker_.values.stagingMapSuccesses = scalerStatistics.stagingMaps;
            tracker_.values.cpuBuffersCreated = desktopStagingRingCapacity;
            tracker_.values.gpuScaleDraws = scalerStatistics.gpuScaleDraws;
            tracker_.values.scalerResourceRecreations =
                scalerStatistics.resourceRecreations;
            tracker_.values.stagingRingContentions = scalerStatistics.ringContentions;
            if (readback.frameReady)
            {
                tracker_.recordScaleSubmission(readback.gpuSubmissionMilliseconds);
                tracker_.recordMapWait(readback.mapWaitMilliseconds);
                tracker_.recordCpuRepack(readback.cpuRepackMilliseconds);
            }
        }
        if (!readback.frameReady)
        {
            return true;
        }
        if (!firstStagingMapTraced_)
        {
            std::cout << "desktop-first-frame stage=cpu_staging_map_succeeded"
                      << " capture_candidate=" << tracker_.values.acquiredFrames + 1U << '\n';
            firstStagingMapTraced_ = true;
        }
        frame.sourceWidth = readback.width;
        frame.sourceHeight = readback.height;
        frame.sourceFormat = readback.format;
        frame.cpuPixels = readback.pixels;
        frame.cpuRowPitch = readback.rowPitch;
        frame.originalSourceWidth = description.Width;
        frame.originalSourceHeight = description.Height;
        frame.cropX = resolved.plan->crop.x;
        frame.cropY = resolved.plan->crop.y;
        frame.cropWidth = resolved.plan->crop.width;
        frame.cropHeight = resolved.plan->crop.height;
        {
            std::scoped_lock lock(mutex_);
            tracker_.values.originalSourceBytes += resolved.plan->sourceBytesPerFrame;
            tracker_.values.cropBytes += resolved.plan->cropBytesPerFrame;
            tracker_.values.scaledTargetBytes += resolved.plan->targetBytesPerFrame;
            tracker_.values.stagingBytesMapped += readback.mappedBytes;
            tracker_.values.cpuBytesCopied += readback.mappedBytes;
            tracker_.values.cropX = resolved.plan->crop.x;
            tracker_.values.cropY = resolved.plan->crop.y;
            tracker_.values.cropWidth = resolved.plan->crop.width;
            tracker_.values.cropHeight = resolved.plan->crop.height;
            tracker_.values.transferWidth = readback.width;
            tracker_.values.transferHeight = readback.height;
        }
        return true;
    }

    void collectMetadata(const DXGI_OUTDUPL_FRAME_INFO& information, DesktopCaptureFrame& frame)
    {
        frame.metadataBytes = information.TotalMetadataBufferSize;
        if (information.TotalMetadataBufferSize > metadata_.size())
        {
            metadata_.resize(information.TotalMetadataBufferSize);
        }
        if (!metadata_.empty() && information.TotalMetadataBufferSize > 0U)
        {
            UINT required{};
            HRESULT result = duplication_->GetFrameMoveRects(
                static_cast<UINT>(metadata_.size()),
                reinterpret_cast<DXGI_OUTDUPL_MOVE_RECT*>(metadata_.data()), &required);
            if (SUCCEEDED(result))
            {
                frame.moveRectCount = required / sizeof(DXGI_OUTDUPL_MOVE_RECT);
            }
            result = duplication_->GetFrameDirtyRects(
                static_cast<UINT>(metadata_.size()),
                reinterpret_cast<RECT*>(metadata_.data()), &required);
            if (SUCCEEDED(result))
            {
                frame.dirtyRectCount = required / sizeof(RECT);
            }
        }
        frame.pointerVisible = config_.options.showCursor
            && information.PointerPosition.Visible != FALSE;
        frame.pointerX = information.PointerPosition.Position.x;
        frame.pointerY = information.PointerPosition.Position.y;
    }

    void run(std::stop_token stopToken)
    {
        if (!initializeDuplication(false))
        {
            return;
        }
        auto nextCapture = std::chrono::steady_clock::now();
        while (!stopToken.stop_requested())
        {
            double cadence = maximumFramesPerSecond_.load(std::memory_order_acquire);
            if (cadence <= 0.0)
            {
                std::unique_lock cadenceLock(cadenceMutex_);
                (void)cadenceCondition_.wait(cadenceLock, stopToken, [this] {
                    return maximumFramesPerSecond_.load(std::memory_order_acquire) > 0.0;
                });
                nextCapture = std::chrono::steady_clock::now();
                continue;
            }
            DXGI_OUTDUPL_FRAME_INFO information{};
            ComPtr<IDXGIResource> resource;
            {
                std::scoped_lock lock(mutex_);
                ++tracker_.values.captureAttempts;
            }
            const auto acquireStart = std::chrono::steady_clock::now();
            retention_.recordCaptureAttempt(acquireStart);
            const HRESULT acquired = duplication_->AcquireNextFrame(
                config_.options.timeoutMilliseconds, &information, &resource);
            const auto acquireEnd = std::chrono::steady_clock::now();
            {
                std::scoped_lock lock(mutex_);
                tracker_.recordAcquireDuration(std::chrono::duration<double, std::milli>(
                    acquireEnd - acquireStart).count());
            }
            if (acquired == DXGI_ERROR_WAIT_TIMEOUT)
            {
                std::scoped_lock lock(mutex_);
                ++tracker_.values.waitTimeouts;
                tracker_.values.state = DesktopCaptureStatus::waitTimeout;
                retention_.recordWaitTimeout(acquireEnd,
                    std::chrono::milliseconds(config_.options.staleThresholdMilliseconds));
                tracker_.values.availability = retention_.availability();
                continue;
            }
            if (acquired == DXGI_ERROR_ACCESS_LOST
                || acquired == DXGI_ERROR_DEVICE_REMOVED
                || acquired == DXGI_ERROR_DEVICE_RESET)
            {
                {
                    std::scoped_lock lock(mutex_);
                    ++tracker_.values.accessLossEvents;
                    tracker_.values.state = DesktopCaptureStatus::accessLost;
                    retention_.recordAccessLost();
                    tracker_.values.availability = retention_.availability();
                }
                if (stopToken.stop_requested())
                {
                    break;
                }
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(config_.options.retryMilliseconds));
                if (!initializeDuplication(true))
                {
                    break;
                }
                continue;
            }
            if (FAILED(acquired))
            {
                setError("IDXGIOutputDuplication::AcquireNextFrame failed: "
                    + hresultText(acquired));
                break;
            }
            struct FrameRelease
            {
                IDXGIOutputDuplication* duplication{};
                ~FrameRelease() { if (duplication != nullptr) { (void)duplication->ReleaseFrame(); } }
            } release{duplication_.Get()};

            ComPtr<ID3D11Texture2D> source;
            HRESULT result = resource.As(&source);
            if (FAILED(result))
            {
                setError("Acquired desktop resource is not an ID3D11Texture2D: "
                    + hresultText(result));
                break;
            }
            D3D11_TEXTURE2D_DESC description{};
            source->GetDesc(&description);
            DesktopCaptureFrame frame;
            frame.captureHostTimestamp = std::chrono::steady_clock::now();
            frame.lastPresentTimestamp = information.LastPresentTime.QuadPart;
            frame.accumulatedFrames = information.AccumulatedFrames;
            frame.sourceWidth = description.Width;
            frame.sourceHeight = description.Height;
            frame.sourceFormat = static_cast<std::uint32_t>(description.Format);
            frame.rotation = rotationFromDxgi(outputDescription_.Rotation);
            frame.sourceMonitorDeviceName = config_.captureMonitor.deviceName;
            frame.sourceAdapterLuid = config_.captureMonitor.dxgiOutput->adapterLuid;
            frame.valid = true;
            frame.status = DesktopCaptureStatus::active;
            {
                std::scoped_lock lock(mutex_);
                frame.recoveryGeneration = tracker_.values.recoveryGeneration;
            }
            collectMetadata(information, frame);
            if (!firstAcquiredTraced_)
            {
                std::cout << "desktop-first-frame stage=acquired capture_candidate="
                          << tracker_.values.acquiredFrames + 1U << '\n';
                firstAcquiredTraced_ = true;
            }
            if (cpuFallback_)
            {
                if (!copyThroughCpu(source.Get(), description, frame))
                {
                    break;
                }
                if (!frame.cpuPixels)
                {
                    continue;
                }
            }
            else
            {
                if (!ensureSharedSurface(description))
                {
                    break;
                }
                result = sharedMutex_->AcquireSync(0U, 0U);
                if (result == WAIT_TIMEOUT)
                {
                    continue;
                }
                if (FAILED(result))
                {
                    setError("Capture keyed-mutex AcquireSync failed: " + hresultText(result));
                    break;
                }
                context_->CopyResource(
                    static_cast<ID3D11Texture2D*>(sharedSurface_->nativeTexture()), source.Get());
                result = sharedMutex_->ReleaseSync(1U);
                if (FAILED(result))
                {
                    setError("Capture keyed-mutex ReleaseSync failed: " + hresultText(result));
                    break;
                }
                frame.surface = sharedSurface_;
            }
            if (cpuFallback_ && !firstFrameDumpAttempted_
                && config_.options.firstFrameBmpPath.has_value())
            {
                firstFrameDumpAttempted_ = true;
                const auto dump = writeDesktopFrameBmp(frame, *config_.options.firstFrameBmpPath);
                if (dump.success)
                {
                    std::cout << "Desktop first-frame BMP written: "
                              << *config_.options.firstFrameBmpPath << '\n';
                }
                else
                {
                    std::cerr << "Desktop first-frame BMP failed: " << dump.error << '\n';
                }
            }
            const std::uint64_t publishedSequence = bridge_.publish(frame);
            const bool published = publishedSequence != 0U;
            if (cpuFallback_ && published)
            {
                std::scoped_lock lock(mutex_);
                ++tracker_.values.cpuFramesPublished;
                tracker_.values.latestPublishedSequence = publishedSequence;
            }
            if (cpuFallback_ && published && !firstPublishedTraced_)
            {
                std::cout << "desktop-first-frame stage=cpu_frame_published sequence="
                          << publishedSequence << '\n';
                firstPublishedTraced_ = true;
            }
            {
                std::scoped_lock lock(mutex_);
                tracker_.recordAcquired(frame.captureHostTimestamp);
                tracker_.values.state = DesktopCaptureStatus::active;
                retention_.recordValidFrame(frame.captureHostTimestamp);
                tracker_.values.availability = retention_.availability();
                if (cpuFallback_)
                {
                    ++tracker_.values.cpuFallbackCopies;
                    tracker_.values.cpuFallbackBytes += static_cast<std::uint64_t>(
                        frame.cpuRowPitch) * frame.sourceHeight;
                }
                else
                {
                    ++tracker_.values.gpuCopies;
                }
                tracker_.values.sourceWidth = description.Width;
                tracker_.values.sourceHeight = description.Height;
                tracker_.values.sourceFormat = static_cast<std::uint32_t>(description.Format);
                tracker_.values.sourceRotation = frame.rotation;
                tracker_.values.dirtyRectCount += frame.dirtyRectCount;
                tracker_.values.moveRectCount += frame.moveRectCount;
                tracker_.values.pointerMetadataCount += frame.pointerVisible ? 1U : 0U;
                if (!published)
                {
                    ++tracker_.values.captureErrors;
                }
            }
            const auto capturePeriod = std::chrono::duration_cast<
                std::chrono::steady_clock::duration>(std::chrono::duration<double>(
                    1.0 / cadence));
            nextCapture += capturePeriod;
            const auto cadenceNow = std::chrono::steady_clock::now();
            if (nextCapture > cadenceNow)
            {
                std::unique_lock cadenceLock(cadenceMutex_);
                (void)cadenceCondition_.wait_until(
                    cadenceLock, stopToken, nextCapture, [this, cadence] {
                        return maximumFramesPerSecond_.load(std::memory_order_acquire)
                            != cadence;
                    });
            }
            else
            {
                nextCapture = cadenceNow;
            }
        }
        {
            std::scoped_lock lock(mutex_);
            tracker_.values.state = desktopCaptureLoopExitStatus(
                tracker_.values.state, stopToken.stop_requested());
        }
        resetDuplication();
    }

    DesktopCaptureBridge& bridge_;
    DesktopDuplicationConfig config_;
    mutable std::mutex mutex_;
    DesktopCaptureStatisticsTracker tracker_;
    std::string error_;
    std::jthread worker_;
    std::atomic<double> maximumFramesPerSecond_{30.0};
    std::mutex cadenceMutex_;
    std::condition_variable_any cadenceCondition_;
    ComPtr<ID3D11Device> device_;
    ComPtr<ID3D11DeviceContext> context_;
    ComPtr<IDXGIOutput> output_;
    DXGI_OUTPUT_DESC outputDescription_{};
    ComPtr<IDXGIOutputDuplication> duplication_;
    DesktopCaptureScaler scaler_;
    DesktopFrameRetention retention_;
    std::shared_ptr<DesktopCaptureSurface> sharedSurface_;
    ComPtr<IDXGIKeyedMutex> sharedMutex_;
    std::uint32_t sharedWidth_{};
    std::uint32_t sharedHeight_{};
    DXGI_FORMAT sharedFormat_{DXGI_FORMAT_UNKNOWN};
    std::vector<std::byte> metadata_;
    bool cpuFallback_{};
    bool firstAcquiredTraced_{};
    bool firstStagingMapTraced_{};
    bool firstPublishedTraced_{};
    bool firstFrameDumpAttempted_{};
};

DesktopDuplicationCapture::DesktopDuplicationCapture(DesktopCaptureBridge& bridge)
    : implementation_(std::make_unique<Implementation>(bridge))
{
}

DesktopDuplicationCapture::~DesktopDuplicationCapture() = default;

bool DesktopDuplicationCapture::start(DesktopDuplicationConfig config)
{
    return implementation_->start(std::move(config));
}

void DesktopDuplicationCapture::setMaximumFramesPerSecond(double value) noexcept
{
    implementation_->setMaximumFramesPerSecond(value);
}

void DesktopDuplicationCapture::stop() { implementation_->stop(); }

DesktopCaptureStatistics DesktopDuplicationCapture::statistics() const
{
    return implementation_->statistics();
}

DesktopCaptureStatistics DesktopDuplicationCapture::detailedStatistics() const
{
    return implementation_->detailedStatistics();
}

std::string DesktopDuplicationCapture::error() const { return implementation_->error(); }

} // namespace xreal::capture
