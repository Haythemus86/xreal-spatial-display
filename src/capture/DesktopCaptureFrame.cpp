#include "capture/DesktopCaptureFrame.hpp"

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

#include <d3d11.h>
#include <wrl/client.h>

namespace xreal::capture
{

class DesktopCaptureSurface::Implementation
{
public:
    Implementation(void* nativeTexture, void* sharedHandleValue)
        : sharedHandle(static_cast<HANDLE>(sharedHandleValue))
    {
        if (nativeTexture != nullptr)
        {
            texture.Attach(static_cast<ID3D11Texture2D*>(nativeTexture));
        }
    }

    ~Implementation()
    {
        if (sharedHandle != nullptr)
        {
            CloseHandle(sharedHandle);
        }
    }

    Microsoft::WRL::ComPtr<ID3D11Texture2D> texture;
    HANDLE sharedHandle{};
};

DesktopCaptureSurface::DesktopCaptureSurface(void* nativeTexture, void* sharedHandle)
    : implementation_(std::make_unique<Implementation>(nativeTexture, sharedHandle))
{
}

DesktopCaptureSurface::~DesktopCaptureSurface() = default;

void* DesktopCaptureSurface::sharedHandle() const noexcept
{
    return implementation_->sharedHandle;
}

void* DesktopCaptureSurface::nativeTexture() const noexcept
{
    return implementation_->texture.Get();
}

std::string desktopCaptureStatusText(DesktopCaptureStatus status)
{
    switch (status)
    {
    case DesktopCaptureStatus::disabled: return "disabled";
    case DesktopCaptureStatus::initializing: return "initializing";
    case DesktopCaptureStatus::active: return "active";
    case DesktopCaptureStatus::waitTimeout: return "wait_timeout";
    case DesktopCaptureStatus::accessLost: return "access_lost";
    case DesktopCaptureStatus::recreating: return "recreating";
    case DesktopCaptureStatus::outputMissing: return "output_missing";
    case DesktopCaptureStatus::unsupported: return "unsupported";
    case DesktopCaptureStatus::fatalError: return "fatal_error";
    case DesktopCaptureStatus::shuttingDown: return "shutting_down";
    }
    return "unknown";
}

DesktopCaptureStatus desktopCaptureLoopExitStatus(
    DesktopCaptureStatus current,
    bool stopRequested) noexcept
{
    return stopRequested ? DesktopCaptureStatus::shuttingDown : current;
}

} // namespace xreal::capture
