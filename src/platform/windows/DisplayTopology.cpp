#include "platform/windows/DisplayTopology.hpp"

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

#include <dxgi1_2.h>
#include <wrl/client.h>

#include <algorithm>
#include <cctype>
#include <sstream>
#include <unordered_map>
#include <utility>

namespace xreal::platform::windows
{
namespace
{

using Microsoft::WRL::ComPtr;

[[nodiscard]] std::string utf8(const wchar_t* text)
{
    if (text == nullptr || *text == L'\0')
    {
        return {};
    }
    const int size = WideCharToMultiByte(CP_UTF8, 0, text, -1, nullptr, 0, nullptr, nullptr);
    if (size <= 1)
    {
        return {};
    }
    std::string result(static_cast<std::size_t>(size), '\0');
    (void)WideCharToMultiByte(CP_UTF8, 0, text, -1, result.data(), size, nullptr, nullptr);
    result.pop_back();
    return result;
}

[[nodiscard]] std::string hresultText(HRESULT value)
{
    std::ostringstream output;
    output << "HRESULT 0x" << std::hex << std::uppercase
           << static_cast<unsigned long>(value);
    return output.str();
}

[[nodiscard]] std::string normalizedDeviceName(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
        return static_cast<char>(std::toupper(character));
    });
    return value;
}

BOOL CALLBACK collectMonitor(HMONITOR handle, HDC, LPRECT, LPARAM context)
{
    auto& monitors = *reinterpret_cast<std::vector<MonitorInformation>*>(context);
    MONITORINFOEXW information{};
    information.cbSize = sizeof(information);
    if (!GetMonitorInfoW(handle, &information))
    {
        return TRUE;
    }
    MonitorInformation result;
    result.deviceName = utf8(information.szDevice);
    result.left = information.rcMonitor.left;
    result.top = information.rcMonitor.top;
    result.right = information.rcMonitor.right;
    result.bottom = information.rcMonitor.bottom;
    result.workLeft = information.rcWork.left;
    result.workTop = information.rcWork.top;
    result.workRight = information.rcWork.right;
    result.workBottom = information.rcWork.bottom;
    result.primary = (information.dwFlags & MONITORINFOF_PRIMARY) != 0U;
    monitors.push_back(std::move(result));
    return TRUE;
}

[[nodiscard]] std::vector<MonitorInformation> activeMonitors()
{
    std::vector<MonitorInformation> result;
    (void)EnumDisplayMonitors(nullptr, nullptr, collectMonitor,
        reinterpret_cast<LPARAM>(&result));
    std::stable_sort(result.begin(), result.end(), [](const auto& left, const auto& right) {
        return left.primary && !right.primary;
    });
    for (std::size_t index = 0; index < result.size(); ++index)
    {
        result[index].index = static_cast<unsigned int>(index);
    }
    return result;
}

void attachStableMonitorIdentities(std::span<MonitorInformation> monitors)
{
    UINT32 pathCount{};
    UINT32 modeCount{};
    LONG status = GetDisplayConfigBufferSizes(
        QDC_ONLY_ACTIVE_PATHS, &pathCount, &modeCount);
    if (status != ERROR_SUCCESS || pathCount == 0U)
    {
        return;
    }
    std::vector<DISPLAYCONFIG_PATH_INFO> paths(pathCount);
    std::vector<DISPLAYCONFIG_MODE_INFO> modes(modeCount);
    status = QueryDisplayConfig(
        QDC_ONLY_ACTIVE_PATHS,
        &pathCount,
        paths.data(),
        &modeCount,
        modes.data(),
        nullptr);
    if (status != ERROR_SUCCESS)
    {
        return;
    }
    std::unordered_map<std::string, std::pair<std::string, std::string>> identities;
    for (UINT32 index = 0U; index < pathCount; ++index)
    {
        const auto& path = paths[index];
        DISPLAYCONFIG_SOURCE_DEVICE_NAME source{};
        source.header.type = DISPLAYCONFIG_DEVICE_INFO_GET_SOURCE_NAME;
        source.header.size = sizeof(source);
        source.header.adapterId = path.sourceInfo.adapterId;
        source.header.id = path.sourceInfo.id;
        DISPLAYCONFIG_TARGET_DEVICE_NAME target{};
        target.header.type = DISPLAYCONFIG_DEVICE_INFO_GET_TARGET_NAME;
        target.header.size = sizeof(target);
        target.header.adapterId = path.targetInfo.adapterId;
        target.header.id = path.targetInfo.id;
        if (DisplayConfigGetDeviceInfo(&source.header) != ERROR_SUCCESS
            || DisplayConfigGetDeviceInfo(&target.header) != ERROR_SUCCESS)
        {
            continue;
        }
        const std::string sourceName = normalizedDeviceName(utf8(source.viewGdiDeviceName));
        identities[sourceName] = {
            utf8(target.monitorDevicePath),
            utf8(target.monitorFriendlyDeviceName),
        };
    }
    for (auto& monitor : monitors)
    {
        const auto found = identities.find(normalizedDeviceName(monitor.deviceName));
        if (found != identities.end())
        {
            monitor.stableIdentity = found->second.first;
            monitor.friendlyName = found->second.second;
        }
    }
}

[[nodiscard]] std::vector<DxgiOutputInformation> activeDxgiOutputs(std::string& error)
{
    ComPtr<IDXGIFactory1> factory;
    HRESULT result = CreateDXGIFactory1(IID_PPV_ARGS(&factory));
    if (FAILED(result))
    {
        error = "CreateDXGIFactory1 failed while enumerating displays: " + hresultText(result);
        return {};
    }

    std::vector<DxgiOutputInformation> outputs;
    for (UINT adapterIndex = 0U;; ++adapterIndex)
    {
        ComPtr<IDXGIAdapter1> adapter;
        result = factory->EnumAdapters1(adapterIndex, &adapter);
        if (result == DXGI_ERROR_NOT_FOUND)
        {
            break;
        }
        if (FAILED(result))
        {
            error = "IDXGIFactory1::EnumAdapters1 failed for adapter "
                + std::to_string(adapterIndex) + ": " + hresultText(result);
            return outputs;
        }
        DXGI_ADAPTER_DESC1 adapterDescription{};
        result = adapter->GetDesc1(&adapterDescription);
        if (FAILED(result))
        {
            error = "IDXGIAdapter1::GetDesc1 failed for adapter "
                + std::to_string(adapterIndex) + ": " + hresultText(result);
            return outputs;
        }
        for (UINT outputIndex = 0U;; ++outputIndex)
        {
            ComPtr<IDXGIOutput> output;
            result = adapter->EnumOutputs(outputIndex, &output);
            if (result == DXGI_ERROR_NOT_FOUND)
            {
                break;
            }
            if (FAILED(result))
            {
                error = "IDXGIAdapter::EnumOutputs failed for adapter "
                    + std::to_string(adapterIndex) + ", output "
                    + std::to_string(outputIndex) + ": " + hresultText(result);
                return outputs;
            }
            DXGI_OUTPUT_DESC outputDescription{};
            result = output->GetDesc(&outputDescription);
            if (FAILED(result))
            {
                error = "IDXGIOutput::GetDesc failed for adapter "
                    + std::to_string(adapterIndex) + ", output "
                    + std::to_string(outputIndex) + ": " + hresultText(result);
                return outputs;
            }
            if (!outputDescription.AttachedToDesktop)
            {
                continue;
            }
            DxgiOutputInformation information;
            information.adapterIndex = adapterIndex;
            information.outputIndex = outputIndex;
            information.deviceName = utf8(outputDescription.DeviceName);
            information.left = outputDescription.DesktopCoordinates.left;
            information.top = outputDescription.DesktopCoordinates.top;
            information.right = outputDescription.DesktopCoordinates.right;
            information.bottom = outputDescription.DesktopCoordinates.bottom;
            information.attachedToDesktop = outputDescription.AttachedToDesktop != FALSE;
            information.adapterDescription = utf8(adapterDescription.Description);
            information.adapterLuid = {
                adapterDescription.AdapterLuid.LowPart,
                adapterDescription.AdapterLuid.HighPart,
            };
            outputs.push_back(std::move(information));
        }
    }
    return outputs;
}

} // namespace

void associateDxgiOutputs(
    std::span<MonitorInformation> monitors,
    std::span<const DxgiOutputInformation> outputs)
{
    for (auto& monitor : monitors)
    {
        monitor.dxgiOutput.reset();
        const std::string monitorName = normalizedDeviceName(monitor.deviceName);
        const auto matching = std::find_if(outputs.begin(), outputs.end(), [&](const auto& output) {
            return output.attachedToDesktop
                && normalizedDeviceName(output.deviceName) == monitorName;
        });
        if (matching != outputs.end())
        {
            monitor.dxgiOutput = *matching;
        }
    }
}

std::optional<WindowPlacement> calculateWindowPlacement(
    const MonitorInformation& monitor,
    unsigned int requestedWidth,
    unsigned int requestedHeight,
    std::optional<int> monitorRelativeX,
    std::optional<int> monitorRelativeY,
    bool fullscreen) noexcept
{
    const int monitorWidth = monitor.right - monitor.left;
    const int monitorHeight = monitor.bottom - monitor.top;
    if (monitorWidth <= 0 || monitorHeight <= 0 || requestedWidth == 0U || requestedHeight == 0U)
    {
        return std::nullopt;
    }
    if (fullscreen)
    {
        return WindowPlacement{
            monitor.left,
            monitor.top,
            static_cast<unsigned int>(monitorWidth),
            static_cast<unsigned int>(monitorHeight),
        };
    }

    const unsigned int width = std::min(requestedWidth, static_cast<unsigned int>(monitorWidth));
    const unsigned int height = std::min(requestedHeight, static_cast<unsigned int>(monitorHeight));
    const int requestedX = monitorRelativeX.has_value()
        ? monitor.left + *monitorRelativeX
        : monitor.left + (monitorWidth - static_cast<int>(width)) / 2;
    const int requestedY = monitorRelativeY.has_value()
        ? monitor.top + *monitorRelativeY
        : monitor.top + (monitorHeight - static_cast<int>(height)) / 2;
    const int maximumX = monitor.right - static_cast<int>(width);
    const int maximumY = monitor.bottom - static_cast<int>(height);
    return WindowPlacement{
        std::clamp(requestedX, monitor.left, maximumX),
        std::clamp(requestedY, monitor.top, maximumY),
        width,
        height,
    };
}

DisplayTopologyResult enumerateDisplayTopology()
{
    DisplayTopologyResult result;
    result.monitors = activeMonitors();
    attachStableMonitorIdentities(result.monitors);
    const auto outputs = activeDxgiOutputs(result.error);
    associateDxgiOutputs(result.monitors, outputs);
    return result;
}

const MonitorInformation* findMonitorByStableIdentity(
    std::span<const MonitorInformation> monitors,
    std::string_view stableIdentity) noexcept
{
    if (stableIdentity.empty())
    {
        return nullptr;
    }
    const auto found = std::find_if(monitors.begin(), monitors.end(), [&](const auto& monitor) {
        return monitor.stableIdentity == stableIdentity;
    });
    return found == monitors.end() ? nullptr : &*found;
}

} // namespace xreal::platform::windows
