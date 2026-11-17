#include "WindowsVirtualDisplayTransport.hpp"

#include "platform/windows/DisplayTopology.hpp"

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

#include <iomanip>
#include <algorithm>
#include <sstream>
#include <string_view>

namespace xreal::virtual_display
{
namespace
{

class UniqueHandle
{
public:
    explicit UniqueHandle(HANDLE handle) noexcept : handle_(handle) {}
    ~UniqueHandle()
    {
        if (handle_ != INVALID_HANDLE_VALUE)
        {
            (void)CloseHandle(handle_);
        }
    }
    UniqueHandle(const UniqueHandle&) = delete;
    UniqueHandle& operator=(const UniqueHandle&) = delete;
    [[nodiscard]] HANDLE get() const noexcept { return handle_; }

private:
    HANDLE handle_{INVALID_HANDLE_VALUE};
};

[[nodiscard]] std::string windowsError(std::string_view operation, DWORD code)
{
    LPSTR message{};
    const DWORD count = FormatMessageA(
        FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM
            | FORMAT_MESSAGE_IGNORE_INSERTS,
        nullptr, code, 0U, reinterpret_cast<LPSTR>(&message), 0U, nullptr);
    std::ostringstream output;
    output << operation << " failed with Win32 error " << code;
    if (count > 0U && message != nullptr)
    {
        output << ": " << std::string_view(message, count);
        (void)LocalFree(message);
    }
    return output.str();
}

void attachActiveWindowsIdentities(VirtualDisplayProtocolResponse& response)
{
    const auto topology = platform::windows::enumerateDisplayTopology();
    for (std::size_t index = 0U; index < response.monitorCount; ++index)
    {
        auto& wireMonitor = response.monitors[index];
        const auto friendlyEnd = std::find(
            wireMonitor.friendlyName.begin(), wireMonitor.friendlyName.end(), '\0');
        const std::string friendlyName(wireMonitor.friendlyName.begin(), friendlyEnd);
        const std::string edidName = "XREAL VDISP "
            + std::to_string(wireMonitor.logicalIndex + 1U);
        const auto matching = std::find_if(
            topology.monitors.begin(), topology.monitors.end(), [&](const auto& monitor) {
                return monitor.friendlyName == friendlyName
                    || monitor.friendlyName == edidName;
            });
        if (matching == topology.monitors.end())
        {
            continue;
        }
        const auto copy = [](auto& destination, std::string_view source) {
            destination.fill('\0');
            const auto count = std::min(source.size(), destination.size() - 1U);
            std::copy_n(source.data(), count, destination.data());
        };
        copy(wireMonitor.windowsDeviceName, matching->deviceName);
        copy(wireMonitor.windowsMonitorIdentity, matching->stableIdentity);
    }
}

} // namespace

std::optional<VirtualDisplayProtocolResponse> WindowsVirtualDisplayTransport::transact(
    const VirtualDisplayProtocolRequest& request)
{
    error_.clear();
    UniqueHandle device(CreateFileW(
        L"\\\\.\\XrealVirtualDisplay",
        GENERIC_READ | GENERIC_WRITE,
        FILE_SHARE_READ | FILE_SHARE_WRITE,
        nullptr,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        nullptr));
    if (device.get() == INVALID_HANDLE_VALUE)
    {
        error_ = windowsError(
            "Opening the XREAL virtual-display driver", GetLastError());
        return std::nullopt;
    }
    VirtualDisplayProtocolResponse response;
    DWORD returnedBytes{};
    if (!DeviceIoControl(
            device.get(),
            virtualDisplayIoControlCode,
            const_cast<VirtualDisplayProtocolRequest*>(&request),
            static_cast<DWORD>(sizeof(request)),
            &response,
            static_cast<DWORD>(sizeof(response)),
            &returnedBytes,
            nullptr))
    {
        error_ = windowsError("Sending the virtual-display configuration", GetLastError());
        return std::nullopt;
    }
    if (returnedBytes != sizeof(response))
    {
        error_ = "The virtual-display driver returned an unexpected response size: "
            + std::to_string(returnedBytes) + " bytes.";
        return std::nullopt;
    }
    attachActiveWindowsIdentities(response);
    return response;
}

const std::string& WindowsVirtualDisplayTransport::error() const noexcept
{
    return error_;
}

} // namespace xreal::virtual_display
