#include "platform/windows/HidDeviceCapabilities.hpp"

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <Windows.h>
#include <hidsdi.h>

#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

namespace xreal::platform::windows
{
namespace
{

class UniqueHandle
{
public:
    explicit UniqueHandle(HANDLE handle) noexcept
        : handle_(handle)
    {
    }

    ~UniqueHandle()
    {
        if (handle_ != INVALID_HANDLE_VALUE)
        {
            CloseHandle(handle_);
        }
    }

    UniqueHandle(const UniqueHandle&) = delete;
    UniqueHandle& operator=(const UniqueHandle&) = delete;

    [[nodiscard]] HANDLE get() const noexcept
    {
        return handle_;
    }

private:
    HANDLE handle_;
};

struct PreparsedDataDeleter
{
    void operator()(_HIDP_PREPARSED_DATA* data) const noexcept
    {
        if (data != nullptr)
        {
            HidD_FreePreparsedData(data);
        }
    }
};

std::string windowsError(std::string_view operation, DWORD errorCode)
{
    char* systemMessage{};
    const DWORD messageLength = FormatMessageA(
        FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
        nullptr,
        errorCode,
        0,
        reinterpret_cast<char*>(&systemMessage),
        0,
        nullptr);

    std::ostringstream message;
    message << operation << " failed with Win32 error " << errorCode;
    if (messageLength != 0 && systemMessage != nullptr)
    {
        message << ": " << std::string(systemMessage, messageLength);
    }

    if (systemMessage != nullptr)
    {
        LocalFree(systemMessage);
    }

    return message.str();
}

} // namespace

HidDeviceCapabilities queryHidDeviceCapabilities(std::string_view path)
{
    const std::string nullTerminatedPath(path);
    UniqueHandle handle(CreateFileA(
        nullTerminatedPath.c_str(),
        0,
        FILE_SHARE_READ | FILE_SHARE_WRITE,
        nullptr,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        nullptr));

    if (handle.get() == INVALID_HANDLE_VALUE)
    {
        return {false, 0, 0, 0, windowsError("CreateFileA for HID capabilities", GetLastError())};
    }

    PHIDP_PREPARSED_DATA rawPreparsedData{};
    if (HidD_GetPreparsedData(handle.get(), &rawPreparsedData) == FALSE)
    {
        return {false, 0, 0, 0, windowsError("HidD_GetPreparsedData", GetLastError())};
    }

    const std::unique_ptr<_HIDP_PREPARSED_DATA, PreparsedDataDeleter> preparsedData(rawPreparsedData);
    HIDP_CAPS capabilities{};
    const NTSTATUS status = HidP_GetCaps(preparsedData.get(), &capabilities);

    if (status != HIDP_STATUS_SUCCESS)
    {
        std::ostringstream message;
        message << "HidP_GetCaps failed with HID parser status 0x" << std::hex << std::uppercase
                << static_cast<unsigned long>(status) << '.';
        return {false, 0, 0, 0, message.str()};
    }

    return {
        true,
        capabilities.InputReportByteLength,
        capabilities.OutputReportByteLength,
        capabilities.FeatureReportByteLength,
        {},
    };
}

} // namespace xreal::platform::windows
