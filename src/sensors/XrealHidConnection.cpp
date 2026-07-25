#include "sensors/XrealHidConnection.hpp"

#include <hidapi.h>

#include <limits>
#include <vector>

namespace xreal::sensors
{
namespace
{

std::wstring hidError(hid_device* handle)
{
    const wchar_t* message = hid_error(handle);
    return message != nullptr ? message : L"No HIDAPI error detail available.";
}

} // namespace

void XrealHidConnection::HandleDeleter::operator()(hid_device_* handle) const noexcept
{
    if (handle != nullptr)
    {
        hid_close(handle);
    }
}

XrealHidConnection::XrealHidConnection(const std::string& path)
    : handle_(hid_open_path(path.c_str()))
{
    if (handle_ == nullptr)
    {
        openError_ = hidError(nullptr);
    }
}

bool XrealHidConnection::isOpen() const noexcept
{
    return handle_ != nullptr;
}

const std::wstring& XrealHidConnection::openError() const noexcept
{
    return openError_;
}

HidReadResult XrealHidConnection::readTimeout(
    std::span<std::byte> buffer,
    std::chrono::milliseconds timeout) const
{
    if (handle_ == nullptr)
    {
        return {HidReadStatus::error, 0, L"The HID interface is not open."};
    }

    if (buffer.size() > static_cast<std::size_t>(std::numeric_limits<int>::max()))
    {
        return {HidReadStatus::error, 0, L"The read buffer is too large for HIDAPI."};
    }

    const int readLength = hid_read_timeout(
        handle_.get(),
        reinterpret_cast<unsigned char*>(buffer.data()),
        buffer.size(),
        static_cast<int>(timeout.count()));

    if (readLength > 0)
    {
        return {HidReadStatus::packet, static_cast<std::size_t>(readLength), {}};
    }

    if (readLength == 0)
    {
        return {HidReadStatus::timeout, 0, {}};
    }

    return {HidReadStatus::error, 0, hidError(handle_.get())};
}

HidWriteResult XrealHidConnection::writeOutputReport(
    std::uint8_t reportId,
    std::span<const std::byte> reportPayload) const
{
    if (handle_ == nullptr)
    {
        return {false, -1, L"The HID interface is not open."};
    }

    std::vector<unsigned char> transportBuffer(reportPayload.size() + 1);
    transportBuffer[0] = reportId;

    for (std::size_t index = 0; index < reportPayload.size(); ++index)
    {
        transportBuffer[index + 1] = std::to_integer<unsigned char>(reportPayload[index]);
    }

    const int writtenLength = hid_write(handle_.get(), transportBuffer.data(), transportBuffer.size());
    if (writtenLength < 0)
    {
        return {false, writtenLength, hidError(handle_.get())};
    }

    return {true, writtenLength, {}};
}

HidWriteResult XrealHidConnection::writeRawReport(std::span<const std::byte> report) const
{
    if (handle_ == nullptr)
    {
        return {false, -1, L"The HID interface is not open."};
    }

    std::vector<unsigned char> transportBuffer;
    transportBuffer.reserve(report.size());

    for (const std::byte byte : report)
    {
        transportBuffer.push_back(std::to_integer<unsigned char>(byte));
    }

    const int writtenLength = hid_write(handle_.get(), transportBuffer.data(), transportBuffer.size());
    if (writtenLength < 0)
    {
        return {false, writtenLength, hidError(handle_.get())};
    }

    return {true, writtenLength, {}};
}

} // namespace xreal::sensors
