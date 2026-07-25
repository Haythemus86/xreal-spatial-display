#pragma once

#include <chrono>
#include <cstddef>
#include <memory>
#include <span>
#include <string>

struct hid_device_;

namespace xreal::sensors
{

enum class HidReadStatus
{
    packet,
    timeout,
    error,
};

struct HidReadResult
{
    HidReadStatus status{HidReadStatus::error};
    std::size_t length{};
    std::wstring errorMessage;
};

class XrealHidConnection
{
public:
    explicit XrealHidConnection(const std::string& path);
    ~XrealHidConnection() = default;

    XrealHidConnection(const XrealHidConnection&) = delete;
    XrealHidConnection& operator=(const XrealHidConnection&) = delete;
    XrealHidConnection(XrealHidConnection&&) noexcept = default;
    XrealHidConnection& operator=(XrealHidConnection&&) noexcept = default;

    [[nodiscard]] bool isOpen() const noexcept;
    [[nodiscard]] const std::wstring& openError() const noexcept;
    [[nodiscard]] HidReadResult readTimeout(
        std::span<std::byte> buffer,
        std::chrono::milliseconds timeout) const;

private:
    struct HandleDeleter
    {
        void operator()(hid_device_* handle) const noexcept;
    };

    std::unique_ptr<hid_device_, HandleDeleter> handle_;
    std::wstring openError_;
};

} // namespace xreal::sensors
