#pragma once

#include "sensors/ImuSample.hpp"
#include "sensors/XrealDevice.hpp"
#include "sensors/XrealHidConnection.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <thread>
#include <vector>

namespace xreal::sensors
{

struct XrealImuStreamStatistics
{
    std::uint64_t received{};
    std::uint64_t dropped{};
    std::uint64_t invalid{};
    std::uint64_t outOfSequence{};
    std::uint64_t activationResponses{};
};

class XrealImuStream
{
public:
    static constexpr std::uint16_t vendorId = 0x3318;
    static constexpr std::uint16_t productId = 0x0426;
    static constexpr int interfaceNumber = 2;
    static constexpr std::size_t reportSize = 64;

    using ReportCallback = std::function<void(
        std::span<const std::uint8_t, reportSize>,
        const ImuSample&)>;

    explicit XrealImuStream(XrealDeviceInfo deviceInfo);
    ~XrealImuStream();

    XrealImuStream(const XrealImuStream&) = delete;
    XrealImuStream& operator=(const XrealImuStream&) = delete;
    XrealImuStream(XrealImuStream&&) = delete;
    XrealImuStream& operator=(XrealImuStream&&) = delete;

    [[nodiscard]] static std::optional<XrealDeviceInfo> findInterface(
        const std::vector<XrealDeviceInfo>& devices);
    [[nodiscard]] bool start(ReportCallback callback);
    void stop() noexcept;

    [[nodiscard]] bool isRunning() const noexcept;
    [[nodiscard]] XrealImuStreamStatistics statistics() const noexcept;
    [[nodiscard]] std::wstring errorMessage() const;

private:
    void readLoop(std::stop_token stopToken) noexcept;
    void setError(std::wstring errorMessage);

    XrealDeviceInfo deviceInfo_;
    std::optional<XrealHidConnection> connection_;
    ReportCallback callback_;
    std::jthread worker_;
    std::atomic_bool running_{};
    std::atomic_uint64_t received_{};
    std::atomic_uint64_t dropped_{};
    std::atomic_uint64_t invalid_{};
    std::atomic_uint64_t outOfSequence_{};
    std::atomic_uint64_t activationResponses_{};
    mutable std::mutex errorMutex_;
    std::wstring errorMessage_;
};

} // namespace xreal::sensors
