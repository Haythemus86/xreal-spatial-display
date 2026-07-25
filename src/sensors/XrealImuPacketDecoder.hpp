#pragma once

#include "sensors/ImuSample.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace xreal::sensors
{

enum class XrealImuPacketKind
{
    sensor,
    activationResponse,
    invalid,
};

class XrealImuPacketDecoder
{
public:
    static constexpr std::size_t reportSize = 64;

    [[nodiscard]] XrealImuPacketKind classify(std::span<const std::uint8_t> report) const noexcept;

    [[nodiscard]] std::optional<ImuSample> decode(
        std::span<const std::uint8_t, reportSize> report,
        std::chrono::steady_clock::time_point hostReceiveTimestamp) const noexcept;

    // Dynamic-extent boundary for callers that have not yet validated HID report size.
    [[nodiscard]] std::optional<ImuSample> decode(
        std::span<const std::uint8_t> report,
        std::chrono::steady_clock::time_point hostReceiveTimestamp) const noexcept;
};

} // namespace xreal::sensors
