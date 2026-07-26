#pragma once

#include <cstdint>
#include <limits>
#include <optional>

namespace xreal::sensors
{

[[nodiscard]] constexpr std::optional<std::uint64_t> forwardDeviceTimestampDelta(
    std::uint64_t current,
    std::uint64_t previous) noexcept
{
    if (current > previous)
    {
        return current - previous;
    }
    if (current == previous)
    {
        return std::nullopt;
    }

    constexpr std::uint64_t lowerWrapBoundary = std::numeric_limits<std::uint64_t>::max() / 4U;
    constexpr std::uint64_t upperWrapBoundary = lowerWrapBoundary * 3U;
    if (previous >= upperWrapBoundary && current <= lowerWrapBoundary)
    {
        return (std::numeric_limits<std::uint64_t>::max() - previous) + 1U + current;
    }
    return std::nullopt;
}

} // namespace xreal::sensors
