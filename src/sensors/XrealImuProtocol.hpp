#pragma once

#include <array>
#include <cstddef>

namespace xreal::sensors
{

inline constexpr std::array<std::byte, 10> directImuActivationReport{
    std::byte{0x00}, std::byte{0xAA}, std::byte{0xC5}, std::byte{0xD1}, std::byte{0x21},
    std::byte{0x42}, std::byte{0x04}, std::byte{0x00}, std::byte{0x19}, std::byte{0x01},
};

} // namespace xreal::sensors
