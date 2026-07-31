#pragma once

#include <cstdint>

namespace xreal::platform::windows
{

struct ProcessMemoryUsage
{
    bool available{};
    std::uint64_t workingSetBytes{};
    std::uint64_t privateBytes{};
};

[[nodiscard]] ProcessMemoryUsage currentProcessMemoryUsage() noexcept;

} // namespace xreal::platform::windows
