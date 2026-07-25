#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace xreal::diagnostics
{

struct ProbeOptions
{
    std::chrono::seconds duration{5};
    std::optional<int> interfaceNumber;
    bool verbose{};
    bool enableImu{};
    bool enableImuDirect{};
    std::chrono::milliseconds prelistenDuration{500};
};

struct ProbeOptionsResult
{
    std::optional<ProbeOptions> options;
    std::string errorMessage;
    bool showHelp{};
};

[[nodiscard]] ProbeOptionsResult parseProbeOptions(std::span<const std::string_view> arguments);
[[nodiscard]] std::string formatHexPrefix(std::span<const std::byte> packet, std::size_t maximumBytes = 32);
[[nodiscard]] std::uint32_t crc32(std::span<const std::byte> bytes) noexcept;
[[nodiscard]] std::array<std::byte, 64> buildImuActivationReport(std::uint32_t requestId);
[[nodiscard]] std::array<std::byte, 10> buildDirectImuActivationReport() noexcept;
[[nodiscard]] bool containsLittleEndianValue(std::span<const std::byte> packet, std::uint32_t value) noexcept;
[[nodiscard]] bool isCompatibleImuActivationAck(
    std::span<const std::byte> packet,
    std::uint32_t requestId) noexcept;

class PacketStatistics
{
public:
    void record(std::span<const std::byte> packet);

    [[nodiscard]] std::size_t packetCount() const noexcept;
    [[nodiscard]] std::size_t uniquePayloadCount() const noexcept;
    [[nodiscard]] const std::map<std::size_t, std::size_t>& lengthCounts() const noexcept;
    [[nodiscard]] double packetsPerSecond(std::chrono::steady_clock::duration elapsed) const noexcept;

private:
    std::size_t packetCount_{};
    std::map<std::size_t, std::size_t> lengthCounts_;
    std::set<std::vector<std::byte>> uniquePayloads_;
};

} // namespace xreal::diagnostics
