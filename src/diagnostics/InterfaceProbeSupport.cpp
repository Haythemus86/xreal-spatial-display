#include "diagnostics/InterfaceProbeSupport.hpp"

#include <algorithm>
#include <charconv>
#include <iomanip>
#include <limits>
#include <sstream>

namespace xreal::diagnostics
{
namespace
{

std::optional<int> parseNonNegativeInteger(std::string_view value)
{
    int result{};
    const auto conversion = std::from_chars(value.data(), value.data() + value.size(), result);

    if (conversion.ec != std::errc{} || conversion.ptr != value.data() + value.size() || result < 0)
    {
        return std::nullopt;
    }

    return result;
}

} // namespace

ProbeOptionsResult parseProbeOptions(std::span<const std::string_view> arguments)
{
    ProbeOptions options;

    for (std::size_t index = 0; index < arguments.size(); ++index)
    {
        const std::string_view argument = arguments[index];

        if (argument == "--verbose")
        {
            options.verbose = true;
            continue;
        }

        if (argument == "--help" || argument == "-h")
        {
            return {options, {}, true};
        }

        if (argument != "--duration" && argument != "--interface")
        {
            return {std::nullopt, "Unknown option: " + std::string(argument), false};
        }

        if (index + 1 >= arguments.size())
        {
            return {std::nullopt, "Missing value for " + std::string(argument), false};
        }

        const std::string_view value = arguments[++index];
        const auto parsedValue = parseNonNegativeInteger(value);

        if (!parsedValue.has_value())
        {
            return {std::nullopt, "Invalid value for " + std::string(argument) + ": " + std::string(value), false};
        }

        if (argument == "--duration")
        {
            if (*parsedValue == 0)
            {
                return {std::nullopt, "--duration must be greater than zero.", false};
            }

            options.duration = std::chrono::seconds(*parsedValue);
        }
        else
        {
            options.interfaceNumber = *parsedValue;
        }
    }

    return {options, {}, false};
}

std::string formatHexPrefix(std::span<const std::byte> packet, std::size_t maximumBytes)
{
    std::ostringstream output;
    output << std::hex << std::uppercase << std::setfill('0');

    const std::size_t byteCount = std::min(packet.size(), maximumBytes);
    for (std::size_t index = 0; index < byteCount; ++index)
    {
        if (index != 0)
        {
            output << ' ';
        }

        output << std::setw(2) << std::to_integer<unsigned int>(packet[index]);
    }

    return output.str();
}

void PacketStatistics::record(std::span<const std::byte> packet)
{
    ++packetCount_;
    ++lengthCounts_[packet.size()];
    uniquePayloads_.emplace(packet.begin(), packet.end());
}

std::size_t PacketStatistics::packetCount() const noexcept
{
    return packetCount_;
}

std::size_t PacketStatistics::uniquePayloadCount() const noexcept
{
    return uniquePayloads_.size();
}

const std::map<std::size_t, std::size_t>& PacketStatistics::lengthCounts() const noexcept
{
    return lengthCounts_;
}

double PacketStatistics::packetsPerSecond(std::chrono::steady_clock::duration elapsed) const noexcept
{
    const double seconds = std::chrono::duration<double>(elapsed).count();
    return seconds > 0.0 ? static_cast<double>(packetCount_) / seconds : 0.0;
}

} // namespace xreal::diagnostics
