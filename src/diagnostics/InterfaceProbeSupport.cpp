#include "diagnostics/InterfaceProbeSupport.hpp"

#include "sensors/XrealImuProtocol.hpp"

#include <algorithm>
#include <charconv>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace xreal::diagnostics
{
namespace
{

constexpr std::size_t checksumOffset = 1;
constexpr std::size_t checksumInputOffset = 5;
constexpr std::size_t lengthOffset = 5;
constexpr std::size_t requestIdOffset = 7;
constexpr std::size_t commandIdOffset = 15;
constexpr std::size_t dataOffset = 22;
constexpr std::uint16_t imuActivationCommandId = 0x0019;
constexpr std::byte packetHeader{0xFD};
constexpr std::byte enableImuValue{0x01};
constexpr std::byte ackValue{0x04};

void writeLittleEndian16(std::span<std::byte> destination, std::uint16_t value)
{
    destination[0] = static_cast<std::byte>(value & 0xFFU);
    destination[1] = static_cast<std::byte>((value >> 8U) & 0xFFU);
}

void writeLittleEndian32(std::span<std::byte> destination, std::uint32_t value)
{
    for (std::size_t index = 0; index < sizeof(value); ++index)
    {
        destination[index] = static_cast<std::byte>((value >> (index * 8U)) & 0xFFU);
    }
}

std::uint16_t readLittleEndian16(std::span<const std::byte> source)
{
    return static_cast<std::uint16_t>(std::to_integer<std::uint16_t>(source[0])
        | (std::to_integer<std::uint16_t>(source[1]) << 8U));
}

std::uint32_t readLittleEndian32(std::span<const std::byte> source)
{
    std::uint32_t value{};

    for (std::size_t index = 0; index < sizeof(value); ++index)
    {
        value |= std::to_integer<std::uint32_t>(source[index]) << (index * 8U);
    }

    return value;
}

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

        if (argument == "--enable-imu")
        {
            options.enableImu = true;
            continue;
        }

        if (argument == "--enable-imu-direct")
        {
            options.enableImuDirect = true;
            continue;
        }

        if (argument == "--help" || argument == "-h")
        {
            return {options, {}, true};
        }

        if (argument != "--duration" && argument != "--interface" && argument != "--prelisten-ms")
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
        else if (argument == "--interface")
        {
            options.interfaceNumber = *parsedValue;
        }
        else
        {
            options.prelistenDuration = std::chrono::milliseconds(*parsedValue);
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

std::uint32_t crc32(std::span<const std::byte> bytes) noexcept
{
    std::uint32_t checksum = 0xFFFFFFFFU;

    for (const std::byte byte : bytes)
    {
        checksum ^= std::to_integer<std::uint8_t>(byte);

        for (int bit = 0; bit < 8; ++bit)
        {
            const std::uint32_t polynomial = (checksum & 1U) != 0U ? 0xEDB88320U : 0U;
            checksum = (checksum >> 1U) ^ polynomial;
        }
    }

    return checksum ^ 0xFFFFFFFFU;
}

std::array<std::byte, 64> buildImuActivationReport(std::uint32_t requestId)
{
    if (requestId == 0)
    {
        throw std::invalid_argument("The IMU activation request ID must be non-zero.");
    }

    constexpr std::uint16_t dataLength = 1;
    constexpr std::uint16_t protocolLength = dataLength + 17;
    std::array<std::byte, 64> report{};

    report[0] = packetHeader;
    writeLittleEndian16(std::span(report).subspan(lengthOffset, 2), protocolLength);
    writeLittleEndian32(std::span(report).subspan(requestIdOffset, 4), requestId);
    writeLittleEndian16(std::span(report).subspan(commandIdOffset, 2), imuActivationCommandId);
    report[dataOffset] = enableImuValue;

    const std::uint32_t checksum = crc32(
        std::span<const std::byte>(report).subspan(checksumInputOffset, protocolLength));
    writeLittleEndian32(std::span(report).subspan(checksumOffset, 4), checksum);
    return report;
}

std::array<std::byte, 10> buildDirectImuActivationReport() noexcept
{
    return sensors::directImuActivationReport;
}

bool containsLittleEndianValue(std::span<const std::byte> packet, std::uint32_t value) noexcept
{
    std::array<std::byte, sizeof(value)> encodedValue{};
    writeLittleEndian32(encodedValue, value);

    return std::search(packet.begin(), packet.end(), encodedValue.begin(), encodedValue.end()) != packet.end();
}

bool isCompatibleImuActivationAck(std::span<const std::byte> packet, std::uint32_t requestId) noexcept
{
    if (packet.size() <= dataOffset || packet[0] != packetHeader)
    {
        return false;
    }

    return readLittleEndian32(packet.subspan(requestIdOffset, 4)) == requestId
        && readLittleEndian16(packet.subspan(commandIdOffset, 2)) == imuActivationCommandId
        && packet[dataOffset] == ackValue;
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
