#include "diagnostics/InterfaceProbeSupport.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <string_view>

namespace
{

int failureCount{};

void expect(bool condition, std::string_view description)
{
    if (!condition)
    {
        ++failureCount;
        std::cerr << "FAILED: " << description << '\n';
    }
}

void testDefaultOptions()
{
    const auto result = xreal::diagnostics::parseProbeOptions({});
    expect(result.options.has_value(), "default arguments parse");
    expect(result.options->duration == std::chrono::seconds(5), "default duration is five seconds");
    expect(!result.options->interfaceNumber.has_value(), "all interfaces selected by default");
    expect(!result.options->verbose, "verbose mode is disabled by default");
    expect(!result.options->enableImu, "IMU activation is disabled by default");
    expect(!result.options->enableImuDirect, "direct IMU activation is disabled by default");
    expect(result.options->prelistenDuration == std::chrono::milliseconds(500),
           "default pre-listen duration is 500 milliseconds");
}

void testExplicitOptions()
{
    constexpr std::array arguments{std::string_view{"--interface"}, std::string_view{"2"},
                                   std::string_view{"--duration"}, std::string_view{"10"},
                                   std::string_view{"--prelisten-ms"}, std::string_view{"1000"},
                                   std::string_view{"--verbose"}, std::string_view{"--enable-imu"}};
    const auto result = xreal::diagnostics::parseProbeOptions(arguments);
    expect(result.options.has_value(), "explicit arguments parse");
    expect(result.options->interfaceNumber == 2, "interface number is parsed");
    expect(result.options->duration == std::chrono::seconds(10), "duration is parsed");
    expect(result.options->verbose, "verbose mode is parsed");
    expect(result.options->enableImu, "explicit IMU activation is parsed");
    expect(result.options->prelistenDuration == std::chrono::milliseconds(1000),
           "pre-listen duration is parsed");
}

void testInvalidOptions()
{
    constexpr std::array zeroDuration{std::string_view{"--duration"}, std::string_view{"0"}};
    constexpr std::array missingValue{std::string_view{"--interface"}};
    constexpr std::array invalidPrelisten{std::string_view{"--prelisten-ms"}, std::string_view{"-1"}};
    constexpr std::array unknownOption{std::string_view{"--active"}};

    expect(!xreal::diagnostics::parseProbeOptions(zeroDuration).options.has_value(), "zero duration is rejected");
    expect(!xreal::diagnostics::parseProbeOptions(missingValue).options.has_value(), "missing value is rejected");
    expect(!xreal::diagnostics::parseProbeOptions(invalidPrelisten).options.has_value(),
           "negative pre-listen duration is rejected");
    expect(!xreal::diagnostics::parseProbeOptions(unknownOption).options.has_value(), "unknown option is rejected");
}

void testPacketStatistics()
{
    constexpr std::array first{std::byte{0x01}, std::byte{0x02}, std::byte{0x03}};
    constexpr std::array second{std::byte{0x01}, std::byte{0x02}, std::byte{0x04}};
    xreal::diagnostics::PacketStatistics statistics;

    statistics.record(first);
    statistics.record(first);
    statistics.record(second);

    expect(statistics.packetCount() == 3, "packet count includes duplicates");
    expect(statistics.uniquePayloadCount() == 2, "unique payloads are counted");
    expect(statistics.lengthCounts().at(3) == 3, "packet lengths are counted");
    expect(std::abs(statistics.packetsPerSecond(std::chrono::seconds(2)) - 1.5) < 0.001,
           "packet rate uses elapsed time");
    expect(xreal::diagnostics::formatHexPrefix(first) == "01 02 03", "packet bytes use hexadecimal format");
    expect(xreal::diagnostics::formatHexPrefix(first, 2) == "01 02", "hexadecimal output respects its limit");
}

void testDirectImuOption()
{
    constexpr std::array arguments{std::string_view{"--enable-imu-direct"}};
    const auto result = xreal::diagnostics::parseProbeOptions(arguments);

    expect(result.options.has_value(), "direct IMU option parses");
    expect(result.options->enableImuDirect, "direct IMU activation is explicitly enabled");
    expect(!result.options->enableImu, "direct mode does not enable the MCU experiment");
}

void testImuActivationPacketAndCrcRange()
{
    constexpr std::uint32_t requestId = 0x0437;
    const auto report = xreal::diagnostics::buildImuActivationReport(requestId);

    expect(report[0] == std::byte{0xFD}, "activation packet header is 0xFD");
    expect(report[5] == std::byte{0x12} && report[6] == std::byte{0x00}, "protocol length is 18");
    expect(report[7] == std::byte{0x37} && report[8] == std::byte{0x04}, "request ID is little-endian");
    expect(report[15] == std::byte{0x19} && report[16] == std::byte{0x00}, "command ID is little-endian");
    expect(report[22] == std::byte{0x01}, "activation data enables the IMU");

    const std::uint32_t storedChecksum = std::to_integer<std::uint32_t>(report[1])
        | (std::to_integer<std::uint32_t>(report[2]) << 8U)
        | (std::to_integer<std::uint32_t>(report[3]) << 16U)
        | (std::to_integer<std::uint32_t>(report[4]) << 24U);
    expect(storedChecksum == xreal::diagnostics::crc32(std::span(report).subspan(5, 18)),
           "CRC covers exactly bytes 5 through 22");

    auto outsideRangeMutation = report;
    outsideRangeMutation[23] = std::byte{0xFF};
    expect(storedChecksum == xreal::diagnostics::crc32(std::span(outsideRangeMutation).subspan(5, 18)),
           "bytes after the declared range do not affect CRC");

    auto insideRangeMutation = report;
    insideRangeMutation[22] = std::byte{0x00};
    expect(storedChecksum != xreal::diagnostics::crc32(std::span(insideRangeMutation).subspan(5, 18)),
           "bytes inside the declared range affect CRC");

    constexpr std::array crcVector{std::byte{'1'}, std::byte{'2'}, std::byte{'3'}, std::byte{'4'},
                                   std::byte{'5'}, std::byte{'6'}, std::byte{'7'}, std::byte{'8'},
                                   std::byte{'9'}};
    expect(xreal::diagnostics::crc32(crcVector) == 0xCBF43926U, "CRC32 matches the standard check vector");
}

void testDirectImuActivationPacket()
{
    constexpr std::array expected{
        std::byte{0x00}, std::byte{0xAA}, std::byte{0xC5}, std::byte{0xD1}, std::byte{0x21},
        std::byte{0x42}, std::byte{0x04}, std::byte{0x00}, std::byte{0x19}, std::byte{0x01},
    };

    expect(xreal::diagnostics::buildDirectImuActivationReport() == expected,
           "direct IMU activation report matches the exact AirAPI_Windows sequence");
}

} // namespace

int main()
{
    testDefaultOptions();
    testExplicitOptions();
    testInvalidOptions();
    testDirectImuOption();
    testPacketStatistics();
    testImuActivationPacketAndCrcRange();
    testDirectImuActivationPacket();

    if (failureCount != 0)
    {
        std::cerr << failureCount << " test(s) failed.\n";
        return 1;
    }

    std::cout << "All interface probe support tests passed.\n";
    return 0;
}
