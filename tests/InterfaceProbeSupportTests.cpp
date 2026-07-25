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
}

void testExplicitOptions()
{
    constexpr std::array arguments{std::string_view{"--interface"}, std::string_view{"2"},
                                   std::string_view{"--duration"}, std::string_view{"10"},
                                   std::string_view{"--verbose"}};
    const auto result = xreal::diagnostics::parseProbeOptions(arguments);
    expect(result.options.has_value(), "explicit arguments parse");
    expect(result.options->interfaceNumber == 2, "interface number is parsed");
    expect(result.options->duration == std::chrono::seconds(10), "duration is parsed");
    expect(result.options->verbose, "verbose mode is parsed");
}

void testInvalidOptions()
{
    constexpr std::array zeroDuration{std::string_view{"--duration"}, std::string_view{"0"}};
    constexpr std::array missingValue{std::string_view{"--interface"}};
    constexpr std::array unknownOption{std::string_view{"--active"}};

    expect(!xreal::diagnostics::parseProbeOptions(zeroDuration).options.has_value(), "zero duration is rejected");
    expect(!xreal::diagnostics::parseProbeOptions(missingValue).options.has_value(), "missing value is rejected");
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

} // namespace

int main()
{
    testDefaultOptions();
    testExplicitOptions();
    testInvalidOptions();
    testPacketStatistics();

    if (failureCount != 0)
    {
        std::cerr << failureCount << " test(s) failed.\n";
        return 1;
    }

    std::cout << "All interface probe support tests passed.\n";
    return 0;
}
