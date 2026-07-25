#include "diagnostics/InterfaceProbeSupport.hpp"
#include "sensors/XrealDevice.hpp"
#include "sensors/XrealHidConnection.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <exception>
#include <iomanip>
#include <iostream>
#include <string_view>
#include <vector>

namespace
{

constexpr std::chrono::milliseconds readTimeout{100};
constexpr std::size_t maximumPacketDumps = 5;
constexpr std::size_t readBufferSize = 4096;

void printUsage()
{
    std::cout << "Usage: xreal-interface-probe [--duration <seconds>] [--interface <number>] [--verbose]\n";
}

void printDeviceIdentity(const xreal::sensors::XrealDeviceInfo& device, bool verbose)
{
    std::cout << "\nInterface " << device.interfaceNumber << '\n'
              << "  Vendor ID:  0x" << std::hex << std::uppercase << std::setw(4)
              << std::setfill('0') << device.vendorId << '\n'
              << "  Product ID: 0x" << std::setw(4) << device.productId << '\n'
              << std::dec << std::setfill(' ');

    if (verbose)
    {
        std::wcout << L"  Serial number: " << device.serialNumber << L'\n';
        std::cout << "  Device path:   " << device.path << '\n';
    }
}

void printSummary(
    const xreal::diagnostics::PacketStatistics& statistics,
    std::chrono::steady_clock::duration elapsed)
{
    std::cout << "  Packets received: " << statistics.packetCount() << '\n'
              << "  Approx. packets/s: " << std::fixed << std::setprecision(2)
              << statistics.packetsPerSecond(elapsed) << '\n'
              << "  Unique payloads: " << statistics.uniquePayloadCount() << '\n'
              << "  Packet lengths:";

    if (statistics.lengthCounts().empty())
    {
        std::cout << " none";
    }
    else
    {
        for (const auto& [length, count] : statistics.lengthCounts())
        {
            std::cout << ' ' << length << " bytes (" << count << ')';
        }
    }

    std::cout << '\n';
}

void probeInterface(const xreal::sensors::XrealDeviceInfo& device, const xreal::diagnostics::ProbeOptions& options)
{
    printDeviceIdentity(device, options.verbose);

    xreal::sensors::XrealHidConnection connection(device.path);
    if (!connection.isOpen())
    {
        std::cout << "  Opened: no\n";
        std::wcerr << L"  HIDAPI error: " << connection.openError() << L'\n';
        return;
    }

    std::cout << "  Opened: yes\n"
              << "  Passive read duration: " << options.duration.count() << " seconds\n";

    std::array<std::byte, readBufferSize> buffer{};
    xreal::diagnostics::PacketStatistics statistics;
    std::size_t dumpedPacketCount{};
    const auto start = std::chrono::steady_clock::now();
    const auto deadline = start + options.duration;

    while (std::chrono::steady_clock::now() < deadline)
    {
        const auto result = connection.readTimeout(buffer, readTimeout);

        if (result.status == xreal::sensors::HidReadStatus::timeout)
        {
            continue;
        }

        if (result.status == xreal::sensors::HidReadStatus::error)
        {
            std::wcerr << L"  Read failed on interface " << device.interfaceNumber
                       << L": " << result.errorMessage << L'\n';
            break;
        }

        const std::span<const std::byte> packet(buffer.data(), result.length);
        statistics.record(packet);

        if (dumpedPacketCount < maximumPacketDumps)
        {
            ++dumpedPacketCount;
            std::cout << "  Packet " << statistics.packetCount() << " (" << result.length
                      << " bytes): " << xreal::diagnostics::formatHexPrefix(packet) << '\n';
        }
    }

    printSummary(statistics, std::chrono::steady_clock::now() - start);
}

} // namespace

int main(int argc, char* argv[])
{
    std::vector<std::string_view> arguments;
    arguments.reserve(static_cast<std::size_t>(argc > 0 ? argc - 1 : 0));

    for (int index = 1; index < argc; ++index)
    {
        arguments.emplace_back(argv[index]);
    }

    const auto optionResult = xreal::diagnostics::parseProbeOptions(arguments);
    if (optionResult.showHelp)
    {
        printUsage();
        return 0;
    }

    if (!optionResult.options.has_value())
    {
        std::cerr << "Error: " << optionResult.errorMessage << '\n';
        printUsage();
        return 2;
    }

    try
    {
        const xreal::sensors::XrealDevice xrealDevice;
        const auto devices = xrealDevice.enumerate();
        std::size_t selectedDeviceCount{};

        for (const auto& device : devices)
        {
            if (optionResult.options->interfaceNumber.has_value()
                && device.interfaceNumber != *optionResult.options->interfaceNumber)
            {
                continue;
            }

            ++selectedDeviceCount;
            probeInterface(device, *optionResult.options);
        }

        if (selectedDeviceCount == 0)
        {
            std::cerr << "No matching XREAL HID interfaces found.\n";
            return 1;
        }

        return 0;
    }
    catch (const std::exception& exception)
    {
        std::cerr << "Startup failed while initializing or enumerating HID interfaces: "
                  << exception.what() << '\n';
        return 1;
    }
}
