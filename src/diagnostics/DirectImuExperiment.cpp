#include "diagnostics/DirectImuExperiment.hpp"

#include "sensors/XrealHidConnection.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <span>
#include <string>
#include <vector>

namespace xreal::diagnostics
{
namespace
{

constexpr std::uint16_t supportedVendorId = 0x3318;
constexpr std::uint16_t supportedProductId = 0x0426;
constexpr int directImuInterfaceNumber = 2;
constexpr std::chrono::milliseconds readTimeout{100};
constexpr std::size_t readBufferSize = 64;
constexpr std::size_t maximumPacketDumps = 20;

const sensors::XrealDeviceInfo* findDirectImuInterface(
    const std::vector<sensors::XrealDeviceInfo>& devices)
{
    for (const auto& device : devices)
    {
        if (device.vendorId == supportedVendorId
            && device.productId == supportedProductId
            && device.interfaceNumber == directImuInterfaceNumber)
        {
            return &device;
        }
    }

    return nullptr;
}

void printPacketLengths(const std::map<std::size_t, std::size_t>& lengthCounts)
{
    if (lengthCounts.empty())
    {
        std::cout << " none";
        return;
    }

    for (const auto& [length, count] : lengthCounts)
    {
        std::cout << ' ' << length << " bytes (" << count << ')';
    }
}

} // namespace

int runDirectImuExperiment(
    const std::vector<sensors::XrealDeviceInfo>& devices,
    const ProbeOptions& options)
{
    if (options.interfaceNumber.has_value())
    {
        std::cerr << "Error: --interface cannot be combined with --enable-imu-direct;"
                     " direct mode always targets exact interface 2.\n";
        return 2;
    }

    const auto* device = findDirectImuInterface(devices);
    if (device == nullptr)
    {
        std::cerr << "Direct activation rejected: exact VID 0x3318/PID 0x0426 interface 2 was not found.\n";
        return 1;
    }

    std::cout << "Direct IMU experiment on exact VID 0x3318/PID 0x0426 interface 2.\n";
    if (options.verbose)
    {
        std::wcout << L"  Serial number: " << device->serialNumber << L'\n';
        std::cout << "  Device path:   " << device->path << '\n';
    }

    sensors::XrealHidConnection connection(device->path);
    if (!connection.isOpen())
    {
        std::wcerr << L"hid_open_path failed for interface 2: " << connection.openError() << L'\n';
        return 1;
    }

    const auto activationReport = buildDirectImuActivationReport();
    std::cout << "WARNING: sending one independent 10-byte direct activation report to interface 2.\n"
              << "  Report: " << formatHexPrefix(activationReport, activationReport.size()) << '\n';

    const auto writeResult = connection.writeRawReport(activationReport);
    std::cout << "  hid_write returned: " << writeResult.returnValue << '\n';
    if (!writeResult.success)
    {
        std::wcerr << L"  hid_write error on interface 2: " << writeResult.errorMessage << L'\n';
        return 1;
    }

    std::cout << "Reading the same interface 2 handle for " << options.duration.count()
              << " seconds. Move the glasses gently during the capture to test payload changes.\n";

    std::array<std::byte, readBufferSize> buffer{};
    PacketStatistics statistics;
    std::size_t dumpedPacketCount{};
    bool readFailed{};
    const auto start = std::chrono::steady_clock::now();
    const auto deadline = start + options.duration;

    while (std::chrono::steady_clock::now() < deadline)
    {
        const auto result = connection.readTimeout(buffer, readTimeout);

        if (result.status == sensors::HidReadStatus::timeout)
        {
            continue;
        }

        if (result.status == sensors::HidReadStatus::error)
        {
            std::wcerr << L"hid_read_timeout error on interface 2: " << result.errorMessage << L'\n';
            readFailed = true;
            break;
        }

        const std::span<const std::byte> packet(buffer.data(), result.length);
        statistics.record(packet);

        if (dumpedPacketCount < maximumPacketDumps)
        {
            ++dumpedPacketCount;
            std::cout << "  Packet " << statistics.packetCount() << " (" << result.length << " bytes)\n"
                      << "    First four bytes: " << formatHexPrefix(packet, 4) << '\n'
                      << "    Raw: " << formatHexPrefix(packet, packet.size()) << '\n';
        }
    }

    const auto elapsed = std::chrono::steady_clock::now() - start;
    std::cout << "\nDirect interface 2 summary\n"
              << "  Packets received: " << statistics.packetCount() << '\n'
              << "  Packet lengths:";
    printPacketLengths(statistics.lengthCounts());
    std::cout << '\n'
              << "  Approx. packets/s: " << std::fixed << std::setprecision(2)
              << statistics.packetsPerSecond(elapsed) << '\n'
              << "  Unique payloads: " << statistics.uniquePayloadCount() << '\n'
              << "  Payload contents changed during capture: "
              << (statistics.uniquePayloadCount() > 1 ? "yes" : "no") << '\n'
              << "  64-byte reports observed: "
              << (statistics.lengthCounts().contains(64) ? "yes" : "no") << '\n'
              << "  63-byte reports observed: "
              << (statistics.lengthCounts().contains(63) ? "yes" : "no") << '\n';

    return readFailed ? 1 : 0;
}

} // namespace xreal::diagnostics
