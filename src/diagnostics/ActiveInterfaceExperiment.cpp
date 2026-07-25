#include "diagnostics/ActiveInterfaceExperiment.hpp"

#include "platform/windows/HidDeviceCapabilities.hpp"
#include "sensors/XrealHidConnection.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <span>
#include <string>
#include <string_view>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

namespace xreal::diagnostics
{
namespace
{

constexpr std::uint16_t supportedVendorId = 0x3318;
constexpr std::uint16_t supportedProductId = 0x0426;
constexpr int controlInterfaceNumber = 0;
constexpr std::array candidateInterfaceNumbers{1, 2, 8};
constexpr std::array reportedInterfaceNumbers{0, 1, 2, 8};
constexpr std::uint32_t activationRequestId = 0x0437;
constexpr std::chrono::milliseconds readTimeout{100};
constexpr std::chrono::seconds acknowledgementTimeout{2};
constexpr std::size_t readBufferSize = 2048;
constexpr std::size_t maximumPacketSamples = 10;
constexpr std::size_t maximumSampleBytes = 64;

struct PacketSample
{
    std::chrono::milliseconds timestamp;
    bool beforeActivation{};
    std::size_t length{};
    std::uint8_t firstByte{};
    std::string hexadecimalPrefix;
};

struct CaptureStatistics
{
    std::size_t packetsBeforeActivation{};
    std::size_t packetsAfterActivation{};
    std::map<std::size_t, std::size_t> lengthCounts;
    std::set<std::uint8_t> observedFirstBytes;
    std::vector<PacketSample> samples;
    std::wstring readError;
    std::chrono::steady_clock::time_point readerStarted;
    std::chrono::steady_clock::time_point readerStopped;

    void record(
        std::span<const std::byte> packet,
        bool beforeActivation,
        std::chrono::steady_clock::time_point experimentStart)
    {
        if (beforeActivation)
        {
            ++packetsBeforeActivation;
        }
        else
        {
            ++packetsAfterActivation;
        }

        ++lengthCounts[packet.size()];
        if (!packet.empty())
        {
            observedFirstBytes.insert(std::to_integer<std::uint8_t>(packet[0]));
        }

        if (samples.size() < maximumPacketSamples)
        {
            samples.push_back(PacketSample{
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - experimentStart),
                beforeActivation,
                packet.size(),
                packet.empty() ? std::uint8_t{0} : std::to_integer<std::uint8_t>(packet[0]),
                formatHexPrefix(packet, maximumSampleBytes),
            });
        }
    }

    [[nodiscard]] std::size_t packetCount() const noexcept
    {
        return packetsBeforeActivation + packetsAfterActivation;
    }

    [[nodiscard]] double packetsPerSecond() const noexcept
    {
        const double elapsed = std::chrono::duration<double>(readerStopped - readerStarted).count();
        return elapsed > 0.0 ? static_cast<double>(packetCount()) / elapsed : 0.0;
    }
};

class InterfaceSession
{
public:
    InterfaceSession(int interfaceNumber, const sensors::XrealDeviceInfo* device)
        : interfaceNumber_(interfaceNumber), device_(device)
    {
        if (device_ != nullptr)
        {
            capabilities_ = platform::windows::queryHidDeviceCapabilities(device_->path);
        }
    }

    ~InterfaceSession()
    {
        stopReader();
    }

    InterfaceSession(const InterfaceSession&) = delete;
    InterfaceSession& operator=(const InterfaceSession&) = delete;

    void open()
    {
        openAttempted_ = true;
        if (device_ == nullptr)
        {
            openError_ = L"Matching VID 0x3318/PID 0x0426 interface was not enumerated.";
            return;
        }

        connection_ = std::make_unique<sensors::XrealHidConnection>(device_->path);
        if (!connection_->isOpen())
        {
            openError_ = connection_->openError();
            connection_.reset();
        }
    }

    void startReader(
        std::chrono::steady_clock::time_point experimentStart,
        const std::atomic<bool>& activationStarted)
    {
        if (connection_ == nullptr || reader_.joinable())
        {
            return;
        }

        if (statistics_.readerStarted == std::chrono::steady_clock::time_point{})
        {
            statistics_.readerStarted = std::chrono::steady_clock::now();
        }
        reader_ = std::jthread([this, experimentStart, &activationStarted](std::stop_token stopToken) {
            std::array<std::byte, readBufferSize> buffer{};

            while (!stopToken.stop_requested())
            {
                const auto result = connection_->readTimeout(buffer, readTimeout);

                if (result.status == sensors::HidReadStatus::timeout)
                {
                    continue;
                }

                if (result.status == sensors::HidReadStatus::error)
                {
                    statistics_.readError = result.errorMessage;
                    break;
                }

                statistics_.record(
                    std::span<const std::byte>(buffer.data(), result.length),
                    !activationStarted.load(std::memory_order_acquire),
                    experimentStart);
            }

            statistics_.readerStopped = std::chrono::steady_clock::now();
        });
    }

    void stopReader()
    {
        if (reader_.joinable())
        {
            reader_.request_stop();
            reader_.join();
        }
    }

    void recordSynchronousPacket(
        std::span<const std::byte> packet,
        bool beforeActivation,
        std::chrono::steady_clock::time_point experimentStart)
    {
        statistics_.record(packet, beforeActivation, experimentStart);
    }

    void markSynchronousReaderStart()
    {
        statistics_.readerStarted = std::chrono::steady_clock::now();
    }

    void markSynchronousReaderStop()
    {
        statistics_.readerStopped = std::chrono::steady_clock::now();
    }

    [[nodiscard]] int interfaceNumber() const noexcept
    {
        return interfaceNumber_;
    }

    [[nodiscard]] const sensors::XrealDeviceInfo* device() const noexcept
    {
        return device_;
    }

    [[nodiscard]] sensors::XrealHidConnection* connection() const noexcept
    {
        return connection_.get();
    }

    [[nodiscard]] bool openedSuccessfully() const noexcept
    {
        return openAttempted_ && connection_ != nullptr;
    }

    [[nodiscard]] const std::wstring& openError() const noexcept
    {
        return openError_;
    }

    [[nodiscard]] const platform::windows::HidDeviceCapabilities& capabilities() const noexcept
    {
        return capabilities_;
    }

    [[nodiscard]] const CaptureStatistics& statistics() const noexcept
    {
        return statistics_;
    }

private:
    int interfaceNumber_{};
    const sensors::XrealDeviceInfo* device_{};
    platform::windows::HidDeviceCapabilities capabilities_;
    std::unique_ptr<sensors::XrealHidConnection> connection_;
    std::wstring openError_;
    bool openAttempted_{};
    CaptureStatistics statistics_;
    std::jthread reader_;
};

const sensors::XrealDeviceInfo* findSupportedInterface(
    const std::vector<sensors::XrealDeviceInfo>& devices,
    int interfaceNumber)
{
    for (const auto& device : devices)
    {
        if (device.vendorId == supportedVendorId
            && device.productId == supportedProductId
            && device.interfaceNumber == interfaceNumber)
        {
            return &device;
        }
    }

    return nullptr;
}

InterfaceSession& findSession(std::vector<std::unique_ptr<InterfaceSession>>& sessions, int interfaceNumber)
{
    for (const auto& session : sessions)
    {
        if (session->interfaceNumber() == interfaceNumber)
        {
            return *session;
        }
    }

    throw std::logic_error("Requested interface session was not created.");
}

void printLengths(const std::map<std::size_t, std::size_t>& lengthCounts)
{
    if (lengthCounts.empty())
    {
        std::cout << "none";
        return;
    }

    for (const auto& [length, count] : lengthCounts)
    {
        std::cout << length << " bytes (" << count << ") ";
    }
}

void printSessionSummary(const InterfaceSession& session, bool verbose)
{
    std::cout << "\nInterface " << session.interfaceNumber() << '\n'
              << "  Opened successfully: " << (session.openedSuccessfully() ? "yes" : "no") << '\n';

    if (session.device() != nullptr)
    {
        const auto& device = *session.device();
        std::cout << "  Usage page: 0x" << std::hex << std::uppercase << std::setw(4)
                  << std::setfill('0') << device.usagePage << '\n'
                  << "  Usage:      0x" << std::setw(4) << device.usage << '\n'
                  << std::dec << std::setfill(' ');

        if (verbose)
        {
            std::wcout << L"  Serial number: " << device.serialNumber << L'\n';
            std::cout << "  Device path:   " << device.path << '\n';
        }
    }
    else
    {
        std::cout << "  Usage page: unavailable\n  Usage: unavailable\n";
    }

    const auto& capabilities = session.capabilities();
    if (capabilities.available)
    {
        std::cout << "  Input report byte length:   " << capabilities.inputReportByteLength << '\n'
                  << "  Output report byte length:  " << capabilities.outputReportByteLength << '\n'
                  << "  Feature report byte length: " << capabilities.featureReportByteLength << '\n';
    }
    else
    {
        std::cout << "  Report lengths: unavailable";
        if (!capabilities.errorMessage.empty())
        {
            std::cout << " (" << capabilities.errorMessage << ')';
        }
        std::cout << '\n';
    }

    if (!session.openError().empty())
    {
        std::wcerr << L"  Open error: " << session.openError() << L'\n';
    }

    const auto& statistics = session.statistics();
    std::cout << "  Packets before activation: " << statistics.packetsBeforeActivation << '\n'
              << "  Packets after activation:  " << statistics.packetsAfterActivation << '\n'
              << "  Packet lengths: ";
    printLengths(statistics.lengthCounts);
    std::cout << '\n'
              << "  Approx. packets/s: " << std::fixed << std::setprecision(2)
              << statistics.packetsPerSecond() << '\n'
              << "  Observed first-byte/report IDs:";

    if (statistics.observedFirstBytes.empty())
    {
        std::cout << " none";
    }
    else
    {
        for (const std::uint8_t value : statistics.observedFirstBytes)
        {
            std::cout << " 0x" << std::hex << std::uppercase << std::setw(2)
                      << std::setfill('0') << static_cast<unsigned int>(value);
        }
        std::cout << std::dec << std::setfill(' ');
    }
    std::cout << '\n';

    for (std::size_t index = 0; index < statistics.samples.size(); ++index)
    {
        const auto& sample = statistics.samples[index];
        std::cout << "  Sample " << index + 1 << " at +" << sample.timestamp.count() << " ms ["
                  << (sample.beforeActivation ? "before" : "after") << "] (" << sample.length
                  << " bytes, first/report ID 0x" << std::hex << std::uppercase << std::setw(2)
                  << std::setfill('0') << static_cast<unsigned int>(sample.firstByte) << std::dec
                  << std::setfill(' ') << "): " << sample.hexadecimalPrefix << '\n';
    }

    if (!statistics.readError.empty())
    {
        std::wcerr << L"  HIDAPI read error: " << statistics.readError << L'\n';
    }
}

} // namespace

int runActiveInterfaceExperiment(
    const std::vector<sensors::XrealDeviceInfo>& devices,
    const ProbeOptions& options)
{
    if (options.interfaceNumber.has_value())
    {
        std::cerr << "Error: --interface cannot be combined with --enable-imu because the experiment"
                     " monitors interfaces 0, 1, 2 and 8 together.\n";
        return 2;
    }

    std::vector<std::unique_ptr<InterfaceSession>> sessions;
    sessions.reserve(reportedInterfaceNumbers.size());
    for (const int interfaceNumber : reportedInterfaceNumbers)
    {
        sessions.push_back(std::make_unique<InterfaceSession>(
            interfaceNumber,
            findSupportedInterface(devices, interfaceNumber)));
    }

    const auto experimentStart = std::chrono::steady_clock::now();
    std::atomic<bool> activationStarted{false};

    for (const int interfaceNumber : candidateInterfaceNumbers)
    {
        auto& session = findSession(sessions, interfaceNumber);
        session.open();
        session.startReader(experimentStart, activationStarted);
    }

    std::cout << "Candidate readers started. Pre-listening for "
              << options.prelistenDuration.count() << " ms before activation.\n";
    std::this_thread::sleep_for(options.prelistenDuration);

    auto& controlSession = findSession(sessions, controlInterfaceNumber);
    controlSession.open();
    controlSession.markSynchronousReaderStart();
    const auto activationTime = std::chrono::steady_clock::now();
    const auto experimentDeadline = activationTime + options.duration;
    bool compatibleAcknowledgementObserved{};

    if (controlSession.connection() == nullptr)
    {
        std::wcerr << L"Cannot send activation: interface 0 failed to open: "
                   << controlSession.openError() << L'\n';
    }
    else
    {
        const auto report = buildImuActivationReport(activationRequestId);
        std::cout << "WARNING: sending one experimental IMU activation command 0x0019 to exact"
                     " VID 0x3318/PID 0x0426 interface 0.\n"
                  << "  Request ID: 0x" << std::hex << std::uppercase << activationRequestId
                  << std::dec << '\n';

        activationStarted.store(true, std::memory_order_release);
        const auto writeResult = controlSession.connection()->writeOutputReport(0, report);

        if (!writeResult.success)
        {
            std::wcerr << L"  HID output report failed on interface 0: "
                       << writeResult.errorMessage << L'\n';
        }
        else
        {
            std::cout << "  HIDAPI bytes written (including Report ID 0): "
                      << writeResult.returnValue << '\n';

            std::array<std::byte, readBufferSize> acknowledgementBuffer{};
            const auto acknowledgementDeadline = std::min(
                std::chrono::steady_clock::now() + acknowledgementTimeout,
                experimentDeadline);

            while (std::chrono::steady_clock::now() < acknowledgementDeadline)
            {
                const auto result = controlSession.connection()->readTimeout(
                    acknowledgementBuffer,
                    readTimeout);

                if (result.status == sensors::HidReadStatus::timeout)
                {
                    continue;
                }

                if (result.status == sensors::HidReadStatus::error)
                {
                    std::wcerr << L"  ACK read failed on interface 0: " << result.errorMessage << L'\n';
                    break;
                }

                const std::span<const std::byte> response(acknowledgementBuffer.data(), result.length);
                controlSession.recordSynchronousPacket(response, false, experimentStart);
                std::cout << "  Raw interface 0 response (" << result.length << " bytes): "
                          << formatHexPrefix(response, maximumSampleBytes) << '\n';

                if (isCompatibleImuActivationAck(response, activationRequestId))
                {
                    compatibleAcknowledgementObserved = true;
                    break;
                }
            }
        }

        controlSession.startReader(experimentStart, activationStarted);
    }

    if (!activationStarted.load(std::memory_order_acquire))
    {
        activationStarted.store(true, std::memory_order_release);
    }

    std::cout << "  Compatible ACK 0x04 observed: "
              << (compatibleAcknowledgementObserved ? "yes" : "no") << '\n'
              << "Readers remain active until " << options.duration.count()
              << " seconds after the activation attempt.\n";

    std::this_thread::sleep_until(experimentDeadline);

    for (const auto& session : sessions)
    {
        session->stopReader();
        if (session->interfaceNumber() == controlInterfaceNumber)
        {
            session->markSynchronousReaderStop();
        }
    }

    std::cout << "\nExperiment summary";
    for (const auto& session : sessions)
    {
        printSessionSummary(*session, options.verbose);
    }

    return compatibleAcknowledgementObserved ? 0 : 1;
}

} // namespace xreal::diagnostics
