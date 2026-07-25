#include "sensors/XrealImuStream.hpp"

#include "sensors/XrealImuPacketDecoder.hpp"
#include "sensors/XrealImuProtocol.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <exception>
#include <utility>

namespace xreal::sensors
{
namespace
{

constexpr std::chrono::milliseconds readTimeout{100};

} // namespace

XrealImuStream::XrealImuStream(XrealDeviceInfo deviceInfo)
    : deviceInfo_(std::move(deviceInfo))
{
}

XrealImuStream::~XrealImuStream()
{
    stop();
}

std::optional<XrealDeviceInfo> XrealImuStream::findInterface(
    const std::vector<XrealDeviceInfo>& devices)
{
    for (const auto& device : devices)
    {
        if (device.vendorId == vendorId
            && device.productId == productId
            && device.interfaceNumber == interfaceNumber)
        {
            return device;
        }
    }

    return std::nullopt;
}

bool XrealImuStream::start(ReportCallback callback)
{
    if (running_)
    {
        setError(L"The XREAL IMU stream is already running.");
        return false;
    }

    if (deviceInfo_.vendorId != vendorId
        || deviceInfo_.productId != productId
        || deviceInfo_.interfaceNumber != interfaceNumber)
    {
        setError(L"The selected device is not exact VID 0x3318/PID 0x0426/interface 2.");
        return false;
    }

    connection_.emplace(deviceInfo_.path);
    if (!connection_->isOpen())
    {
        setError(L"hid_open_path failed for XREAL Air 2 Ultra interface 2: " + connection_->openError());
        connection_.reset();
        return false;
    }

    const HidWriteResult writeResult = connection_->writeRawReport(directImuActivationReport);
    if (!writeResult.success)
    {
        setError(L"hid_write failed while activating XREAL Air 2 Ultra interface 2: "
                 + writeResult.errorMessage);
        connection_.reset();
        return false;
    }

    callback_ = std::move(callback);
    received_ = 0;
    dropped_ = 0;
    invalid_ = 0;
    outOfSequence_ = 0;
    activationResponses_ = 0;
    {
        const std::scoped_lock lock(errorMutex_);
        errorMessage_.clear();
    }

    running_ = true;
    try
    {
        worker_ = std::jthread([this](std::stop_token stopToken) { readLoop(stopToken); });
    }
    catch (const std::exception& exception)
    {
        running_ = false;
        connection_.reset();
        setError(L"Failed to create the XREAL IMU reader thread: "
                 + std::wstring(exception.what(), exception.what() + std::char_traits<char>::length(exception.what())));
        return false;
    }

    return true;
}

void XrealImuStream::stop() noexcept
{
    if (worker_.joinable())
    {
        worker_.request_stop();
        worker_.join();
    }

    running_ = false;
    callback_ = {};
    connection_.reset();
}

bool XrealImuStream::isRunning() const noexcept
{
    return running_;
}

XrealImuStreamStatistics XrealImuStream::statistics() const noexcept
{
    return {
        received_.load(),
        dropped_.load(),
        invalid_.load(),
        outOfSequence_.load(),
        activationResponses_.load(),
    };
}

std::wstring XrealImuStream::errorMessage() const
{
    const std::scoped_lock lock(errorMutex_);
    return errorMessage_;
}

void XrealImuStream::readLoop(std::stop_token stopToken) noexcept
{
    std::array<std::byte, reportSize> buffer{};
    XrealImuPacketDecoder decoder;
    std::optional<std::uint8_t> previousSequence;

    while (!stopToken.stop_requested())
    {
        const HidReadResult readResult = connection_->readTimeout(buffer, readTimeout);
        if (readResult.status == HidReadStatus::timeout)
        {
            continue;
        }

        if (readResult.status == HidReadStatus::error)
        {
            setError(L"hid_read_timeout failed on XREAL Air 2 Ultra interface 2: "
                     + readResult.errorMessage);
            break;
        }

        const auto* bytes = reinterpret_cast<const std::uint8_t*>(buffer.data());
        const std::span<const std::uint8_t> dynamicReport(bytes, readResult.length);
        const XrealImuPacketKind kind = decoder.classify(dynamicReport);

        if (kind == XrealImuPacketKind::activationResponse)
        {
            ++activationResponses_;
            continue;
        }

        if (kind != XrealImuPacketKind::sensor)
        {
            ++invalid_;
            continue;
        }

        const auto fixedReport = std::span<const std::uint8_t, reportSize>(bytes, reportSize);
        const auto sample = decoder.decode(fixedReport, std::chrono::steady_clock::now());
        if (!sample.has_value())
        {
            ++invalid_;
            continue;
        }

        ++received_;
        if (previousSequence.has_value())
        {
            const auto expected = static_cast<std::uint8_t>(*previousSequence + 1U);
            const auto forwardDistance = static_cast<std::uint8_t>(sample->packetSequence - expected);
            if (forwardDistance != 0U)
            {
                ++outOfSequence_;
                if (forwardDistance < 128U)
                {
                    dropped_ += forwardDistance;
                }
            }
        }
        previousSequence = sample->packetSequence;

        try
        {
            if (callback_)
            {
                callback_(fixedReport, *sample);
            }
        }
        catch (...)
        {
            setError(L"The XREAL IMU report callback threw an exception.");
            break;
        }
    }

    running_ = false;
}

void XrealImuStream::setError(std::wstring errorMessage)
{
    const std::scoped_lock lock(errorMutex_);
    errorMessage_ = std::move(errorMessage);
}

} // namespace xreal::sensors
