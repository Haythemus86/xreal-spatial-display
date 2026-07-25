#include "sensors/XrealImuPacketDecoder.hpp"

namespace xreal::sensors
{
namespace
{

constexpr std::uint8_t sensorPrefix = 0x01;
constexpr std::uint8_t sensorType = 0x02;
constexpr std::uint8_t activationResponsePrefix = 0xAA;
constexpr std::size_t deviceTimestampOffset = 4;
constexpr std::size_t gyroscopeOffset = 18;
constexpr std::size_t accelerometerOffset = 33;
constexpr std::size_t packetSequenceOffset = 63;

std::uint64_t readLittleEndian64(std::span<const std::uint8_t, 8> bytes) noexcept
{
    std::uint64_t value{};

    for (std::size_t index = 0; index < bytes.size(); ++index)
    {
        value |= static_cast<std::uint64_t>(bytes[index]) << (index * 8U);
    }

    return value;
}

std::int32_t readSignedLittleEndian24(std::span<const std::uint8_t, 3> bytes) noexcept
{
    std::uint32_t value = static_cast<std::uint32_t>(bytes[0])
        | (static_cast<std::uint32_t>(bytes[1]) << 8U)
        | (static_cast<std::uint32_t>(bytes[2]) << 16U);

    if ((value & 0x00800000U) != 0U)
    {
        value |= 0xFF000000U;
    }

    return static_cast<std::int32_t>(value);
}

RawImuVector3 readRawVector(std::span<const std::uint8_t, 9> bytes) noexcept
{
    return {
        readSignedLittleEndian24(std::span<const std::uint8_t, 3>(bytes.data(), 3)),
        readSignedLittleEndian24(std::span<const std::uint8_t, 3>(bytes.data() + 3, 3)),
        readSignedLittleEndian24(std::span<const std::uint8_t, 3>(bytes.data() + 6, 3)),
    };
}

} // namespace

XrealImuPacketKind XrealImuPacketDecoder::classify(std::span<const std::uint8_t> report) const noexcept
{
    if (report.size() != reportSize)
    {
        return XrealImuPacketKind::invalid;
    }

    if (report[0] == activationResponsePrefix)
    {
        return XrealImuPacketKind::activationResponse;
    }

    if (report[0] == sensorPrefix && report[1] == sensorType)
    {
        return XrealImuPacketKind::sensor;
    }

    return XrealImuPacketKind::invalid;
}

std::optional<ImuSample> XrealImuPacketDecoder::decode(
    std::span<const std::uint8_t, reportSize> report,
    std::chrono::steady_clock::time_point hostReceiveTimestamp) const noexcept
{
    if (classify(report) != XrealImuPacketKind::sensor)
    {
        return std::nullopt;
    }

    ImuSample sample;
    sample.deviceTimestamp.nanoseconds = readLittleEndian64(
        std::span<const std::uint8_t, 8>(report.data() + deviceTimestampOffset, 8));
    sample.gyroscopeRaw = readRawVector(
        std::span<const std::uint8_t, 9>(report.data() + gyroscopeOffset, 9));
    sample.accelerometerRaw = readRawVector(
        std::span<const std::uint8_t, 9>(report.data() + accelerometerOffset, 9));
    sample.packetSequence = report[packetSequenceOffset];
    sample.hostReceiveTimestamp = hostReceiveTimestamp;

    // AirAPI_Windows labels its scale and bias corrections as rough guesses.
    // SI values therefore remain empty until hardware scaling is independently confirmed.
    return sample;
}

std::optional<ImuSample> XrealImuPacketDecoder::decode(
    std::span<const std::uint8_t> report,
    std::chrono::steady_clock::time_point hostReceiveTimestamp) const noexcept
{
    if (report.size() != reportSize)
    {
        return std::nullopt;
    }

    return decode(
        std::span<const std::uint8_t, reportSize>(report.data(), reportSize),
        hostReceiveTimestamp);
}

} // namespace xreal::sensors
