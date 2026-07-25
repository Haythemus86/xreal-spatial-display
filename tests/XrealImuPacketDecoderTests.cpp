#include "sensors/XrealImuPacketDecoder.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <span>
#include <string_view>

namespace
{

using xreal::sensors::XrealImuPacketDecoder;
using xreal::sensors::XrealImuPacketKind;

int failureCount{};

constexpr std::array<std::uint8_t, 64> capturedActivationResponse{
    0xAA, 0x53, 0xE1, 0x26, 0x35, 0x04, 0x00, 0x19,
};

constexpr std::array<std::uint8_t, 64> capturedSensorReport{
    0x01, 0x02, 0x0A, 0x04, 0xF1, 0x2C, 0xCA, 0xAD,
    0xD1, 0x02, 0x00, 0x00, 0xA0, 0x0F, 0x00, 0x00,
    0x00, 0x01, 0x00, 0xB3, 0xFF, 0x60, 0x19, 0x00,
    0x00, 0x15, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x01, 0xC0, 0xE8, 0xFF, 0x40, 0x50, 0x01,
    0x00, 0x0C, 0x08, 0x00, 0x80, 0x00, 0x04, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0,
    0x6B, 0x26, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
};

void expect(bool condition, std::string_view description)
{
    if (!condition)
    {
        ++failureCount;
        std::cerr << "FAILED: " << description << '\n';
    }
}

void testActivationResponseClassification()
{
    const XrealImuPacketDecoder decoder;
    expect(decoder.classify(capturedActivationResponse) == XrealImuPacketKind::activationResponse,
           "captured 0xAA response is classified as activation response");
    expect(!decoder.decode(std::span<const std::uint8_t, 64>(capturedActivationResponse), {}).has_value(),
           "activation response is not decoded as a sensor sample");
}

void testCapturedSensorPacketParsing()
{
    const XrealImuPacketDecoder decoder;
    const auto hostTimestamp = std::chrono::steady_clock::time_point(std::chrono::nanoseconds(123456));
    const auto sample = decoder.decode(
        std::span<const std::uint8_t, 64>(capturedSensorReport), hostTimestamp);

    expect(sample.has_value(), "captured sensor report parses");
    if (!sample.has_value())
    {
        return;
    }

    expect(sample->deviceTimestamp.nanoseconds == 0x000002D1ADCA2CF1ULL,
           "device timestamp is decoded as little-endian uint64");
    expect(sample->gyroscopeRaw.x == -19712, "negative gyro X is sign-extended from little-endian int24");
    expect(sample->gyroscopeRaw.y == 6496, "gyro Y is decoded from little-endian int24");
    expect(sample->gyroscopeRaw.z == 5376, "gyro Z is decoded from little-endian int24");
    expect(sample->accelerometerRaw.x == -1523711,
           "negative accelerometer X is sign-extended from little-endian int24");
    expect(sample->accelerometerRaw.y == 5259519,
           "accelerometer Y is decoded from little-endian int24");
    expect(sample->accelerometerRaw.z == 786433,
           "accelerometer Z is decoded from little-endian int24");
    expect(sample->packetSequence == 0x01, "last report byte is extracted as sequence");
    expect(sample->hostReceiveTimestamp == hostTimestamp, "host receive timestamp is preserved");
    expect(!sample->gyroscopeRadiansPerSecond.has_value(), "unverified gyroscope scaling is not emitted");
    expect(!sample->accelerometerMetersPerSecondSquared.has_value(),
           "unverified accelerometer scaling is not emitted");
}

void testMalformedSizeAndPrefixRejection()
{
    const XrealImuPacketDecoder decoder;
    const std::span<const std::uint8_t> shortReport(capturedSensorReport.data(), 63);
    expect(decoder.classify(shortReport) == XrealImuPacketKind::invalid,
           "63-byte report is rejected by the production decoder");
    expect(!decoder.decode(shortReport, {}).has_value(), "malformed report size is rejected cleanly");

    auto invalidPrefix = capturedSensorReport;
    invalidPrefix[0] = 0xFF;
    expect(decoder.classify(invalidPrefix) == XrealImuPacketKind::invalid,
           "unknown packet prefix is rejected");
    expect(!decoder.decode(std::span<const std::uint8_t, 64>(invalidPrefix), {}).has_value(),
           "invalid-prefix report does not produce a sample");
}

} // namespace

int main()
{
    testActivationResponseClassification();
    testCapturedSensorPacketParsing();
    testMalformedSizeAndPrefixRejection();

    if (failureCount != 0)
    {
        std::cerr << failureCount << " IMU decoder test(s) failed.\n";
        return 1;
    }

    std::cout << "All XREAL IMU packet decoder tests passed.\n";
    return 0;
}
