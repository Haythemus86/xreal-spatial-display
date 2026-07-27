#include "sensors/GyroscopeRecordingCsv.hpp"

#include <charconv>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string_view>
#include <unordered_map>
#include <utility>

namespace xreal::sensors
{
namespace
{

[[nodiscard]] std::vector<std::string_view> split(std::string_view line)
{
    std::vector<std::string_view> fields;
    std::size_t begin{};
    while (begin <= line.size())
    {
        const std::size_t comma = line.find(',', begin);
        fields.push_back(line.substr(begin, comma == std::string_view::npos
            ? line.size() - begin : comma - begin));
        if (comma == std::string_view::npos)
        {
            break;
        }
        begin = comma + 1U;
    }
    return fields;
}

template <typename Value>
[[nodiscard]] bool integer(std::string_view text, Value& value)
{
    const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
    return parsed.ec == std::errc{} && parsed.ptr == text.data() + text.size();
}

[[nodiscard]] bool floating(std::string_view text, double& value)
{
    const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
    return parsed.ec == std::errc{} && parsed.ptr == text.data() + text.size()
        && std::isfinite(value);
}

[[nodiscard]] GyroscopeRecordingCsvResult error(std::string message, std::uint64_t line)
{
    GyroscopeRecordingCsvResult result;
    result.error = std::move(message);
    result.lineNumber = line;
    return result;
}

} // namespace

GyroscopeRecordingCsvResult parseGyroscopeRecordingCsv(std::istream& input)
{
    std::string headerLine;
    if (!std::getline(input, headerLine))
    {
        return error("CSV is empty.", 1U);
    }
    if (!headerLine.empty() && headerLine.back() == '\r')
    {
        headerLine.pop_back();
    }
    const auto headers = split(headerLine);
    std::unordered_map<std::string_view, std::size_t> columns;
    for (std::size_t index = 0; index < headers.size(); ++index)
    {
        if (!columns.emplace(headers[index], index).second)
        {
            return error("CSV contains a duplicate column: " + std::string(headers[index]), 1U);
        }
    }
    constexpr std::string_view required[]{
        "sequence", "device_timestamp", "gyro_raw_x", "gyro_raw_y", "gyro_raw_z",
        "bias_raw_x", "bias_raw_y", "bias_raw_z"};
    for (const std::string_view name : required)
    {
        if (!columns.contains(name))
        {
            return error("CSV is missing required column: " + std::string(name), 1U);
        }
    }

    GyroscopeRecordingCsvResult result;
    std::string line;
    std::uint64_t lineNumber{1U};
    while (std::getline(input, line))
    {
        ++lineNumber;
        if (!line.empty() && line.back() == '\r')
        {
            line.pop_back();
        }
        if (line.empty())
        {
            continue;
        }
        const auto fields = split(line);
        if (fields.size() != headers.size())
        {
            return error("CSV row has a different field count than the header.", lineNumber);
        }
        const auto field = [&](std::string_view name) { return fields[columns.at(name)]; };
        unsigned int sequence{};
        std::uint64_t timestamp{};
        std::int32_t gyroX{}, gyroY{}, gyroZ{};
        double biasX{}, biasY{}, biasZ{};
        if (!integer(field("sequence"), sequence) || sequence > 255U
            || !integer(field("device_timestamp"), timestamp)
            || !integer(field("gyro_raw_x"), gyroX) || !integer(field("gyro_raw_y"), gyroY)
            || !integer(field("gyro_raw_z"), gyroZ) || !floating(field("bias_raw_x"), biasX)
            || !floating(field("bias_raw_y"), biasY) || !floating(field("bias_raw_z"), biasZ))
        {
            return error("CSV row contains an invalid required numeric value.", lineNumber);
        }
        const GyroscopeBias rowBias{biasX, biasY, biasZ};
        if (!result.bias.has_value())
        {
            result.bias = rowBias;
        }
        else if (std::abs(result.bias->x - biasX) > 1.0e-9
                 || std::abs(result.bias->y - biasY) > 1.0e-9
                 || std::abs(result.bias->z - biasZ) > 1.0e-9)
        {
            return error("CSV bias metadata changes between rows.", lineNumber);
        }
        ImuSample sample;
        sample.packetSequence = static_cast<std::uint8_t>(sequence);
        sample.deviceTimestamp.nanoseconds = timestamp;
        sample.gyroscopeRaw = {gyroX, gyroY, gyroZ};
        if (columns.contains("host_timestamp_ns") && !field("host_timestamp_ns").empty())
        {
            std::int64_t host{};
            if (!integer(field("host_timestamp_ns"), host))
            {
                return error("CSV row contains an invalid host timestamp.", lineNumber);
            }
            sample.hostReceiveTimestamp = std::chrono::steady_clock::time_point(
                std::chrono::nanoseconds(host));
        }
        if (columns.contains("accel_raw_x") && columns.contains("accel_raw_y")
            && columns.contains("accel_raw_z"))
        {
            std::int32_t x{}, y{}, z{};
            if (!integer(field("accel_raw_x"), x) || !integer(field("accel_raw_y"), y)
                || !integer(field("accel_raw_z"), z))
            {
                return error("CSV row contains invalid accelerometer data.", lineNumber);
            }
            sample.accelerometerRaw = {x, y, z};
        }
        result.samples.push_back(sample);
    }
    if (result.samples.empty())
    {
        return error("CSV contains no sample rows.", lineNumber);
    }
    return result;
}

} // namespace xreal::sensors
