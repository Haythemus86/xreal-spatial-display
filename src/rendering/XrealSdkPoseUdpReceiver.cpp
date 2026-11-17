#include "rendering/XrealSdkPoseUdpReceiver.hpp"

#include <winsock2.h>
#include <ws2tcpip.h>

#include <array>
#include <charconv>
#include <cmath>
#include <string_view>

namespace xreal::rendering
{
namespace
{

constexpr std::size_t maximumDatagramSize = 256U;

template<typename Value>
[[nodiscard]] bool parseNumber(std::string_view text, Value& value) noexcept
{
    const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
    return result.ec == std::errc{} && result.ptr == text.data() + text.size();
}

[[nodiscard]] bool parsePose(std::string_view text, XrealSdkHeadPose& pose) noexcept
{
    std::array<std::string_view, 11U> fields{};
    std::size_t count{};
    while (true)
    {
        if (count == fields.size())
        {
            return false;
        }
        const std::size_t separator = text.find(',');
        fields[count++] = text.substr(0U, separator);
        if (separator == std::string_view::npos)
        {
            break;
        }
        text.remove_prefix(separator + 1U);
    }
    if (count != fields.size())
    {
        return false;
    }

    unsigned int version{};
    unsigned int valid{};
    double x{}, y{}, z{}, qx{}, qy{}, qz{}, qw{};
    if (!parseNumber(fields[0], version) || version != 1U
        || !parseNumber(fields[1], pose.sequence)
        || !parseNumber(fields[2], valid) || valid > 1U
        || !parseNumber(fields[3], pose.senderTimeSeconds)
        || !parseNumber(fields[4], x) || !parseNumber(fields[5], y)
        || !parseNumber(fields[6], z) || !parseNumber(fields[7], qx)
        || !parseNumber(fields[8], qy) || !parseNumber(fields[9], qz)
        || !parseNumber(fields[10], qw))
    {
        return false;
    }

    pose.valid = valid == 1U;
    pose.position = {x, y, z};
    pose.orientation = {qw, qx, qy, qz};
    if (!std::isfinite(pose.senderTimeSeconds) || !pose.position.finite())
    {
        return false;
    }
    if (pose.valid && !pose.orientation.normalized().has_value())
    {
        return false;
    }
    return true;
}

} // namespace

XrealSdkPoseUdpReceiver::~XrealSdkPoseUdpReceiver()
{
    stop();
}

bool XrealSdkPoseUdpReceiver::start(std::uint16_t port)
{
    stop();
    error_.clear();

    WSADATA data{};
    const int startupResult = WSAStartup(MAKEWORD(2, 2), &data);
    if (startupResult != 0)
    {
        error_ = "WSAStartup failed with code " + std::to_string(startupResult) + ".";
        return false;
    }
    winsockStarted_ = true;

    const SOCKET socketHandle = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socketHandle == INVALID_SOCKET)
    {
        error_ = "Could not create the XREAL SDK pose UDP socket.";
        stop();
        return false;
    }
    socket_ = static_cast<std::uintptr_t>(socketHandle);

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (::bind(socketHandle, reinterpret_cast<const sockaddr*>(&address), sizeof(address))
        == SOCKET_ERROR)
    {
        error_ = "Could not bind the XREAL SDK pose UDP port "
            + std::to_string(port) + ".";
        stop();
        return false;
    }

    u_long nonBlocking = 1UL;
    if (ioctlsocket(socketHandle, FIONBIO, &nonBlocking) == SOCKET_ERROR)
    {
        error_ = "Could not make the XREAL SDK pose UDP socket non-blocking.";
        stop();
        return false;
    }
    return true;
}

void XrealSdkPoseUdpReceiver::stop() noexcept
{
    if (socket_ != 0U)
    {
        ::closesocket(static_cast<SOCKET>(socket_));
        socket_ = 0U;
    }
    if (winsockStarted_)
    {
        WSACleanup();
        winsockStarted_ = false;
    }
}

std::optional<XrealSdkHeadPose> XrealSdkPoseUdpReceiver::poll() noexcept
{
    if (socket_ == 0U)
    {
        return std::nullopt;
    }

    std::optional<XrealSdkHeadPose> latest;
    std::array<char, maximumDatagramSize> buffer{};
    while (true)
    {
        const int received = ::recv(static_cast<SOCKET>(socket_), buffer.data(),
            static_cast<int>(buffer.size()), 0);
        if (received == SOCKET_ERROR)
        {
            const int error = WSAGetLastError();
            if (error != WSAEWOULDBLOCK)
            {
                error_ = "XREAL SDK pose UDP receive failed with code "
                    + std::to_string(error) + ".";
            }
            break;
        }
        XrealSdkHeadPose pose;
        if (parsePose(std::string_view(buffer.data(), static_cast<std::size_t>(received)), pose))
        {
            latest = pose;
        }
    }
    return latest;
}

const std::string& XrealSdkPoseUdpReceiver::error() const noexcept
{
    return error_;
}

} // namespace xreal::rendering
