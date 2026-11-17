#pragma once

#include "rendering/RenderMath.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace xreal::rendering
{

struct XrealSdkHeadPose
{
    std::uint32_t sequence{};
    double senderTimeSeconds{};
    Vector3 position;
    sensors::Quaternion orientation;
    bool valid{};
};

class XrealSdkPoseUdpReceiver
{
public:
    XrealSdkPoseUdpReceiver() = default;
    ~XrealSdkPoseUdpReceiver();
    XrealSdkPoseUdpReceiver(const XrealSdkPoseUdpReceiver&) = delete;
    XrealSdkPoseUdpReceiver& operator=(const XrealSdkPoseUdpReceiver&) = delete;

    [[nodiscard]] bool start(std::uint16_t port = 45871U);
    void stop() noexcept;
    [[nodiscard]] std::optional<XrealSdkHeadPose> poll() noexcept;
    [[nodiscard]] const std::string& error() const noexcept;

private:
    std::uintptr_t socket_{};
    bool winsockStarted_{};
    std::string error_;
};

} // namespace xreal::rendering
