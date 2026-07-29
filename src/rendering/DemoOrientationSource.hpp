#pragma once

#include "rendering/OrientationRenderBridge.hpp"

namespace xreal::rendering
{

class DemoOrientationSource
{
public:
    DemoOrientationSource(OrientationRenderBridge& bridge, double predictionHorizonMilliseconds);

    void update(double elapsedSeconds) noexcept;
    void adjustYaw(double radians) noexcept;
    void adjustPitch(double radians) noexcept;
    void adjustRoll(double radians) noexcept;
    void reset() noexcept;

private:
    [[nodiscard]] sensors::Quaternion orientationAt(double elapsedSeconds) const noexcept;

    OrientationRenderBridge& bridge_;
    double predictionHorizonSeconds_{};
    sensors::Quaternion recenterReference_;
    bool recenterActive_{};
    std::uint64_t recenterGeneration_{};
    double yawOffset_{};
    double pitchOffset_{};
    double rollOffset_{};
};

} // namespace xreal::rendering
