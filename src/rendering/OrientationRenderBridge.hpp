#pragma once

#include "rendering/RenderMath.hpp"
#include "sensors/OrientationPrediction.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

namespace xreal::rendering
{

enum class RenderAxis { positiveX, negativeX, positiveY, negativeY, positiveZ, negativeZ };
enum class RenderHandedness { rightHanded };
enum class RenderOrientationSource { measured, predicted };
enum class RenderOrientationFrame { absolute, relative };

struct RenderAxisMapping
{
    RenderAxis sensorX{RenderAxis::positiveX};
    RenderAxis sensorY{RenderAxis::negativeZ};
    RenderAxis sensorZ{RenderAxis::positiveY};
};

struct OrientationToRenderMapping
{
    RenderAxisMapping axes;
    RenderHandedness handedness{RenderHandedness::rightHanded};
    bool experimental{true};
    bool verified{true};
    std::string source{"sensor-x-right-y-forward-z-up_to_render-x-right-y-up-z-back"};
};

struct ImuHealthCounters
{
    std::uint64_t received{};
    std::uint64_t dropped{};
    std::uint64_t invalid{};
    std::uint64_t outOfSequence{};
};

struct RenderOrientationSnapshot
{
    sensors::Quaternion measuredAbsolute;
    sensors::Quaternion measuredRelative;
    sensors::Quaternion predictedAbsolute;
    sensors::Quaternion predictedRelative;
    std::uint64_t deviceTimestampNanoseconds{};
    std::uint64_t hostPublishTimestampNanoseconds{};
    double predictionHorizonMilliseconds{};
    bool measuredValid{};
    bool predictionValid{};
    Vector3 measuredPositionRelative;
    bool measuredPositionValid{};
    std::uint64_t recenterGeneration{};
    ImuHealthCounters imu;
    std::uint64_t sequence{};
};

struct SelectedRenderOrientation
{
    sensors::Quaternion orientation;
    bool valid{};
    bool preservingLastValid{};
    std::uint64_t snapshotSequence{};
};

[[nodiscard]] Vector3 mapSensorVectorToRender(
    Vector3 sensorVector,
    const OrientationToRenderMapping& mapping) noexcept;
[[nodiscard]] std::optional<sensors::Quaternion> mapSensorOrientationToRender(
    const sensors::Quaternion& sensorOrientation,
    const OrientationToRenderMapping& mapping) noexcept;
[[nodiscard]] bool validateRenderMapping(const OrientationToRenderMapping& mapping) noexcept;
[[nodiscard]] std::string renderAxisText(RenderAxis axis);
[[nodiscard]] std::string renderMappingText(const OrientationToRenderMapping& mapping);

class OrientationRenderBridge
{
public:
    [[nodiscard]] bool publish(RenderOrientationSnapshot snapshot) noexcept;
    [[nodiscard]] std::optional<RenderOrientationSnapshot> latest() const noexcept;
    void requestRecenter() noexcept;
    void requestClearRecenter() noexcept;
    [[nodiscard]] bool consumeRecenterRequest() noexcept;
    [[nodiscard]] bool consumeClearRecenterRequest() noexcept;

private:
    mutable std::mutex mutex_;
    std::optional<RenderOrientationSnapshot> latest_;
    std::uint64_t nextSequence_{1};
    std::atomic_bool recenterRequested_{};
    std::atomic_bool clearRecenterRequested_{};
};

class RenderOrientationSelector
{
public:
    [[nodiscard]] SelectedRenderOrientation select(
        const RenderOrientationSnapshot& snapshot,
        RenderOrientationSource source,
        RenderOrientationFrame frame,
        const OrientationToRenderMapping& mapping) noexcept;
    void reset() noexcept;

private:
    std::optional<sensors::Quaternion> lastValid_;
};

} // namespace xreal::rendering
