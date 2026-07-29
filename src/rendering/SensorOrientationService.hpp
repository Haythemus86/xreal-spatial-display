#pragma once

#include "rendering/OrientationRenderBridge.hpp"
#include "rendering/RenderDiagnostics.hpp"
#include "sensors/AccelerometerPhysicalUnits.hpp"
#include "sensors/GyroscopeBiasCalibration.hpp"
#include "sensors/GyroscopePhysicalUnits.hpp"
#include "sensors/OrientationFusion.hpp"
#include "sensors/OrientationPrediction.hpp"
#include "sensors/XrealDevice.hpp"
#include "sensors/XrealImuStream.hpp"

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace xreal::rendering
{

struct SensorOrientationConfig
{
    sensors::GyroscopeScaleProfile gyroscopeScale;
    sensors::AccelerometerCalibrationProfile accelerometerProfile;
    sensors::GyroscopeBiasCalibrationConfig biasCalibration;
    sensors::OrientationFusionConfig fusion;
    sensors::OrientationPredictorConfig prediction;
    bool predictionEnabled{};
    bool recenterOnStart{};
};

class SensorOrientationService
{
public:
    SensorOrientationService(
        OrientationRenderBridge& bridge,
        SensorOrientationConfig configuration);
    ~SensorOrientationService();

    SensorOrientationService(const SensorOrientationService&) = delete;
    SensorOrientationService& operator=(const SensorOrientationService&) = delete;

    [[nodiscard]] bool start();
    void stop() noexcept;
    [[nodiscard]] RendererStartupState state() const noexcept;
    [[nodiscard]] std::string error() const;
    [[nodiscard]] bool calibrationAccepted() const noexcept;
    [[nodiscard]] sensors::XrealImuStreamStatistics streamStatistics() const noexcept;

private:
    void consume(const sensors::ImuSample& sample) noexcept;
    void setError(std::string message) noexcept;

    OrientationRenderBridge& bridge_;
    SensorOrientationConfig configuration_;
    sensors::GyroscopeBiasCalibrator calibrator_;
    sensors::OrientationFusionFilter fusion_;
    sensors::OrientationPredictor predictor_;
    std::optional<sensors::GyroscopeBias> bias_;
    std::unique_ptr<sensors::XrealDevice> hidRuntime_;
    std::unique_ptr<sensors::XrealImuStream> stream_;
    std::atomic<RendererStartupState> state_{RendererStartupState::openingImu};
    std::atomic_bool calibrationAccepted_{};
    std::atomic_bool recenteredOnStart_{};
    std::uint64_t recenterGeneration_{};
    std::optional<std::uint64_t> firstDeviceTimestamp_;
    mutable std::mutex errorMutex_;
    std::string error_;
};

} // namespace xreal::rendering
