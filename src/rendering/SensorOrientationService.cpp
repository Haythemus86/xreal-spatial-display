#include "rendering/SensorOrientationService.hpp"

#include <chrono>
#include <numbers>
#include <span>

namespace xreal::rendering
{
namespace
{

[[nodiscard]] std::string narrowDiagnostic(const std::wstring& value)
{
    std::string result;
    result.reserve(value.size());
    for (const wchar_t character : value)
    {
        result.push_back(character >= 0 && character <= 0x7f
            ? static_cast<char>(character) : '?');
    }
    return result;
}

} // namespace

SensorOrientationService::SensorOrientationService(
    OrientationRenderBridge& bridge,
    SensorOrientationConfig configuration)
    : bridge_(bridge),
      configuration_(std::move(configuration)),
      calibrator_(configuration_.biasCalibration),
      fusion_(configuration_.fusion),
      predictor_(configuration_.prediction)
{
}

SensorOrientationService::~SensorOrientationService()
{
    stop();
}

bool SensorOrientationService::start()
{
    try
    {
        state_ = RendererStartupState::openingImu;
        hidRuntime_ = std::make_unique<sensors::XrealDevice>();
        const auto device = sensors::XrealImuStream::findInterface(hidRuntime_->enumerate());
        if (!device.has_value())
        {
            setError("No exact XREAL Air 2 Ultra VID 0x3318/PID 0x0426/interface 2 was found.");
            return false;
        }
        stream_ = std::make_unique<sensors::XrealImuStream>(*device);
        state_ = configuration_.biasCalibration.warmupDuration.has_value()
            ? RendererStartupState::warmingUpGyro : RendererStartupState::calibratingGyro;
        const bool started = stream_->start(
            [this](std::span<const std::uint8_t, sensors::XrealImuStream::reportSize>,
                   const sensors::ImuSample& sample) { consume(sample); });
        if (!started)
        {
            setError(narrowDiagnostic(stream_->errorMessage()));
            stream_.reset();
            hidRuntime_.reset();
            return false;
        }
        return true;
    }
    catch (const std::exception& exception)
    {
        setError(std::string("Failed to start the XREAL orientation service: ") + exception.what());
        stream_.reset();
        hidRuntime_.reset();
        return false;
    }
}

void SensorOrientationService::stop() noexcept
{
    state_ = RendererStartupState::shuttingDown;
    if (stream_)
    {
        stream_->stop();
        stream_.reset();
    }
    hidRuntime_.reset();
}

RendererStartupState SensorOrientationService::state() const noexcept
{
    const RendererStartupState current = state_.load();
    if (stream_ && current != RendererStartupState::shuttingDown
        && !stream_->isRunning() && !stream_->errorMessage().empty())
    {
        return RendererStartupState::error;
    }
    return current;
}

std::string SensorOrientationService::error() const
{
    const std::scoped_lock lock(errorMutex_);
    if (!error_.empty())
    {
        return error_;
    }
    return stream_ ? narrowDiagnostic(stream_->errorMessage()) : std::string{};
}

bool SensorOrientationService::calibrationAccepted() const noexcept
{
    return calibrationAccepted_.load();
}

sensors::XrealImuStreamStatistics SensorOrientationService::streamStatistics() const noexcept
{
    return stream_ ? stream_->statistics() : sensors::XrealImuStreamStatistics{};
}

void SensorOrientationService::consume(const sensors::ImuSample& sample) noexcept
{
    if (!firstDeviceTimestamp_.has_value())
    {
        firstDeviceTimestamp_ = sample.deviceTimestamp.nanoseconds;
    }
    if (!bias_.has_value())
    {
        calibrator_.consume(sample);
        const auto elapsed = sample.deviceTimestamp.nanoseconds - *firstDeviceTimestamp_;
        if (configuration_.biasCalibration.warmupDuration.has_value()
            && elapsed >= static_cast<std::uint64_t>(
                configuration_.biasCalibration.warmupDuration->count()))
        {
            state_ = RendererStartupState::calibratingGyro;
        }
        if (!calibrator_.isComplete())
        {
            return;
        }
        const auto result = calibrator_.result();
        if (!result.has_value() || !result->accepted || !result->biasRaw.has_value())
        {
            setError("Gyroscope startup bias calibration was rejected: "
                     + sensors::gyroscopeBiasCalibrationRejectionReasonText(
                         result.has_value() ? result->rejectionReason
                                            : sensors::GyroscopeBiasCalibrationRejectionReason::insufficientSamples));
            return;
        }
        bias_ = *result->biasRaw;
        calibrationAccepted_ = true;
        state_ = RendererStartupState::initializingFusion;
        return;
    }

    const auto gyro = sensors::convertGyroscopeToPhysicalUnits(
        sample.gyroscopeRaw, *bias_, configuration_.gyroscopeScale);
    const auto acceleration = sensors::convertAccelerometerToPhysicalUnits(
        sample.accelerometerRaw, configuration_.accelerometerProfile);
    if (!gyro.x.valid || !gyro.y.valid || !gyro.z.valid || !acceleration.valid)
    {
        return;
    }
    const sensors::AngularVelocityRadians sensorAngularVelocity{
        gyro.x.radiansPerSecond,
        gyro.y.radiansPerSecond,
        gyro.z.radiansPerSecond,
    };
    const auto fusionResult = fusion_.update(
        sensorAngularVelocity, acceleration, sample.deviceTimestamp.nanoseconds);
    if (!fusion_.state().initialized || !fusion_.state().valid
        || fusionResult.gyroscopeStatus == sensors::OrientationSampleStatus::rejected)
    {
        return;
    }

    if (bridge_.consumeClearRecenterRequest())
    {
        fusion_.clearRecenter();
        ++recenterGeneration_;
    }
    const bool startRecenter = configuration_.recenterOnStart
        && !recenteredOnStart_.exchange(true);
    if (bridge_.consumeRecenterRequest() || startRecenter)
    {
        if (fusion_.recenter())
        {
            ++recenterGeneration_;
        }
    }

    RenderOrientationSnapshot snapshot;
    snapshot.measuredAbsolute = fusion_.orientation();
    snapshot.measuredRelative = fusion_.relativeOrientation();
    snapshot.measuredValid = true;
    snapshot.deviceTimestampNanoseconds = sample.deviceTimestamp.nanoseconds;
    snapshot.hostPublishTimestampNanoseconds = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
    snapshot.recenterGeneration = recenterGeneration_;
    const auto streamStats = streamStatistics();
    snapshot.imu = {streamStats.received, streamStats.dropped,
                    streamStats.invalid, streamStats.outOfSequence};

    if (configuration_.predictionEnabled)
    {
        const auto bodyAngularVelocity = sensors::mapAngularVelocity(
            sensorAngularVelocity, configuration_.gyroscopeScale.axisMapping);
        const auto predicted = predictor_.predict({
            {snapshot.measuredAbsolute},
            {fusion_.state().recenterReference, fusion_.state().recenterActive},
            bodyAngularVelocity,
            sample.deviceTimestamp.nanoseconds,
        });
        snapshot.predictionValid = predicted.validity == sensors::PredictionValidity::valid;
        snapshot.predictedAbsolute = predicted.absolute.value;
        snapshot.predictedRelative = predicted.relative.value;
        snapshot.predictionHorizonMilliseconds = predicted.horizon.applied.count() * 1000.0;
    }
    (void)bridge_.publish(snapshot);
    state_ = RendererStartupState::ready;
}

void SensorOrientationService::setError(std::string message) noexcept
{
    {
        const std::scoped_lock lock(errorMutex_);
        error_ = std::move(message);
    }
    state_ = RendererStartupState::error;
}

} // namespace xreal::rendering
