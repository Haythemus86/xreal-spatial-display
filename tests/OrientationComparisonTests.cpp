#include "sensors/OrientationComparison.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <string_view>
#include <type_traits>

namespace
{

static_assert(!std::is_convertible_v<
    xreal::sensors::GyroPredictionAbsolute,
    xreal::sensors::GyroPredictionRelative>);
static_assert(!std::is_convertible_v<
    xreal::sensors::FusedOrientationAbsolute,
    xreal::sensors::FusedOrientationRelative>);

using namespace std::chrono_literals;
int failures{};

void expect(bool condition, std::string_view description)
{
    if (!condition)
    {
        ++failures;
        std::cerr << "FAILED: " << description << '\n';
    }
}

[[nodiscard]] bool near(double left, double right, double tolerance = 1.0e-7)
{
    return std::abs(left - right) <= tolerance;
}

class JsonSyntaxParser
{
public:
    explicit JsonSyntaxParser(std::string_view text) : text_(text) {}

    [[nodiscard]] bool valid()
    {
        skipWhitespace();
        const bool parsed = parseValue();
        skipWhitespace();
        return parsed && position_ == text_.size();
    }

private:
    void skipWhitespace()
    {
        while (position_ < text_.size()
               && (text_[position_] == ' ' || text_[position_] == '\n'
                   || text_[position_] == '\r' || text_[position_] == '\t'))
        {
            ++position_;
        }
    }

    [[nodiscard]] bool consume(char expected)
    {
        skipWhitespace();
        if (position_ >= text_.size() || text_[position_] != expected)
        {
            return false;
        }
        ++position_;
        return true;
    }

    [[nodiscard]] bool parseValue()
    {
        skipWhitespace();
        if (position_ >= text_.size())
        {
            return false;
        }
        if (text_[position_] == '{')
        {
            return parseObject();
        }
        if (text_[position_] == '[')
        {
            return parseArray();
        }
        if (text_[position_] == '"')
        {
            return parseString();
        }
        if (text_.substr(position_, 4) == "true" || text_.substr(position_, 4) == "null")
        {
            position_ += 4U;
            return true;
        }
        if (text_.substr(position_, 5) == "false")
        {
            position_ += 5U;
            return true;
        }
        return parseNumber();
    }

    [[nodiscard]] bool parseString()
    {
        if (!consume('"'))
        {
            return false;
        }
        while (position_ < text_.size())
        {
            const char character = text_[position_++];
            if (character == '"')
            {
                return true;
            }
            if (character == '\\')
            {
                if (position_ >= text_.size())
                {
                    return false;
                }
                ++position_;
            }
            else if (static_cast<unsigned char>(character) < 0x20U)
            {
                return false;
            }
        }
        return false;
    }

    [[nodiscard]] bool parseNumber()
    {
        const std::size_t begin = position_;
        if (position_ < text_.size() && text_[position_] == '-')
        {
            ++position_;
        }
        while (position_ < text_.size() && text_[position_] >= '0' && text_[position_] <= '9')
        {
            ++position_;
        }
        if (position_ < text_.size() && text_[position_] == '.')
        {
            ++position_;
            while (position_ < text_.size() && text_[position_] >= '0' && text_[position_] <= '9')
            {
                ++position_;
            }
        }
        if (position_ < text_.size() && (text_[position_] == 'e' || text_[position_] == 'E'))
        {
            ++position_;
            if (position_ < text_.size() && (text_[position_] == '+' || text_[position_] == '-'))
            {
                ++position_;
            }
            while (position_ < text_.size() && text_[position_] >= '0' && text_[position_] <= '9')
            {
                ++position_;
            }
        }
        return position_ > begin;
    }

    [[nodiscard]] bool parseObject()
    {
        if (!consume('{'))
        {
            return false;
        }
        skipWhitespace();
        if (consume('}'))
        {
            return true;
        }
        do
        {
            if (!parseString() || !consume(':') || !parseValue())
            {
                return false;
            }
            skipWhitespace();
            if (consume('}'))
            {
                return true;
            }
        } while (consume(','));
        return false;
    }

    [[nodiscard]] bool parseArray()
    {
        if (!consume('['))
        {
            return false;
        }
        skipWhitespace();
        if (consume(']'))
        {
            return true;
        }
        do
        {
            if (!parseValue())
            {
                return false;
            }
            skipWhitespace();
            if (consume(']'))
            {
                return true;
            }
        } while (consume(','));
        return false;
    }

    std::string_view text_;
    std::size_t position_{};
};

[[nodiscard]] xreal::sensors::Quaternion rotation(double x, double y, double z, double degrees)
{
    const double radians = degrees * std::numbers::pi / 180.0;
    const double sine = std::sin(radians * 0.5);
    return {std::cos(radians * 0.5), x * sine, y * sine, z * sine};
}

[[nodiscard]] xreal::sensors::AccelerometerPhysicalSample acceleration(
    double x, double y, double z)
{
    xreal::sensors::AccelerometerPhysicalSample sample;
    sample.accelerationG = {x, y, z};
    sample.normG = sample.accelerationG.norm();
    sample.valid = sample.accelerationG.finite();
    return sample;
}

void testFramesAndDistances()
{
    using namespace xreal::sensors;
    const Quaternion identity;
    const Quaternion x90 = rotation(1, 0, 0, 90);
    const Quaternion y180 = rotation(0, 1, 0, 180);
    const Quaternion negative{-x90.w, -x90.x, -x90.y, -x90.z};
    expect(near(quaternionAngularDistance(identity, identity)->degrees, 0.0),
           "identity distance is zero");
    expect(near(quaternionAngularDistance(x90, negative)->degrees, 0.0),
           "q and -q are equivalent");
    expect(near(quaternionAngularDistance(identity, x90)->degrees, 90.0),
           "identity to X 90 is 90 degrees");
    expect(near(quaternionAngularDistance(identity, y180)->degrees, 180.0),
           "identity to Y 180 is 180 degrees");
    expect(near(quaternionAngularDistance({2, 0, 0, 0}, {0, 2, 0, 0})->degrees, 180.0),
           "finite non-normalized quaternions are normalized");
    expect(near(quaternionAngularDistance(x90, identity)->degrees,
                quaternionAngularDistance(identity, x90)->degrees),
           "quaternion distance is symmetric");
    expect(!quaternionAngularDistance({}, {0, 0, 0, 0}).has_value(),
           "zero quaternion is rejected");
    expect(!quaternionAngularDistance({}, {
               std::numeric_limits<double>::quiet_NaN(), 0, 0, 0}).has_value(),
           "NaN quaternion is rejected");
    expect(!quaternionAngularDistance({}, {
               std::numeric_limits<double>::infinity(), 0, 0, 0}).has_value(),
           "infinite quaternion is rejected");
    for (int degrees = 0; degrees <= 360; degrees += 5)
    {
        const auto value = quaternionAngularDistance(identity, rotation(1, 0, 0, degrees));
        expect(value.has_value() && value->degrees >= 0.0 && value->degrees <= 180.0,
               "quaternion angular distance stays in its documented range");
    }
    expect(near(quaternionTiltDistance(identity, rotation(0, 0, 1, 90))->degrees, 0.0),
           "tilt distance ignores pure yaw");
    expect(near(quaternionTiltDistance(identity, x90)->degrees, 90.0),
           "tilt distance detects roll");
    expect(near(quaternionTiltDistance(identity, rotation(0, 1, 0, 30))->degrees, 30.0),
           "tilt distance detects pitch");

    RecenterReference reference{x90, true};
    const auto relative = makeRelativeOrientation({x90}, reference);
    expect(relative.has_value()
               && near(quaternionAngularDistance(relative->value, identity)->degrees, 0.0),
           "relative orientation uses inverse(reference) times absolute");
}

void testParallelEngineAndRecenter()
{
    using namespace xreal::sensors;
    OrientationComparisonConfig config;
    config.fusion.confidence.smoothingTimeConstant = 0s;
    config.fusion.correctionTimeConstant = 0.25s;
    config.fusion.maximumCorrectionDegreesPerSecond = 180.0;
    OrientationComparisonEngine engine(config);
    expect(engine.recenter(), "recenter at identity succeeds");
    expect(near(quaternionAngularDistance({}, engine.snapshot().gyroRelative.value)->degrees, 0.0)
               && near(quaternionAngularDistance({}, engine.snapshot().fusedRelative.value)->degrees, 0.0),
           "recenter at identity changes nothing");
    engine.clearRecenter();
    const AngularVelocityRadians bias{0.01, 0.0, 0.0};
    auto update = engine.update(bias, acceleration(0, 0, 1), 0U);
    for (std::uint64_t timestamp = 10'000'000U; timestamp <= 5'000'000'000U;
         timestamp += 10'000'000U)
    {
        update = engine.update(bias, acceleration(0, 0, 1), timestamp);
    }
    expect(update.gyroscope.deltaTime == update.fusion.deltaTime,
           "parallel paths consume the same device timestamps");
    expect(update.fusion.correctionStatus == FusionCorrectionStatus::applied,
           "only fused path receives gravity correction");
    const double gyroTilt = quaternionTiltDistance({}, update.orientations.gyroAbsolute.value)->degrees;
    const double fusedTilt = quaternionTiltDistance({}, update.orientations.fusedAbsolute.value)->degrees;
    expect(fusedTilt < gyroTilt * 0.25, "fusion reduces synthetic roll bias drift");

    const AngularVelocityRadians originalInput{0.02, -0.03, 0.04};
    const auto originalAcceleration = acceleration(0.1, 0.2, 0.97);
    const auto inputCopy = originalInput;
    const auto accelerationCopy = originalAcceleration;
    static_cast<void>(engine.update(originalInput, originalAcceleration, 5'010'000'000U));
    expect(originalInput.xRadiansPerSecond == inputCopy.xRadiansPerSecond
               && originalInput.yRadiansPerSecond == inputCopy.yRadiansPerSecond
               && originalInput.zRadiansPerSecond == inputCopy.zRadiansPerSecond
               && originalAcceleration.accelerationG.x == accelerationCopy.accelerationG.x,
           "parallel comparison does not modify input samples");

    const Quaternion gyroAbsoluteBefore = engine.gyroscope().orientation();
    const Quaternion fusedAbsoluteBefore = engine.fusion().orientation();
    expect(engine.recenter(), "parallel recenter succeeds");
    const auto recentered = engine.snapshot();
    expect(recentered.gyroRecenterReference.active && recentered.fusedRecenterReference.active,
           "both recenter references become active");
    expect(near(quaternionAngularDistance(gyroAbsoluteBefore,
                    recentered.gyroAbsolute.value)->degrees, 0.0)
               && near(quaternionAngularDistance(fusedAbsoluteBefore,
                    recentered.fusedAbsolute.value)->degrees, 0.0),
           "recenter never modifies either absolute orientation");
    expect(near(quaternionAngularDistance({}, recentered.gyroRelative.value)->degrees, 0.0)
               && near(quaternionAngularDistance({}, recentered.fusedRelative.value)->degrees, 0.0),
           "both relative outputs become identity under compatible reference policy");
    engine.clearRecenter();
    const auto cleared = engine.snapshot();
    expect(near(quaternionAngularDistance(cleared.gyroAbsolute.value,
                    cleared.gyroRelative.value)->degrees, 0.0)
               && near(quaternionAngularDistance(cleared.fusedAbsolute.value,
                    cleared.fusedRelative.value)->degrees, 0.0),
           "clearing recenter restores absolute outputs");

    OrientationComparisonEngine yawEngine(config);
    auto yawUpdate = yawEngine.update({0, 0, 0.01}, acceleration(0, 0, 1), 0U);
    for (std::uint64_t timestamp = 10'000'000U; timestamp <= 2'000'000'000U;
         timestamp += 10'000'000U)
    {
        yawUpdate = yawEngine.update({0, 0, 0.01}, acceleration(0, 0, 1), timestamp);
    }
    expect(std::abs(yawUpdate.orientations.fusedAbsoluteEuler.yawDegrees) > 0.5,
           "fusion does not claim to correct yaw bias");

    OrientationComparisonEngine pitchEngine(config);
    auto pitchUpdate = pitchEngine.update({0, 0.01, 0}, acceleration(0, 0, 1), 0U);
    for (std::uint64_t timestamp = 10'000'000U; timestamp <= 5'000'000'000U;
         timestamp += 10'000'000U)
    {
        pitchUpdate = pitchEngine.update({0, 0.01, 0}, acceleration(0, 0, 1), timestamp);
    }
    expect(quaternionTiltDistance({}, pitchUpdate.orientations.fusedAbsolute.value)->degrees
               < quaternionTiltDistance({}, pitchUpdate.orientations.gyroAbsolute.value)->degrees * 0.25,
           "fusion reduces synthetic pitch bias drift");
}

void testStationaryDriftAndConvergence()
{
    using namespace xreal::sensors;
    StationaryDetectorConfig config;
    config.minimumDuration = 500ms;
    StationaryDetector detector(config);
    auto result = detector.update({}, acceleration(0, 0, 1), 1.0, 0U);
    expect(!result.stationary && result.reason == StationaryReason::sustaining,
           "stationary detector enforces sustained duration");
    result = detector.update({}, acceleration(0, 0, 1), 1.0, 500'000'000U);
    expect(result.stationary, "level low-speed sample becomes stationary after sustain time");
    expect(detector.update({1, 0, 0}, acceleration(0, 0, 1), 1.0, 600'000'000U).reason
               == StationaryReason::angularSpeedTooHigh,
           "high gyro speed is not stationary");
    expect(detector.update({}, acceleration(0, 0, 1.2), 1.0, 700'000'000U).reason
               == StationaryReason::accelerationNormOutsideRange,
           "acceleration far from one g is not stationary");
    expect(detector.update({}, acceleration(0, 0, 1), 0.1, 800'000'000U).reason
               == StationaryReason::accelerometerConfidenceTooLow,
           "low confidence is not stationary");
    detector.reset();
    static_cast<void>(detector.update({}, acceleration(0, 0, 1), 1.0, 100U));
    expect(detector.update({}, acceleration(0, 0, 1), 1.0, 100U).reason
               == StationaryReason::invalidTimestamp,
           "duplicate timestamp does not advance stationary duration");
    expect(detector.update({}, acceleration(0, 0, 1), 1.0, 99U).reason
               == StationaryReason::invalidTimestamp,
           "decreasing timestamp is rejected");

    DriftAccumulator drift;
    drift.consume({}, true, 0U);
    drift.consume(rotation(1, 0, 0, 10), true, 1'000'000'000U);
    const auto metrics = drift.metrics();
    expect(metrics.available && near(metrics.totalDegrees, 10.0)
               && near(metrics.totalDegreesPerSecond, 10.0),
           "drift and rate use device duration");
    DriftAccumulator unavailable;
    unavailable.consume({}, false, 0U);
    expect(!unavailable.metrics().available, "insufficient stationary samples are unavailable");
    DriftAccumulator zeroDrift;
    zeroDrift.consume({}, true, 0U);
    zeroDrift.consume({-1, 0, 0, 0}, true, 1'000'000'000U);
    expect(zeroDrift.metrics().available && near(zeroDrift.metrics().totalDegrees, 0.0),
           "q sign changes do not create artificial drift");
    DriftAccumulator yawDrift;
    yawDrift.consume({}, true, 0U);
    yawDrift.consume(rotation(0, 0, 1, 20), true, 1'000'000'000U);
    expect(near(yawDrift.metrics().tiltDegrees, 0.0), "tilt drift excludes pure yaw");

    const double thresholds[]{5.0, 2.0, 1.0};
    TiltConvergenceTracker convergence({}, thresholds, 500ms);
    convergence.consume(rotation(1, 0, 0, 4), 0U);
    convergence.consume(rotation(1, 0, 0, 6), 250'000'000U);
    convergence.consume(rotation(1, 0, 0, 4), 500'000'000U);
    convergence.consume(rotation(1, 0, 0, 4), 1'000'000'000U);
    expect(convergence.results()[0].reachedAfterSeconds == 0.5,
           "temporary crossing does not count and sustained convergence uses device time");
    expect(!convergence.results()[1].reachedAfterSeconds.has_value(),
           "unreached threshold remains unavailable");
    TiltConvergenceTracker yawConvergence({}, thresholds, 500ms);
    yawConvergence.consume(rotation(0, 0, 1, 90), 0U);
    yawConvergence.consume(rotation(0, 0, 1, 90), 500'000'000U);
    expect(yawConvergence.results()[0].reachedAfterSeconds == 0.0,
           "yaw does not affect sustained tilt convergence");
}

void testPhasesCsvAndJson()
{
    using namespace xreal::sensors;
    OrientationExperimentConfig experiment;
    expect(experimentPhaseAt(0s, experiment) == ExperimentPhase::stationaryBefore
               && experimentPhaseAt(7s, experiment) == ExperimentPhase::motion
               && experimentPhaseAt(19s, experiment) == ExperimentPhase::stationaryAfter,
           "experiment phases follow configured device durations");

    OrientationComparisonRecord record;
    record.deviceTimestampNanoseconds = 123;
    record.elapsedSeconds = 1.25;
    record.phase = ExperimentPhase::motion;
    record.stationary.reason = StationaryReason::angularSpeedTooHigh;
    record.acceleration = acceleration(0, 0, 1);
    record.absoluteDifference = compareOrientations({}, rotation(1, 0, 0, 10));
    record.relativeDifference = record.absoluteDifference;
    const auto header = orientationComparisonCsvHeader();
    const auto row = serializeOrientationComparisonCsvRow(record);
    expect(header.find("gyro_absolute_w") != std::string::npos
               && header.find("fused_relative_w") != std::string::npos
               && header.find("tilt_difference_degrees") != std::string::npos,
           "CSV header contains absolute, relative and difference fields");
    expect(row.find("\"motion\"") != std::string::npos
               && row.find("\"angular_speed_too_high\"") != std::string::npos
               && row.find("1.25") != std::string::npos,
           "CSV row is locale-independent and quotes textual fields");
    expect(std::count(header.begin(), header.end(), ',')
               == std::count(row.begin(), row.end(), ','),
           "CSV row parses to the same column count as its header");

    OrientationComparisonSummary summary;
    summary.gyroscopeProfileSource = "experimental-4090";
    summary.accelerometerProfileSource = "profile\"with-quote";
    summary.finalAbsoluteDifference = record.absoluteDifference;
    summary.finalRelativeDifference = record.relativeDifference;
    summary.gyroConvergence = {{5.0, 1.0}, {2.0, std::nullopt}};
    const auto json = serializeOrientationComparisonJson(summary);
    expect(JsonSyntaxParser(json).valid(), "JSON summary parses with the test syntax parser");
    expect(json.find("gyro-vs-fusion-orientation-comparison") != std::string::npos
               && json.find("quaternion_convention") != std::string::npos
               && json.find("stationary_before") != std::string::npos
               && json.find("stationary_gyro_threshold_dps") != std::string::npos
               && json.find("start_device_timestamp_ns") != std::string::npos
               && json.find("gyro_recovery") != std::string::npos
               && json.find("fused_absolute_quaternion_wxyz") != std::string::npos
               && json.find("profile\\\"with-quote") != std::string::npos
               && json.find("\"experimental\":true") != std::string::npos,
           "JSON contains schema, provenance, phases, frames and escaped strings");
}

} // namespace

int main()
{
    testFramesAndDistances();
    testParallelEngineAndRecenter();
    testStationaryDriftAndConvergence();
    testPhasesCsvAndJson();
    if (failures != 0)
    {
        std::cerr << failures << " orientation comparison test(s) failed.\n";
        return 1;
    }
    std::cout << "All orientation comparison tests passed.\n";
    return 0;
}
