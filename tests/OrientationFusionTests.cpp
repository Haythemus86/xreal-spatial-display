#include "sensors/AccelerometerPhysicalUnits.hpp"
#include "sensors/OrientationFusion.hpp"

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <numbers>
#include <string>
#include <string_view>

namespace
{

using namespace std::chrono_literals;
int failures{};
std::string fusionJson;

void expect(bool condition, std::string_view description)
{
    if (!condition)
    {
        ++failures;
        std::cerr << "FAILED: " << description << '\n';
    }
}

[[nodiscard]] bool near(double actual, double expected, double tolerance = 1.0e-8)
{
    return std::abs(actual - expected) <= tolerance;
}

[[nodiscard]] bool equivalent(
    const xreal::sensors::Quaternion& left,
    const xreal::sensors::Quaternion& right,
    double tolerance = 1.0e-7)
{
    const bool same = near(left.w, right.w, tolerance) && near(left.x, right.x, tolerance)
        && near(left.y, right.y, tolerance) && near(left.z, right.z, tolerance);
    const bool opposite = near(left.w, -right.w, tolerance) && near(left.x, -right.x, tolerance)
        && near(left.y, -right.y, tolerance) && near(left.z, -right.z, tolerance);
    return same || opposite;
}

[[nodiscard]] xreal::sensors::AccelerometerCalibrationProfile profile()
{
    xreal::sensors::AccelerometerCalibrationProfile result;
    result.x = {10.0, 1000.0, true};
    result.y = {-20.0, 2000.0, true};
    result.z = {30.0, 4000.0, true};
    result.source = "synthetic-profile";
    return result;
}

[[nodiscard]] xreal::sensors::AccelerometerPhysicalSample acceleration(
    double x,
    double y,
    double z,
    double normOverride = -1.0)
{
    xreal::sensors::AccelerometerPhysicalSample result;
    result.accelerationG = {x, y, z};
    result.accelerationMetersPerSecondSquared = {
        x * xreal::sensors::standardGravityMetersPerSecondSquared,
        y * xreal::sensors::standardGravityMetersPerSecondSquared,
        z * xreal::sensors::standardGravityMetersPerSecondSquared,
    };
    result.normG = normOverride >= 0.0 ? normOverride : result.accelerationG.norm();
    result.valid = result.accelerationG.finite() && std::isfinite(result.normG);
    return result;
}

[[nodiscard]] xreal::sensors::OrientationFusionConfig fastConfig()
{
    xreal::sensors::OrientationFusionConfig config;
    config.correctionTimeConstant = 0.25s;
    config.maximumCorrectionDegreesPerSecond = 180.0;
    config.confidence.smoothingTimeConstant = 0s;
    return config;
}

void testPhysicalUnitsAndProfiles()
{
    using namespace xreal::sensors;
    const auto calibration = profile();
    const RawImuVector3 raw{1010, -2020, -3970};
    const auto converted = convertAccelerometerToPhysicalUnits(raw, calibration);
    expect(converted.valid && near(converted.correctedRaw.x, 1000.0)
               && near(converted.correctedRaw.y, -2000.0)
               && near(converted.correctedRaw.z, -4000.0),
           "known offsets are subtracted and negative corrected values are preserved");
    expect(near(converted.accelerationG.x, 1.0)
               && near(converted.accelerationG.y, -1.0)
               && near(converted.accelerationG.z, -1.0),
           "different per-axis raw/g scales are applied");
    expect(near(converted.accelerationMetersPerSecondSquared.x, 9.80665),
           "g conversion uses named standard gravity 9.80665 m/s^2");
    expect(raw.x == 1010 && raw.y == -2020 && raw.z == -3970,
           "original raw accelerometer input is unchanged");

    auto oneGProfile = profile();
    const auto oneG = convertAccelerometerToPhysicalUnits({10, -20, 4030}, oneGProfile);
    expect(near(oneG.normG, 1.0), "synthetic calibrated one-g sample has norm one");

    for (const double invalid : {0.0, -1.0,
                                 std::numeric_limits<double>::quiet_NaN(),
                                 std::numeric_limits<double>::infinity()})
    {
        auto bad = profile();
        bad.x.rawUnitsPerG = invalid;
        expect(!validateAccelerometerCalibrationProfile(bad)
                   && !convertAccelerometerToPhysicalUnits(raw, bad).valid,
               "zero, negative and non-finite scales prevent conversion");
    }
    for (const double invalid : {std::numeric_limits<double>::quiet_NaN(),
                                 std::numeric_limits<double>::infinity()})
    {
        auto bad = profile();
        bad.y.offsetRaw = invalid;
        expect(!validateAccelerometerCalibrationProfile(bad)
                   && !convertAccelerometerToPhysicalUnits(raw, bad).valid,
               "non-finite accelerometer offsets prevent conversion");
    }

    auto mapping = profile();
    mapping.axisMapping.sensorX.logicalAxis = AccelerometerLogicalAxis::z;
    mapping.axisMapping.sensorZ.logicalAxis = AccelerometerLogicalAxis::x;
    mapping.axisMapping.sensorZ.sign = -1;
    const auto mapped = convertAccelerometerToPhysicalUnits({1010, -20, 4030}, mapping);
    expect(near(mapped.accelerationG.x, -1.0) && near(mapped.accelerationG.z, 1.0),
           "accelerometer axis permutation and sign inversion are applied");
    mapping.axisMapping.sensorY.logicalAxis = AccelerometerLogicalAxis::x;
    expect(!validateAccelerometerCalibrationProfile(mapping),
           "duplicate accelerometer mapping targets are rejected");

    auto air2Ultra = profile();
    air2Ultra.axisMapping = makeExperimentalXrealAir2UltraAccelerometerAxisMapping();
    const auto air2UltraMapped = convertAccelerometerToPhysicalUnits(
        {1010, 1980, 4030}, air2Ultra);
    expect(air2UltraMapped.valid && near(air2UltraMapped.accelerationG.x, 1.0)
               && near(air2UltraMapped.accelerationG.y, -1.0)
               && near(air2UltraMapped.accelerationG.z, 1.0),
           "Air 2 Ultra accelerometer mapping corrects pitch gravity while preserving other axes");
    expect(validateAccelerometerAxisMapping(air2Ultra.axisMapping)
               && air2Ultra.axisMapping.experimental && air2Ultra.axisMapping.verified,
           "Air 2 Ultra accelerometer mapping records successful hardware direction validation");
    const auto air2UltraPitchGravity = convertAccelerometerToPhysicalUnits(
        {10, -1020, 3494}, air2Ultra);
    const auto air2UltraPitchStartup = gravityAlignedStartupOrientation(
        air2UltraPitchGravity.accelerationG);
    expect(air2UltraPitchStartup.has_value()
               && near(quaternionToEulerDiagnostic(*air2UltraPitchStartup).rollDegrees,
                       30.0, 0.1),
           "Air 2 Ultra pitch gravity initializes with the corrected positive rotation sign");

    const std::string json = "{\"schema_version\":1,\"offset_raw\":{\"x\":10,\"y\":-20,\"z\":30},"
                             "\"raw_units_per_g\":{\"x\":1000,\"y\":2000,\"z\":4000}}";
    const auto loaded = loadAccelerometerCalibrationProfileJson(json, "synthetic.json");
    expect(loaded.profile.has_value() && loaded.profile->source == "synthetic.json"
               && near(loaded.profile->z.rawUnitsPerG, 4000.0),
           "runtime accelerometer profile loads explicit offsets, scales and provenance");
    expect(!loadAccelerometerCalibrationProfileJson("{}", "bad.json").profile.has_value(),
           "missing accelerometer calibration fields are rejected without fallback");

    const std::string hardwareProfileJson =
        "{\"schema_version\":1,\"model\":\"axis_aligned_ellipsoid\","
        "\"units\":{\"offset\":\"raw\",\"scale\":\"raw_units_per_g\","
        "\"corrected_magnitude\":\"g\"},"
        "\"offset_raw\":{\"x\":-977.40765288074635,"
        "\"y\":594.07200886237661,\"z\":15221.033085708183},"
        "\"raw_units_per_g\":{\"x\":524628.92912459513,"
        "\"y\":524758.38647404185,\"z\":524390.1036666123}}";
    const auto hardwareProfile = loadAccelerometerCalibrationProfileJson(
        hardwareProfileJson,
        "xreal-air2-ultra-calibration.json");
    expect(hardwareProfile.profile.has_value(),
           "accepted Air 2 Ultra ellipsoid profile loads for runtime fusion");
}

void testConfidence()
{
    using namespace xreal::sensors;
    GravityConfidenceConfig config;
    config.smoothingTimeConstant = 0s;
    AccelerometerConfidenceEstimator estimator(config);
    expect(near(estimator.update(acceleration(0, 0, 1), 0U).correctionConfidence, 1.0),
           "exactly one g has full confidence");
    expect(near(estimator.update(acceleration(0, 0, 1.04), 1'000'000U).rawConfidence, 1.0),
           "small norm deviation keeps full confidence");
    const auto partial = estimator.update(acceleration(0, 0, 1.125), 2'000'000U);
    expect(partial.rawConfidence > 0.0 && partial.rawConfidence < 1.0,
           "intermediate norm deviation has partial confidence");
    expect(near(estimator.update(acceleration(0, 0, 1.3), 3'000'000U).correctionConfidence, 0.0),
           "large norm deviation has zero correction confidence");
    expect(!estimator.update(acceleration(0, 0, 0), 4'000'000U).valid,
           "near-zero acceleration is rejected");
    auto invalid = acceleration(0, 0, 1);
    invalid.accelerationG.x = std::numeric_limits<double>::quiet_NaN();
    expect(!estimator.update(invalid, 5'000'000U).valid,
           "non-finite acceleration is rejected");

    config.smoothingTimeConstant = 1s;
    AccelerometerConfidenceEstimator smoothed(config);
    smoothed.update(acceleration(0, 0, 1.3), 0U);
    const auto regular = smoothed.update(acceleration(0, 0, 1), 500'000'000U);
    expect(near(regular.smoothedConfidence, 1.0 - std::exp(-0.5), 1.0e-9),
           "confidence smoothing is time-based");
    AccelerometerConfidenceEstimator irregular(config);
    irregular.update(acceleration(0, 0, 1.3), 0U);
    irregular.update(acceleration(0, 0, 1), 200'000'000U);
    const auto split = irregular.update(acceleration(0, 0, 1), 500'000'000U);
    expect(near(split.smoothedConfidence, regular.smoothedConfidence, 1.0e-9),
           "irregular timestamps produce equivalent confidence smoothing");

    config.smoothingTimeConstant = 0s;
    config.maximumJerkGPerSecond = 2.0;
    AccelerometerConfidenceEstimator jerk(config);
    jerk.update(acceleration(0, 0, 1), 0U);
    expect(jerk.update(acceleration(0.5, 0, std::sqrt(0.75)), 10'000'000U).reason
               == FusionReason::jerkExceeded,
           "optional jerk rejection suppresses sudden direction change");

    config.maximumJerkGPerSecond.reset();
    config.hysteresisEnabled = true;
    AccelerometerConfidenceEstimator hysteresis(config);
    expect(hysteresis.update(acceleration(0, 0, 1.15), 0U).correctionConfidence == 0.0,
           "confidence hysteresis waits below its entry threshold");
    expect(hysteresis.update(acceleration(0, 0, 1), 1'000'000U).correctionConfidence == 1.0,
           "confidence hysteresis enters deterministically");
}

void testGravityPredictionAndStartup()
{
    using namespace xreal::sensors;
    const double half = std::sqrt(0.5);
    const auto identity = predictBodyFrameAccelerometerUp({});
    expect(near(identity.x, 0.0) && near(identity.y, 0.0) && near(identity.z, 1.0),
           "identity predicts body-frame accelerometer up as +Z");
    const auto rollPositive = predictBodyFrameAccelerometerUp({half, half, 0, 0});
    const auto rollNegative = predictBodyFrameAccelerometerUp({half, -half, 0, 0});
    expect(near(rollPositive.y, 1.0) && near(rollNegative.y, -1.0),
           "positive and negative 90-degree roll predict opposite body Y gravity");
    const auto pitchPositive = predictBodyFrameAccelerometerUp({half, 0, half, 0});
    const auto pitchNegative = predictBodyFrameAccelerometerUp({half, 0, -half, 0});
    expect(near(pitchPositive.x, -1.0) && near(pitchNegative.x, 1.0),
           "positive and negative 90-degree pitch predict body X gravity");
    const auto yaw = predictBodyFrameAccelerometerUp({half, 0, 0, half});
    expect(near(yaw.x, 0.0) && near(yaw.y, 0.0) && near(yaw.z, 1.0),
           "pure yaw does not change predicted gravity and prediction remains normalized");

    const double angle = 30.0 * std::numbers::pi / 180.0;
    const auto startup = gravityAlignedStartupOrientation({0.0, std::sin(angle), std::cos(angle)});
    expect(startup.has_value() && near(quaternionToEulerDiagnostic(*startup).rollDegrees, 30.0)
               && near(quaternionToEulerDiagnostic(*startup).yawDegrees, 0.0),
           "gravity startup initializes known roll while preserving yaw zero");
    const auto pitchStartup = gravityAlignedStartupOrientation(
        {-std::sin(angle), 0.0, std::cos(angle)});
    expect(pitchStartup.has_value()
               && near(quaternionToEulerDiagnostic(*pitchStartup).pitchDegrees, 30.0),
           "gravity startup initializes known pitch and normalized quaternion");
}

void testFusionBehavior()
{
    using namespace xreal::sensors;
    auto config = fastConfig();
    OrientationFusionFilter level(config);
    level.update({}, acceleration(0, 0, 1), 0U);
    for (std::uint64_t timestamp = 10'000'000U; timestamp <= 1'000'000'000U;
         timestamp += 10'000'000U)
    {
        level.update({}, acceleration(0, 0, 1), timestamp);
    }
    expect(equivalent(level.orientation(), {}) && near(level.orientation().norm(), 1.0),
           "stationary level fusion preserves normalized identity");

    OrientationFusionFilter yaw(config);
    yaw.update({0, 0, std::numbers::pi / 2.0}, acceleration(0, 0, 1), 0U);
    for (std::uint64_t timestamp = 10'000'000U; timestamp <= 1'000'000'000U;
         timestamp += 10'000'000U)
    {
        yaw.update({0, 0, std::numbers::pi / 2.0}, acceleration(0, 0, 1), timestamp);
    }
    const auto yawEuler = quaternionToEulerDiagnostic(yaw.orientation());
    expect(near(yawEuler.yawDegrees, 90.0, 1.0e-5)
               && near(yawEuler.pitchDegrees, 0.0, 1.0e-5),
           "accelerometer correction does not create or cancel absolute yaw");

    const double twenty = 20.0 * std::numbers::pi / 180.0;
    OrientationFusionFilter roll(config);
    roll.setOrientation({std::cos(twenty / 2.0), std::sin(twenty / 2.0), 0, 0});
    roll.update({}, acceleration(0, 0, 1), 0U);
    for (std::uint64_t timestamp = 10'000'000U; timestamp <= 3'000'000'000U;
         timestamp += 10'000'000U)
    {
        roll.update({}, acceleration(0, 0, 1), timestamp);
    }
    expect(std::abs(quaternionToEulerDiagnostic(roll.orientation()).rollDegrees) < 1.0,
           "synthetic roll error converges toward measured gravity");

    OrientationFusionFilter pitch(config);
    pitch.setOrientation({std::cos(twenty / 2.0), 0, std::sin(twenty / 2.0), 0});
    pitch.update({}, acceleration(0, 0, 1), 0U);
    for (std::uint64_t timestamp = 10'000'000U; timestamp <= 3'000'000'000U;
         timestamp += 10'000'000U)
    {
        pitch.update({}, acceleration(0, 0, 1), timestamp);
    }
    expect(std::abs(quaternionToEulerDiagnostic(pitch.orientation()).pitchDegrees) < 1.0,
           "synthetic pitch error converges toward measured gravity");

    const Quaternion combinedStart = Quaternion{
        std::cos(twenty / 2.0), std::sin(twenty / 2.0), 0, 0}
        * Quaternion{std::cos(twenty / 2.0), 0, std::sin(twenty / 2.0), 0};
    OrientationFusionFilter combined(config);
    expect(combined.setOrientation(combinedStart), "combined synthetic tilt is accepted");
    const auto combinedInput = acceleration(0, 0, 1);
    combined.update({}, combinedInput, 0U);
    for (std::uint64_t timestamp = 10'000'000U; timestamp <= 3'000'000'000U;
         timestamp += 10'000'000U)
    {
        combined.update({}, combinedInput, timestamp);
    }
    const auto combinedEuler = quaternionToEulerDiagnostic(combined.orientation());
    expect(std::abs(combinedEuler.rollDegrees) < 1.0
               && std::abs(combinedEuler.pitchDegrees) < 1.0
               && combinedInput.accelerationG.z == 1.0,
           "combined pitch/roll converges without modifying accelerometer input");

    OrientationFusionFilter fullConfidence(config);
    OrientationFusionFilter partialConfidence(config);
    fullConfidence.setOrientation({std::cos(twenty / 2.0), std::sin(twenty / 2.0), 0, 0});
    partialConfidence.setOrientation({std::cos(twenty / 2.0), std::sin(twenty / 2.0), 0, 0});
    fullConfidence.update({}, acceleration(0, 0, 1), 0U);
    partialConfidence.update({}, acceleration(0, 0, 1.125), 0U);
    const auto fullResult = fullConfidence.update({}, acceleration(0, 0, 1), 10'000'000U);
    const auto partialResult = partialConfidence.update(
        {}, acceleration(0, 0, 1.125), 10'000'000U);
    expect(fullResult.appliedCorrectionAngleRadians > partialResult.appliedCorrectionAngleRadians
               && partialResult.appliedCorrectionAngleRadians > 0.0,
           "partial confidence applies less correction than full confidence");

    auto disabledConfig = config;
    disabledConfig.accelerometerCorrectionEnabled = false;
    OrientationFusionFilter disabled(disabledConfig);
    disabled.setOrientation({std::cos(twenty / 2.0), std::sin(twenty / 2.0), 0, 0});
    disabled.update({}, acceleration(0, 0, 1), 0U);
    const auto disabledResult = disabled.update({}, acceleration(0, 0, 1), 10'000'000U);
    expect(disabledResult.reason == FusionReason::correctionDisabled
               && disabledResult.appliedCorrectionAngleRadians == 0.0,
           "accelerometer correction can be explicitly disabled");

    auto limitedConfig = fastConfig();
    limitedConfig.maximumCorrectionDegreesPerSecond = 1.0;
    OrientationFusionFilter limited(limitedConfig);
    limited.setOrientation({std::cos(twenty / 2.0), std::sin(twenty / 2.0), 0, 0});
    limited.update({}, acceleration(0, 0, 1), 0U);
    const auto limitedResult = limited.update({}, acceleration(0, 0, 1), 10'000'000U);
    expect(limitedResult.appliedCorrectionAngleRadians
               <= 1.0 * std::numbers::pi / 180.0 * 0.01 + 1.0e-12,
           "correction respects configured maximum degrees per second");

    const auto beforeDynamic = limited.orientation();
    const auto dynamic = limited.update({}, acceleration(0, 0, 2), 20'000'000U);
    expect(dynamic.correctionStatus == FusionCorrectionStatus::skipped
               && equivalent(limited.orientation(), beforeDynamic),
           "dynamic two-g acceleration applies no tilt correction or jump");
    expect(limited.update({}, acceleration(0, 0, 1), 30'000'000U).correctionStatus
               == FusionCorrectionStatus::applied,
           "correction resumes progressively after return to one g");

    OrientationFusionFilter antiParallel(config);
    antiParallel.update({}, acceleration(0, 0, -1), 0U);
    const auto anti = antiParallel.update({}, acceleration(0, 0, -1), 10'000'000U);
    expect(anti.reason == FusionReason::antiParallelGravity
               && antiParallel.orientation().finite(),
           "anti-parallel gravity ambiguity is rejected safely");

    OrientationFusionFilter timing(config);
    timing.update({}, acceleration(0, 0, 1), 100U);
    const auto duplicate = timing.update({}, acceleration(0, 0, 1), 100U);
    const auto decreasing = timing.update({}, acceleration(0, 0, 1), 50U);
    const auto excessive = timing.update({}, acceleration(0, 0, 1), 100'000'100U);
    expect(duplicate.reason == FusionReason::duplicateTimestamp
               && decreasing.reason == FusionReason::decreasingTimestamp
               && excessive.reason == FusionReason::excessiveTimestampDelta,
           "duplicate, decreasing and excessive device times follow gyro timing policy");

    OrientationFusionFilter longRun(config);
    longRun.update({0.001, -0.002, 0.003}, acceleration(0, 0, 1), 0U);
    for (std::uint64_t timestamp = 1'000'000U; timestamp <= 60'000'000'000U;
         timestamp += 1'000'000U)
    {
        longRun.update({0.001, -0.002, 0.003}, acceleration(0, 0, 1), timestamp);
    }
    expect(longRun.orientation().finite() && near(longRun.orientation().norm(), 1.0, 1.0e-10),
           "long 1000 Hz fusion run remains finite and normalized");
}

void testStartupRecenterDriftAndJson()
{
    using namespace xreal::sensors;
    auto gravityConfig = fastConfig();
    gravityConfig.startupMode = FusionStartupMode::gravity;
    OrientationFusionFilter startup(gravityConfig);
    startup.update({}, acceleration(0, 0, 1.3), 0U);
    expect(!startup.state().initialized
               && startup.state().lastReason == FusionReason::startupWaitingForGravity,
           "low-confidence dynamic acceleration delays gravity startup");
    startup.update({}, acceleration(0, 0, 1), 10'000'000U);
    expect(startup.state().initialized && startup.state().gravityStartupApplied
               && startup.state().startupStatus == FusionStartupStatus::gravityAligned
               && equivalent(startup.orientation(), {}),
           "valid gravity initializes normalized level orientation");

    OrientationFusionFilter tiltedStartup(gravityConfig);
    const double startupAngle = 25.0 * std::numbers::pi / 180.0;
    tiltedStartup.update({}, acceleration(0, std::sin(startupAngle), std::cos(startupAngle)), 0U);
    expect(near(quaternionToEulerDiagnostic(tiltedStartup.orientation()).rollDegrees, 25.0)
               && near(quaternionToEulerDiagnostic(tiltedStartup.orientation()).yawDegrees, 0.0),
           "gravity startup initializes a tilted pose with yaw zero");

    OrientationFusionFilter recentered(fastConfig());
    const double angle = 20.0 * std::numbers::pi / 180.0;
    recentered.setOrientation({std::cos(angle / 2), std::sin(angle / 2), 0, 0});
    expect(recentered.recenter() && equivalent(recentered.relativeOrientation(), {}),
           "recenter after fused tilt produces relative identity");
    recentered.update({}, acceleration(0, 0, 1), 0U);
    for (std::uint64_t timestamp = 10'000'000U; timestamp <= 1'000'000'000U;
         timestamp += 10'000'000U)
    {
        recentered.update({}, acceleration(0, 0, 1), timestamp);
    }
    expect(recentered.state().appliedAccelerometerCorrectionCount > 0U
               && near(recentered.relativeOrientation().norm(), 1.0)
               && !equivalent(recentered.relativeOrientation(), recentered.orientation()),
           "gravity correction continues after recenter without invalidating relative output");
    recentered.clearRecenter();
    expect(equivalent(recentered.relativeOrientation(), recentered.orientation()),
           "clearing recenter restores absolute fused orientation");

    auto driftConfig = fastConfig();
    GyroscopeOrientationIntegrator gyroOnly;
    OrientationFusionFilter fused(driftConfig);
    const AngularVelocityRadians bias{0.01, 0.0, 0.0};
    gyroOnly.update(bias, 0U);
    fused.update(bias, acceleration(0, 0, 1), 0U);
    for (std::uint64_t timestamp = 10'000'000U; timestamp <= 10'000'000'000U;
         timestamp += 10'000'000U)
    {
        gyroOnly.update(bias, timestamp);
        fused.update(bias, acceleration(0, 0, 1), timestamp);
    }
    const double gyroRoll = std::abs(quaternionToEulerDiagnostic(gyroOnly.orientation()).rollDegrees);
    const double fusedRoll = std::abs(quaternionToEulerDiagnostic(fused.orientation()).rollDegrees);
    expect(fusedRoll < gyroRoll * 0.25,
           "complementary fusion reduces stationary roll drift caused by gyro bias");

    const auto gyroScale = makeExperimentalGyroscopeScaleProfile(4090.0);
    const auto accelProfile = profile();
    fusionJson = serializeOrientationFusionJson(fused, gyroScale, accelProfile, true);
    expect(fusionJson.find("\"gyro-accelerometer-complementary\"") != std::string::npos
               && fusionJson.find("\"quaternion_convention\"") != std::string::npos
               && fusionJson.find("\"accelerometer_profile\"") != std::string::npos
               && fusionJson.find("\"final_confidence\"") != std::string::npos
               && fusionJson.find("\"experimental\":true") != std::string::npos,
           "fusion JSON contains conventions, provenance, confidence and correction statistics");
}

} // namespace

int main(int argc, char* argv[])
{
    testPhysicalUnitsAndProfiles();
    testConfidence();
    testGravityPredictionAndStartup();
    testFusionBehavior();
    testStartupRecenterDriftAndJson();
    if (argc == 3 && std::string_view(argv[1]) == "--json-output")
    {
        std::ofstream output(argv[2], std::ios::out | std::ios::trunc);
        output << fusionJson;
        expect(static_cast<bool>(output), "fusion JSON test artifact is written");
    }
    if (failures != 0)
    {
        std::cerr << failures << " orientation fusion test(s) failed.\n";
        return 1;
    }
    std::cout << "All orientation fusion tests passed.\n";
    return 0;
}
