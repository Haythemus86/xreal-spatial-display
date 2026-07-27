#include "sensors/GyroscopeOrientation.hpp"

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <numbers>
#include <string_view>

namespace
{

using namespace std::chrono_literals;
int failures{};
std::string serializedOrientationJson;

void expect(bool condition, std::string_view description)
{
    if (!condition)
    {
        ++failures;
        std::cerr << "FAILED: " << description << '\n';
    }
}

[[nodiscard]] bool near(double actual, double expected, double tolerance = 1.0e-9)
{
    return std::abs(actual - expected) <= tolerance;
}

[[nodiscard]] bool equivalent(
    const xreal::sensors::Quaternion& left,
    const xreal::sensors::Quaternion& right,
    double tolerance = 1.0e-8)
{
    const bool same = near(left.w, right.w, tolerance) && near(left.x, right.x, tolerance)
        && near(left.y, right.y, tolerance) && near(left.z, right.z, tolerance);
    const bool negated = near(left.w, -right.w, tolerance) && near(left.x, -right.x, tolerance)
        && near(left.y, -right.y, tolerance) && near(left.z, -right.z, tolerance);
    return same || negated;
}

[[nodiscard]] xreal::sensors::Quaternion integrate(
    xreal::sensors::AngularVelocityRadians velocity,
    std::uint64_t durationNanoseconds,
    std::uint64_t stepNanoseconds)
{
    xreal::sensors::GyroscopeOrientationIntegrator integrator;
    integrator.update(velocity, 0U);
    for (std::uint64_t timestamp = stepNanoseconds; timestamp <= durationNanoseconds;
         timestamp += stepNanoseconds)
    {
        integrator.update(velocity, timestamp);
    }
    return integrator.orientation();
}

void testQuaternionMath()
{
    using xreal::sensors::Quaternion;
    const Quaternion identity;
    expect(near(identity.norm(), 1.0), "identity quaternion has unit norm");
    const auto normalized = Quaternion{2.0, 0.0, 0.0, 0.0}.normalized();
    expect(normalized.has_value() && equivalent(*normalized, identity),
           "normalization produces unit identity");
    expect(!Quaternion{0.0, 0.0, 0.0, 0.0}.normalized().has_value(),
           "zero quaternion normalization is rejected");
    expect(!Quaternion{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.0}.finite(),
           "non-finite quaternion is invalid");
    const Quaternion quarterTurn{std::sqrt(0.5), std::sqrt(0.5), 0.0, 0.0};
    expect(equivalent(identity * quarterTurn, quarterTurn)
               && equivalent(quarterTurn * identity, quarterTurn),
           "identity multiplication works on both sides");
    expect(equivalent(quarterTurn * quarterTurn.conjugate(), identity),
           "conjugate cancels a normalized quaternion");
    expect(quarterTurn.inverseNormalized().has_value()
               && equivalent(*quarterTurn.inverseNormalized(), quarterTurn.conjugate()),
           "normalized inverse is the conjugate");
    const Quaternion xTurn{std::sqrt(0.5), std::sqrt(0.5), 0.0, 0.0};
    const Quaternion zTurn{std::sqrt(0.5), 0.0, 0.0, std::sqrt(0.5)};
    expect(!equivalent(xTurn * zTurn, zTurn * xTurn),
           "multiplication order is explicit and non-commutative");
}

void testIntegration()
{
    using xreal::sensors::AngularVelocityRadians;
    using xreal::sensors::GyroscopeOrientationIntegrator;
    using xreal::sensors::OrientationSampleStatus;
    xreal::sensors::OrientationIntegratorConfig firstConfiguration;
    firstConfiguration.maximumDeviceTimestampDelta = 2s;
    GyroscopeOrientationIntegrator first(firstConfiguration);
    const auto firstResult = first.update({}, 100U);
    expect(firstResult.status == OrientationSampleStatus::skipped
               && first.state().appliedSampleCount == 0U,
           "first timestamp initializes without rotation");
    first.update({}, 1'000'000'100U);
    expect(equivalent(first.orientation(), {}), "zero angular velocity preserves identity");

    const double half = std::sqrt(0.5);
    expect(equivalent(integrate({std::numbers::pi / 2.0, 0.0, 0.0}, 1'000'000'000U, 10'000'000U),
                      {half, half, 0.0, 0.0}),
           "positive X 90-degree rotation integrates correctly");
    expect(equivalent(integrate({0.0, std::numbers::pi / 2.0, 0.0}, 1'000'000'000U, 10'000'000U),
                      {half, 0.0, half, 0.0}),
           "positive Y 90-degree rotation integrates correctly");
    expect(equivalent(integrate({0.0, 0.0, std::numbers::pi / 2.0}, 1'000'000'000U, 10'000'000U),
                      {half, 0.0, 0.0, half}),
           "positive Z 90-degree rotation integrates correctly");
    expect(equivalent(integrate({0.0, 0.0, -std::numbers::pi / 2.0}, 1'000'000'000U, 10'000'000U),
                      {half, 0.0, 0.0, -half}),
           "negative rotation preserves sign");
    expect(equivalent(integrate({0.0, 0.0, std::numbers::pi}, 1'000'000'000U, 10'000'000U),
                      {0.0, 0.0, 0.0, 1.0}),
           "180-degree rotation integrates correctly");
    expect(equivalent(integrate({0.0, 0.0, 2.0 * std::numbers::pi}, 1'000'000'000U, 10'000'000U),
                      xreal::sensors::Quaternion::identity()),
           "360-degree rotation is sign-equivalent to identity");

    GyroscopeOrientationIntegrator irregular;
    irregular.update({0.0, 0.0, 1.0}, 0U);
    irregular.update({0.0, 0.0, 1.0}, 3'000'000U);
    irregular.update({0.0, 0.0, 1.0}, 11'000'000U);
    irregular.update({0.0, 0.0, 1.0}, 20'000'000U);
    expect(equivalent(irregular.orientation(), {std::cos(0.01), 0.0, 0.0, std::sin(0.01)}),
           "irregular timestamps integrate total device time without assuming 1000 Hz");

    GyroscopeOrientationIntegrator small;
    small.update({1.0e-10, 0.0, 0.0}, 0U);
    small.update({1.0e-10, 0.0, 0.0}, 1'000'000U);
    expect(small.orientation().finite() && near(small.orientation().norm(), 1.0),
           "small-angle path remains finite and normalized");
    const auto original = AngularVelocityRadians{1.0, 2.0, 3.0};
    auto input = original;
    small.update(input, 2'000'000U);
    expect(input.xRadiansPerSecond == original.xRadiansPerSecond
               && input.yRadiansPerSecond == original.yRadiansPerSecond,
           "angular velocity input is not modified");

    GyroscopeOrientationIntegrator longRun;
    longRun.update({0.2, -0.3, 0.4}, 0U);
    for (std::uint64_t index = 1; index <= 100'000U; ++index)
    {
        longRun.update({0.2, -0.3, 0.4}, index * 1'000'000U);
    }
    expect(longRun.orientation().finite() && near(longRun.orientation().norm(), 1.0, 1.0e-12),
           "long synthetic integration remains finite and normalized");

    GyroscopeOrientationIntegrator wrapped;
    wrapped.update({}, std::numeric_limits<std::uint64_t>::max() - 5U);
    const auto wrapResult = wrapped.update({}, 4U);
    expect(wrapResult.status == OrientationSampleStatus::applied
               && wrapResult.deltaTime == 10ns,
           "device timestamp wraparound follows the shared forward-delta policy");
}

void testRejectionsAndCounters()
{
    using xreal::sensors::GyroscopeOrientationIntegrator;
    using xreal::sensors::OrientationSampleStatus;
    GyroscopeOrientationIntegrator integrator;
    integrator.update({}, 100U);
    const auto duplicate = integrator.update({}, 100U);
    const auto before = integrator.orientation();
    const auto decreasing = integrator.update({}, 50U);
    const auto excessive = integrator.update({}, 100'000'100U);
    const auto invalid = integrator.update(
        {std::numeric_limits<double>::infinity(), 0.0, 0.0}, 101'000'100U);
    const auto applied = integrator.update({}, 102'000'100U);
    expect(duplicate.status == OrientationSampleStatus::skipped
               && decreasing.status == OrientationSampleStatus::rejected
               && excessive.status == OrientationSampleStatus::rejected
               && invalid.status == OrientationSampleStatus::rejected,
           "duplicate, decreasing, excessive and invalid samples follow rejection policy");
    expect(equivalent(integrator.orientation(), before)
               && integrator.state().skippedSampleCount == 2U
               && integrator.state().rejectedSampleCount == 3U
               && applied.status == OrientationSampleStatus::applied
               && integrator.state().appliedSampleCount == 1U,
           "rejections preserve orientation and update counters");
}

void testMappingRecenterAndEuler()
{
    using namespace xreal::sensors;
    GyroscopeAxisMapping mapping;
    expect(validateGyroscopeAxisMapping(mapping), "identity mapping is valid");
    expect(mapAngularVelocity({1.0, 2.0, 3.0}, mapping).zRadiansPerSecond == 3.0,
           "identity mapping preserves sensor axes");
    mapping.sensorX.logicalAxis = GyroscopeLogicalAxis::sensorZ;
    mapping.sensorZ.logicalAxis = GyroscopeLogicalAxis::sensorX;
    mapping.sensorZ.sign = -1;
    const auto mapped = mapAngularVelocity({1.0, 2.0, 3.0}, mapping);
    expect(mapped.xRadiansPerSecond == -3.0 && mapped.zRadiansPerSecond == 1.0,
           "axis permutation and sign inversion work");
    mapping.sensorY.logicalAxis = GyroscopeLogicalAxis::sensorX;
    expect(!validateGyroscopeAxisMapping(mapping), "duplicated logical axes are rejected");
    mapping.sensorY.logicalAxis = GyroscopeLogicalAxis::sensorY;
    mapping.sensorZ.sign = 2;
    expect(!validateGyroscopeAxisMapping(mapping), "invalid mapping sign is rejected");
    OrientationIntegratorConfig invalidMappingConfiguration;
    invalidMappingConfiguration.axisMapping = mapping;
    GyroscopeOrientationIntegrator invalidMappingIntegrator(invalidMappingConfiguration);
    expect(invalidMappingIntegrator.update({}, 0U).status == OrientationSampleStatus::rejected,
           "integrator rejects an invalid axis mapping without changing orientation");

    GyroscopeAxisMapping provisional;
    provisional.experimental = true;
    provisional.verified = false;
    provisional.source = "flat-chair-yaw-observation";
    provisional.notes = "Sensor Z was dominant; yaw remains provisional.";
    expect(provisional.experimental && !provisional.verified
               && provisional.source == "flat-chair-yaw-observation",
           "mapping provenance and verification metadata are preserved");

    OrientationIntegratorConfig yawMappingConfig;
    yawMappingConfig.maximumDeviceTimestampDelta = 2s;
    yawMappingConfig.axisMapping.sensorX.logicalAxis = GyroscopeLogicalAxis::sensorX;
    yawMappingConfig.axisMapping.sensorY.logicalAxis = GyroscopeLogicalAxis::sensorY;
    yawMappingConfig.axisMapping.sensorZ.logicalAxis = GyroscopeLogicalAxis::sensorZ;
    GyroscopeOrientationIntegrator yawMapping(yawMappingConfig);
    yawMapping.update({0.0, 0.0, std::numbers::pi / 2.0}, 0U);
    yawMapping.update({0.0, 0.0, std::numbers::pi / 2.0}, 1'000'000'000U);
    expect(near(quaternionToEulerDiagnostic(yawMapping.orientation()).yawDegrees, 90.0),
           "provisional sensor Z to logical Z mapping produces diagnostic yaw");

    GyroscopeOrientationIntegrator composition(yawMappingConfig);
    composition.update({std::numbers::pi / 2.0, 0.0, 0.0}, 0U);
    composition.update({std::numbers::pi / 2.0, 0.0, 0.0}, 1'000'000'000U);
    composition.update({0.0, 0.0, std::numbers::pi / 2.0}, 2'000'000'000U);
    const Quaternion xQuarter{std::sqrt(0.5), std::sqrt(0.5), 0.0, 0.0};
    const Quaternion zQuarter{std::sqrt(0.5), 0.0, 0.0, std::sqrt(0.5)};
    expect(equivalent(composition.orientation(), xQuarter * zQuarter),
           "body-frame delta is composed as current orientation times delta quaternion");

    OrientationIntegratorConfig recenterConfig;
    recenterConfig.maximumDeviceTimestampDelta = 2s;
    GyroscopeOrientationIntegrator integrator(recenterConfig);
    expect(integrator.recenter() && equivalent(integrator.relativeOrientation(), {}),
           "recenter at identity changes nothing");
    integrator.update({0.0, 0.0, std::numbers::pi / 2.0}, 0U);
    integrator.update({0.0, 0.0, std::numbers::pi / 2.0}, 1'000'000'000U);
    expect(integrator.recenter() && equivalent(integrator.relativeOrientation(), {}),
           "recenter after rotation produces relative identity");
    integrator.update({0.0, 0.0, std::numbers::pi / 2.0}, 2'000'000'000U);
    expect(near(quaternionToEulerDiagnostic(integrator.relativeOrientation()).yawDegrees, 90.0),
           "post-recenter rotation is relative to inverse(reference) times current");
    integrator.clearRecenter();
    expect(near(std::abs(quaternionToEulerDiagnostic(integrator.relativeOrientation()).yawDegrees), 180.0),
           "clearing recenter restores absolute orientation");
    expect(near(integrator.relativeOrientation().norm(), 1.0),
           "recenter output remains normalized");
    expect(!integrator.setOrientation({0.0, 0.0, 0.0, 0.0}),
           "explicit invalid orientation is rejected");
    expect(integrator.setOrientation({2.0, 0.0, 0.0, 0.0})
               && equivalent(integrator.orientation(), {}),
           "explicit orientation is validated and normalized");
    integrator.clearTimestamp();
    expect(!integrator.state().initialized
               && !integrator.state().lastValidDeviceTimestamp.has_value(),
           "timestamp state can be cleared independently");
    integrator.reset();
    expect(equivalent(integrator.orientation(), {})
               && integrator.state().appliedSampleCount == 0U,
           "reset restores identity and counters");

    const auto identityEuler = quaternionToEulerDiagnostic({});
    expect(near(identityEuler.yawDegrees, 0.0) && near(identityEuler.pitchDegrees, 0.0)
               && near(identityEuler.rollDegrees, 0.0),
           "identity produces zero intrinsic ZYX Euler angles");
    expect(near(quaternionToEulerDiagnostic({std::sqrt(0.5), std::sqrt(0.5), 0.0, 0.0}).rollDegrees, 90.0)
               && near(quaternionToEulerDiagnostic({std::sqrt(0.5), 0.0, std::sqrt(0.5), 0.0}).pitchDegrees, 90.0)
               && near(quaternionToEulerDiagnostic({std::sqrt(0.5), 0.0, 0.0, std::sqrt(0.5)}).yawDegrees, 90.0),
           "pure X/Y/Z rotations map to roll/pitch/yaw diagnostics");
    const auto gimbal = quaternionToEulerDiagnostic({0.5, 0.5, 0.5, 0.5});
    expect(std::isfinite(gimbal.yawRadians) && std::isfinite(gimbal.pitchRadians)
               && std::isfinite(gimbal.rollRadians),
           "gimbal-lock diagnostic remains finite with asin clamping");
    const Quaternion unchanged{std::sqrt(0.5), 0.0, std::sqrt(0.5), 0.0};
    static_cast<void>(quaternionToEulerDiagnostic(unchanged));
    expect(equivalent(unchanged, {std::sqrt(0.5), 0.0, std::sqrt(0.5), 0.0}),
           "Euler diagnostics do not alter quaternion state");
}

void testPhysicalUnitsAndJson()
{
    using namespace xreal::sensors;
    const auto profile = makeExperimentalGyroscopeScaleProfile(4090.0);
    const auto physical = convertGyroscopeToPhysicalUnits({4090, 0, 0}, {}, profile);
    expect(near(physical.x.degreesPerSecond, 1.0)
               && near(physical.x.radiansPerSecond, std::numbers::pi / 180.0),
           "4090 raw converts to radians before integration");
    OrientationIntegratorConfig physicalConfig;
    physicalConfig.maximumDeviceTimestampDelta = 2s;
    GyroscopeOrientationIntegrator integrator(physicalConfig);
    integrator.update({physical.x.radiansPerSecond, 0.0, 0.0}, 0U);
    integrator.update({physical.x.radiansPerSecond, 0.0, 0.0}, 1'000'000'000U);
    expect(near(quaternionToEulerDiagnostic(integrator.orientation()).rollDegrees, 1.0),
           "physical-unit conversion feeds quaternion integration");
    auto invalidProfile = profile;
    invalidProfile.x.rawUnitsPerDegreePerSecond = 0.0;
    const auto invalidPhysical = convertGyroscopeToPhysicalUnits({4090, 0, 0}, {}, invalidProfile);
    expect(!invalidPhysical.x.valid && !validateGyroscopeScaleProfile(invalidProfile).valid,
           "invalid axis scale prevents physical input to integration");
    auto perAxisProfile = profile;
    perAxisProfile.y = makeExperimentalGyroscopeScaleProfile(2045.0).y;
    perAxisProfile.axesShareSameScale = false;
    const auto perAxis = convertGyroscopeToPhysicalUnits({4090, 4090, 0}, {}, perAxisProfile);
    expect(near(perAxis.x.degreesPerSecond, 1.0)
               && near(perAxis.y.degreesPerSecond, 2.0),
           "per-axis physical scales may differ");
    expect(profile.experimental && !profile.verified,
           "experimental and unverified scale state is preserved");
    const std::string json = serializeGyroscopeOrientationJson(integrator, profile, true);
    serializedOrientationJson = json;
    expect(json.starts_with("{") && json.ends_with("}\n")
               && json.find("\"component_order\":\"wxyz\"") != std::string::npos
               && json.find("\"axis_mapping\"") != std::string::npos
               && json.find("\"notes\"") != std::string::npos
               && json.find("\"experimental\":true") != std::string::npos
               && json.find("\"final_quaternion\"") != std::string::npos
               && json.find("\"final_euler_degrees\"") != std::string::npos,
           "orientation JSON contains convention, mapping, scale, quaternion and Euler metadata");
    const std::string quaternionOnlyJson = serializeGyroscopeOrientationJson(
        integrator, profile, false);
    expect(quaternionOnlyJson.find("\"final_euler_degrees\"") == std::string::npos,
           "Euler JSON is omitted when quaternion-only output is requested");
}

} // namespace

int main(int argc, char* argv[])
{
    testQuaternionMath();
    testIntegration();
    testRejectionsAndCounters();
    testMappingRecenterAndEuler();
    testPhysicalUnitsAndJson();
    if (argc == 3 && std::string_view(argv[1]) == "--json-output")
    {
        std::ofstream output(argv[2], std::ios::out | std::ios::trunc);
        output << serializedOrientationJson;
        expect(static_cast<bool>(output), "synthetic orientation JSON file is written");
    }
    if (failures != 0)
    {
        std::cerr << failures << " orientation test(s) failed.\n";
        return 1;
    }
    std::cout << "All gyroscope orientation tests passed.\n";
    return 0;
}
