#include "sensors/GyroscopePhysicalUnits.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <string_view>
#include <vector>

namespace
{

int failures{};

void expect(bool condition, std::string_view description)
{
    if (!condition)
    {
        ++failures;
        std::cerr << "FAILED: " << description << '\n';
    }
}

[[nodiscard]] bool near(double actual, double expected, double tolerance = 1.0e-10)
{
    return std::abs(actual - expected) <= tolerance;
}

void testProfileValidationAndConversion()
{
    auto profile = xreal::sensors::makeExperimentalGyroscopeScaleProfile(4090.0);
    expect(xreal::sensors::validateGyroscopeScaleProfile(profile).valid,
           "valid 4090 profile is accepted");
    expect(xreal::sensors::gyroscopeScaleProvenanceWarning(profile).has_value(),
           "experimental profile produces an explicit diagnostic warning");
    expect(near(profile.x.rawUnitsPerRadianPerSecond, 4090.0 * 180.0 / std::numbers::pi),
           "radian scale is derived with pi");

    const xreal::sensors::RawImuVector3 raw{4190, -3990, 41000};
    const xreal::sensors::GyroscopeBias bias{100.0, 100.0, 100.0};
    const auto converted = xreal::sensors::convertGyroscopeToPhysicalUnits(raw, bias, profile);
    expect(converted.raw.x == 4190 && near(converted.biasCorrectedRaw.x, 4090.0),
           "raw and bias-corrected values are preserved");
    expect(near(converted.x.degreesPerSecond, 1.0)
               && near(converted.y.degreesPerSecond, -1.0)
               && near(converted.z.degreesPerSecond, 10.0),
           "positive, negative and ten degree-per-second values convert exactly");
    expect(near(converted.x.radiansPerSecond, std::numbers::pi / 180.0)
               && near(converted.x.radiansPerSecond,
                       converted.biasCorrectedRaw.x / profile.x.rawUnitsPerRadianPerSecond),
           "degree and direct-radian formulas agree");

    profile.y = xreal::sensors::makeExperimentalGyroscopeScaleProfile(2045.0).y;
    const auto perAxis = xreal::sensors::convertGyroscopeToPhysicalUnits(raw, bias, profile);
    expect(near(perAxis.y.degreesPerSecond, -2.0), "per-axis scales may differ");

    for (const double invalid : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
                                 std::numeric_limits<double>::infinity()})
    {
        auto bad = xreal::sensors::makeExperimentalGyroscopeScaleProfile(4090.0);
        bad.x.rawUnitsPerDegreePerSecond = invalid;
        expect(!xreal::sensors::validateGyroscopeScaleProfile(bad).valid,
               "zero, negative, NaN and infinite scales are rejected");
        expect(!xreal::sensors::convertGyroscopeToPhysicalUnits(raw, bias, bad).x.valid,
               "invalid profile prevents conversion");
    }
    profile.x.valid = false;
    expect(!xreal::sensors::validateGyroscopeScaleProfile(profile).valid,
           "enabled axis marked invalid is rejected");
}

void testJsonAndProvenance()
{
    auto profile = xreal::sensors::makeExperimentalGyroscopeScaleProfile(4090.0);
    profile.axisMapping.sensorX.logicalAxis = xreal::sensors::GyroscopeLogicalAxis::sensorZ;
    profile.axisMapping.sensorY.logicalAxis = xreal::sensors::GyroscopeLogicalAxis::sensorX;
    profile.axisMapping.sensorZ.logicalAxis = xreal::sensors::GyroscopeLogicalAxis::sensorY;
    profile.axisMapping.sensorZ.sign = -1;
    profile.axisMapping.source = "synthetic-permutation";
    const std::string json = xreal::sensors::serializeGyroscopeScaleProfileJson(profile);
    expect(json.find("4090") != std::string::npos
               && json.find("234339") != std::string::npos,
           "JSON preserves 4090 and computed radians scale");
    const auto loaded = xreal::sensors::loadGyroscopeScaleProfileJson(json);
    expect(loaded.profile.has_value()
               && loaded.profile->source == "experimental-manual-calibration"
               && loaded.profile->experimental && !loaded.profile->verified
               && loaded.profile->axisMapping.sensorX.logicalAxis
                    == xreal::sensors::GyroscopeLogicalAxis::sensorZ
               && loaded.profile->axisMapping.sensorZ.sign == -1
               && loaded.profile->axisMapping.source == "synthetic-permutation",
           "JSON loading preserves scale provenance and complete axis mapping");
    expect(!xreal::sensors::loadGyroscopeScaleProfileJson("not-json").profile.has_value(),
           "invalid JSON is rejected");
    expect(!xreal::sensors::loadGyroscopeScaleProfileJson(
               "{\"schema_version\":2,\"source\":\"x\",\"experimental\":true,"
               "\"verified\":false,\"raw_units_per_degree_per_second\":{"
               "\"x\":1,\"y\":1,\"z\":1}}").profile.has_value(),
           "unsupported schema is rejected");
}

void testComparisonsAndMapping()
{
    const auto report = xreal::sensors::compareGyroscopeScales(4090.0, 4096.0);
    expect(near(report.absoluteScaleDifference, 6.0)
               && near(report.relativeScaleDifferencePercent, 6.0 / 4096.0 * 100.0),
           "Q12 absolute and relative differences are correct");
    expect(near(report.representativeSamples[0].selectedDegreesPerSecond, 1.0)
               && report.representativeSamples[0].comparisonDegreesPerSecond < 1.0
               && !report.documentaryProof,
           "representative comparison is diagnostic and not proof");

    const auto selected = xreal::sensors::compareGyroscopeIntegratedAngle(
        4090.0 * 360.0, 360.0, 4090.0, 4096.0);
    const auto negative = xreal::sensors::compareGyroscopeIntegratedAngle(
        -4090.0 * 360.0, -360.0, 4090.0, 4096.0);
    expect(near(selected.selectedDegrees, 360.0) && selected.selectedFitsBetter
               && near(negative.selectedDegrees, -360.0),
           "fixed comparison reconstructs signed angles and selects closer scale");
    const std::vector comparisons{selected, negative};
    const auto statistics = xreal::sensors::calculateGyroscopeAngleErrorStatistics(comparisons);
    expect(near(statistics.meanAbsoluteErrorDegrees, 0.0)
               && near(statistics.rootMeanSquareErrorDegrees, 0.0)
               && statistics.selectedFitsBetter,
           "batch mean absolute and RMS errors are correct");

    auto profile = xreal::sensors::makeExperimentalGyroscopeScaleProfile(4090.0);
    expect(profile.axisMapping.sensorX.logicalAxis
               == xreal::sensors::GyroscopeLogicalAxis::sensorX
               && profile.axisMapping.sensorZ.sign == 1
               && profile.axisMapping.experimental && !profile.axisMapping.verified,
           "axis mapping preserves identity and remains provisional");
    profile.axisMapping.sensorZ.sign = -1;
    expect(xreal::sensors::validateGyroscopeScaleProfile(profile).valid,
           "axis mapping supports sign inversion");
    const auto mapped = xreal::sensors::applyGyroscopeAxisMapping(
        {1.0, 2.0, 3.0}, profile.axisMapping);
    expect(near(mapped.x, 1.0) && near(mapped.y, 2.0) && near(mapped.z, -3.0),
           "axis mapping applies sign inversion without renaming sensor axes");

    const auto air2UltraMapping =
        xreal::sensors::makeExperimentalXrealAir2UltraGyroscopeAxisMapping();
    const auto air2UltraMapped = xreal::sensors::applyGyroscopeAxisMapping(
        {-1.0, 2.0, 3.0}, air2UltraMapping);
    expect(near(air2UltraMapped.x, 1.0) && near(air2UltraMapped.y, 2.0)
               && near(air2UltraMapped.z, 3.0),
           "Air 2 Ultra mapping corrects pitch while preserving roll and yaw signs");
    expect(xreal::sensors::validateGyroscopeAxisMapping(air2UltraMapping)
               && air2UltraMapping.experimental && air2UltraMapping.verified,
           "Air 2 Ultra gyroscope mapping records successful hardware direction validation");
}

} // namespace

int main()
{
    testProfileValidationAndConversion();
    testJsonAndProvenance();
    testComparisonsAndMapping();
    if (failures != 0)
    {
        return 1;
    }
    std::cout << "All gyroscope physical-unit tests passed.\n";
    return 0;
}
