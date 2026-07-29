#include "rendering/Camera.hpp"
#include "rendering/DemoOrientationSource.hpp"
#include "rendering/OrientationRenderBridge.hpp"
#include "sensors/GyroscopePhysicalUnits.hpp"

#include <atomic>
#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <thread>

namespace
{

int failures{};

void expect(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << "FAILED: " << message << '\n';
        ++failures;
    }
}

[[nodiscard]] xreal::sensors::Quaternion axisAngle(
    xreal::rendering::Vector3 axis,
    double degrees)
{
    const double half = degrees * std::numbers::pi / 360.0;
    return {std::cos(half), axis.x * std::sin(half), axis.y * std::sin(half),
            axis.z * std::sin(half)};
}

void testCameraMatrices()
{
    using namespace xreal::rendering;
    const auto identity = makeHeadViewMatrix(xreal::sensors::Quaternion::identity());
    expect(identity.matrix.has_value()
               && approximatelyEqual(*identity.matrix, Matrix4::identity()),
           "identity quaternion produces identity view rotation");

    for (double angle : {90.0, -90.0, 30.0, -30.0})
    {
        const auto yaw = axisAngle({0.0, 1.0, 0.0}, angle);
        const auto view = makeHeadViewMatrix(yaw);
        expect(view.matrix.has_value()
                   && approximatelyEqual(*view.matrix, rotationMatrix(yaw.conjugate()), 1.0e-9),
               "yaw view uses inverse head orientation");
    }
    const auto pitch = axisAngle({1.0, 0.0, 0.0}, 30.0);
    const auto roll = axisAngle({0.0, 0.0, 1.0}, 30.0);
    expect(approximatelyEqual(*makeHeadViewMatrix(pitch).matrix,
                              rotationMatrix(pitch.conjugate())),
           "pitch view is inverted");
    expect(approximatelyEqual(*makeHeadViewMatrix(roll).matrix,
                              rotationMatrix(roll.conjugate())),
           "roll view is inverted");

    const auto combined = (axisAngle({0.0, 1.0, 0.0}, 32.0)
        * axisAngle({1.0, 0.0, 0.0}, -17.0)
        * axisAngle({0.0, 0.0, 1.0}, 11.0)).normalized().value();
    const auto combinedView = makeHeadViewMatrix(combined);
    expect(combinedView.matrix.has_value()
               && std::abs(rotationDeterminant(*combinedView.matrix) - 1.0) < 1.0e-9,
           "combined view is a proper rotation");
    const auto right = transformDirection(*combinedView.matrix, {1.0, 0.0, 0.0});
    const auto up = transformDirection(*combinedView.matrix, {0.0, 1.0, 0.0});
    const double dot = right.x * up.x + right.y * up.y + right.z * up.z;
    expect(std::abs(dot) < 1.0e-9, "combined view remains orthonormal");

    const xreal::sensors::Quaternion negated{
        -combined.w, -combined.x, -combined.y, -combined.z};
    expect(approximatelyEqual(*combinedView.matrix, *makeHeadViewMatrix(negated).matrix),
           "q and -q produce equivalent view matrices");
    expect(!makeHeadViewMatrix({0.0, 0.0, 0.0, 0.0}).matrix.has_value(),
           "zero quaternion is rejected");
    expect(!makeHeadViewMatrix({std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.0})
                .matrix.has_value(),
           "NaN quaternion is rejected");
    expect(!makeHeadViewMatrix({std::numeric_limits<double>::infinity(), 0.0, 0.0, 0.0})
                .matrix.has_value(),
           "infinite quaternion is rejected");

    const auto rightTurn = axisAngle({0.0, 1.0, 0.0}, -90.0);
    const auto worldForwardInView = transformDirection(
        *makeHeadViewMatrix(rightTurn).matrix, {0.0, 0.0, -1.0});
    expect(worldForwardInView.x < -0.99,
           "right head rotation makes a forward world marker move left");

    const auto ninety = rotationMatrix(axisAngle({0.0, 0.0, 1.0}, 90.0));
    expect(std::abs(ninety.at(0, 1) + 1.0) < 1.0e-9
               && std::abs(ninety.values[1] + 1.0) < 1.0e-9,
           "row-major CPU layout matches row_major HLSL matrix policy without transpose");
}

void testProjection()
{
    using namespace xreal::rendering;
    PerspectiveProjection valid;
    const auto result = makePerspectiveProjection(valid);
    expect(result.matrix.has_value() && result.matrix->finite(),
           "valid perspective parameters produce a finite matrix");
    valid.aspectRatio = 0.0;
    expect(!makePerspectiveProjection(valid).matrix.has_value(), "zero aspect ratio is rejected");
    valid = {};
    valid.verticalFieldOfViewDegrees = -1.0;
    expect(!makePerspectiveProjection(valid).matrix.has_value(), "negative FOV is rejected");
    valid.verticalFieldOfViewDegrees = 180.0;
    expect(!makePerspectiveProjection(valid).matrix.has_value(), "FOV >= 180 is rejected");
    valid = {};
    valid.nearPlane = 0.0;
    expect(!makePerspectiveProjection(valid).matrix.has_value(), "non-positive near plane is rejected");
    valid = {};
    valid.farPlane = valid.nearPlane;
    expect(!makePerspectiveProjection(valid).matrix.has_value(), "far <= near is rejected");
}

void testMapping()
{
    using namespace xreal::rendering;
    const OrientationToRenderMapping mapping;
    expect(validateRenderMapping(mapping), "default render mapping is a proper rotation");
    const auto x = mapSensorVectorToRender({1.0, 0.0, 0.0}, mapping);
    const auto y = mapSensorVectorToRender({0.0, 1.0, 0.0}, mapping);
    const auto z = mapSensorVectorToRender({0.0, 0.0, 1.0}, mapping);
    expect(x.x == 1.0 && x.y == 0.0 && x.z == 0.0, "sensor X maps explicitly to render +X");
    expect(y.x == 0.0 && y.y == 0.0 && y.z == -1.0, "sensor Y maps explicitly to render -Z");
    expect(z.x == 0.0 && z.y == 1.0 && z.z == 0.0, "sensor Z maps explicitly to render +Y");
    const auto mapped = mapSensorOrientationToRender(axisAngle({0.0, 0.0, 1.0}, 30.0), mapping);
    expect(mapped.has_value() && std::abs(mapped->norm() - 1.0) < 1.0e-9,
           "orientation mapping preserves quaternion normalization");
    OrientationToRenderMapping invalid;
    invalid.axes.sensorY = RenderAxis::positiveX;
    expect(!validateRenderMapping(invalid), "duplicate mapped axes are rejected");

    const auto hardwareMapping =
        xreal::sensors::makeExperimentalXrealAir2UltraGyroscopeAxisMapping();
    const auto mapHardwareRotation = [&](Vector3 sensorAxis, double degrees) {
        const auto bodyAxis = xreal::sensors::applyGyroscopeAxisMapping(
            {sensorAxis.x, sensorAxis.y, sensorAxis.z}, hardwareMapping);
        return mapSensorOrientationToRender(
            axisAngle({bodyAxis.x, bodyAxis.y, bodyAxis.z}, degrees), mapping);
    };

    const auto lookUp = mapHardwareRotation({-1.0, 0.0, 0.0}, 30.0);
    const auto lookDown = mapHardwareRotation({1.0, 0.0, 0.0}, 30.0);
    const auto fixedForward{Vector3{0.0, 0.0, -1.0}};
    expect(lookUp.has_value()
               && transformDirection(*makeHeadViewMatrix(*lookUp).matrix, fixedForward).y < 0.0,
           "hardware look-up moves a fixed world marker toward screen bottom");
    expect(lookDown.has_value()
               && transformDirection(*makeHeadViewMatrix(*lookDown).matrix, fixedForward).y > 0.0,
           "hardware look-down moves a fixed world marker toward screen top");

    const auto headRight = mapHardwareRotation({0.0, 0.0, -1.0}, 30.0);
    const auto headLeft = mapHardwareRotation({0.0, 0.0, 1.0}, 30.0);
    expect(headRight.has_value()
               && transformDirection(*makeHeadViewMatrix(*headRight).matrix, fixedForward).x < 0.0,
           "hardware head-right keeps moving a fixed world marker toward screen left");
    expect(headLeft.has_value()
               && transformDirection(*makeHeadViewMatrix(*headLeft).matrix, fixedForward).x > 0.0,
           "hardware head-left keeps moving a fixed world marker toward screen right");

    const auto clockwiseRoll = mapHardwareRotation({0.0, 1.0, 0.0}, 30.0);
    const auto counterClockwiseRoll = mapHardwareRotation({0.0, -1.0, 0.0}, 30.0);
    const Vector3 fixedUp{0.0, 1.0, 0.0};
    expect(clockwiseRoll.has_value()
               && transformDirection(*makeHeadViewMatrix(*clockwiseRoll).matrix, fixedUp).x < 0.0,
           "hardware clockwise roll keeps rotating the fixed world counter-clockwise");
    expect(counterClockwiseRoll.has_value()
               && transformDirection(*makeHeadViewMatrix(*counterClockwiseRoll).matrix, fixedUp).x > 0.0,
           "hardware counter-clockwise roll preserves the opposite world direction");

    const auto combinedYawPitch = mapSensorOrientationToRender(
        (axisAngle({0.0, 0.0, 1.0}, 20.0)
         * axisAngle({1.0, 0.0, 0.0}, 15.0)).normalized().value(), mapping);
    const auto combinedPitchRoll = mapSensorOrientationToRender(
        (axisAngle({1.0, 0.0, 0.0}, 15.0)
         * axisAngle({0.0, 1.0, 0.0}, 10.0)).normalized().value(), mapping);
    expect(combinedYawPitch.has_value() && combinedPitchRoll.has_value()
               && std::abs(combinedYawPitch->norm() - 1.0) < 1.0e-9
               && std::abs(combinedPitchRoll->norm() - 1.0) < 1.0e-9,
           "combined yaw/pitch and pitch/roll remain normalized and coherent");
    expect(std::abs(rotationDeterminant(rotationMatrix(*combinedYawPitch)) - 1.0) < 1.0e-9
               && std::abs(rotationDeterminant(rotationMatrix(*combinedPitchRoll)) - 1.0) < 1.0e-9,
           "combined mapped rotations remain right-handed proper rotations");
    const xreal::sensors::Quaternion negatedMapped{
        -combinedYawPitch->w,
        -combinedYawPitch->x,
        -combinedYawPitch->y,
        -combinedYawPitch->z,
    };
    expect(approximatelyEqual(rotationMatrix(*combinedYawPitch), rotationMatrix(negatedMapped)),
           "mapped q and -q produce equivalent render matrices");
}

xreal::rendering::RenderOrientationSnapshot snapshot(double value)
{
    xreal::rendering::RenderOrientationSnapshot result;
    result.measuredAbsolute = axisAngle({0.0, 0.0, 1.0}, value);
    result.measuredRelative = result.measuredAbsolute;
    result.predictedAbsolute = axisAngle({0.0, 0.0, 1.0}, value + 1.0);
    result.predictedRelative = result.predictedAbsolute;
    result.measuredValid = true;
    result.predictionValid = true;
    return result;
}

void testBridge()
{
    using namespace xreal::rendering;
    OrientationRenderBridge bridge;
    expect(bridge.publish(snapshot(1.0)), "first valid snapshot publishes");
    const auto first = bridge.latest();
    expect(first.has_value() && first->sequence == 1U, "first snapshot has sequence one");
    expect(bridge.publish(snapshot(2.0)), "new latest snapshot replaces previous value");
    const auto second = bridge.latest();
    expect(second.has_value() && second->sequence == 2U, "snapshot sequence increases");

    auto invalid = snapshot(1.0);
    invalid.measuredAbsolute = {0.0, 0.0, 0.0, 0.0};
    invalid.measuredRelative = {0.0, 0.0, 0.0, 0.0};
    invalid.predictedAbsolute = {0.0, 0.0, 0.0, 0.0};
    invalid.predictedRelative = {0.0, 0.0, 0.0, 0.0};
    expect(!bridge.publish(invalid) && bridge.latest()->sequence == 2U,
           "invalid snapshot is not published as valid");

    RenderOrientationSelector selector;
    const OrientationToRenderMapping mapping;
    const auto measured = selector.select(*second, RenderOrientationSource::measured,
        RenderOrientationFrame::absolute, mapping);
    const auto predicted = selector.select(*second, RenderOrientationSource::predicted,
        RenderOrientationFrame::relative, mapping);
    expect(measured.valid && predicted.valid, "measured/predicted and absolute/relative are selectable");
    auto unavailable = *second;
    unavailable.predictionValid = false;
    const auto fallback = selector.select(unavailable, RenderOrientationSource::predicted,
        RenderOrientationFrame::relative, mapping);
    expect(fallback.valid && fallback.preservingLastValid,
           "unavailable prediction preserves the last valid pose");
    expect(second->recenterGeneration == 0U, "recenter generation is preserved");

    std::atomic_bool coherent{true};
    std::thread writer([&] {
        for (int index = 0; index < 10000; ++index)
        {
            auto value = snapshot(static_cast<double>(index % 360));
            value.deviceTimestampNanoseconds = static_cast<std::uint64_t>(index);
            if (!bridge.publish(value)) { coherent = false; }
        }
    });
    std::thread reader([&] {
        std::uint64_t previous{};
        for (int index = 0; index < 10000; ++index)
        {
            const auto current = bridge.latest();
            if (current.has_value())
            {
                if (current->sequence < previous || !current->measuredAbsolute.finite())
                {
                    coherent = false;
                }
                previous = current->sequence;
            }
        }
    });
    writer.join();
    reader.join();
    expect(coherent.load(), "concurrent latest-value publication remains coherent");
}

void testDemoStability()
{
    using namespace xreal::rendering;
    OrientationRenderBridge bridge;
    DemoOrientationSource demo(bridge, 15.0);
    for (int index = 0; index < 100000; ++index)
    {
        demo.update(static_cast<double>(index) * 0.001);
    }
    const auto latest = bridge.latest();
    expect(latest.has_value() && latest->measuredAbsolute.finite()
               && latest->predictedAbsolute.finite(),
           "long deterministic demo motion remains finite without HID");
    const auto projection = makePerspectiveProjection({60.0, 1920.0 / 1080.0, 0.05, 100.0});
    const auto resized = makePerspectiveProjection({60.0, 800.0 / 600.0, 0.05, 100.0});
    expect(projection.matrix.has_value() && resized.matrix.has_value()
               && projection.matrix->at(0, 0) != resized.matrix->at(0, 0),
           "projection updates deterministically after resize");
}

} // namespace

int main()
{
    testCameraMatrices();
    testProjection();
    testMapping();
    testBridge();
    testDemoStability();
    if (failures != 0)
    {
        std::cerr << failures << " rendering math test(s) failed.\n";
        return 1;
    }
    std::cout << "All rendering math tests passed.\n";
    return 0;
}
