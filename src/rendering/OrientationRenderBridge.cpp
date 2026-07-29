#include "rendering/OrientationRenderBridge.hpp"

#include <array>
#include <cmath>
#include <sstream>

namespace xreal::rendering
{
namespace
{

[[nodiscard]] Vector3 axisVector(RenderAxis axis) noexcept
{
    switch (axis)
    {
    case RenderAxis::positiveX: return {1.0, 0.0, 0.0};
    case RenderAxis::negativeX: return {-1.0, 0.0, 0.0};
    case RenderAxis::positiveY: return {0.0, 1.0, 0.0};
    case RenderAxis::negativeY: return {0.0, -1.0, 0.0};
    case RenderAxis::positiveZ: return {0.0, 0.0, 1.0};
    case RenderAxis::negativeZ: return {0.0, 0.0, -1.0};
    }
    return {};
}

[[nodiscard]] Matrix4 mappingMatrix(const OrientationToRenderMapping& mapping) noexcept
{
    Matrix4 result = Matrix4::identity();
    const std::array columns{
        axisVector(mapping.axes.sensorX),
        axisVector(mapping.axes.sensorY),
        axisVector(mapping.axes.sensorZ),
    };
    for (std::size_t column = 0; column < columns.size(); ++column)
    {
        result.at(0, column) = columns[column].x;
        result.at(1, column) = columns[column].y;
        result.at(2, column) = columns[column].z;
    }
    return result;
}

[[nodiscard]] Matrix4 transposeRotation(const Matrix4& matrix) noexcept
{
    Matrix4 result = Matrix4::identity();
    for (std::size_t row = 0; row < 3U; ++row)
    {
        for (std::size_t column = 0; column < 3U; ++column)
        {
            result.at(row, column) = matrix.at(column, row);
        }
    }
    return result;
}

[[nodiscard]] bool validQuaternion(const sensors::Quaternion& value) noexcept
{
    return value.normalized().has_value();
}

} // namespace

Vector3 mapSensorVectorToRender(
    Vector3 sensorVector,
    const OrientationToRenderMapping& mapping) noexcept
{
    return transformDirection(mappingMatrix(mapping), sensorVector);
}

std::optional<sensors::Quaternion> mapSensorOrientationToRender(
    const sensors::Quaternion& sensorOrientation,
    const OrientationToRenderMapping& mapping) noexcept
{
    if (!validateRenderMapping(mapping))
    {
        return std::nullopt;
    }
    const auto normalized = sensorOrientation.normalized();
    if (!normalized.has_value())
    {
        return std::nullopt;
    }
    const Matrix4 basis = mappingMatrix(mapping);
    const Matrix4 mapped = basis * rotationMatrix(*normalized) * transposeRotation(basis);
    return quaternionFromRotationMatrix(mapped);
}

bool validateRenderMapping(const OrientationToRenderMapping& mapping) noexcept
{
    const Matrix4 basis = mappingMatrix(mapping);
    return basis.finite() && std::abs(rotationDeterminant(basis) - 1.0) <= 1.0e-9;
}

std::string renderAxisText(RenderAxis axis)
{
    switch (axis)
    {
    case RenderAxis::positiveX: return "+X";
    case RenderAxis::negativeX: return "-X";
    case RenderAxis::positiveY: return "+Y";
    case RenderAxis::negativeY: return "-Y";
    case RenderAxis::positiveZ: return "+Z";
    case RenderAxis::negativeZ: return "-Z";
    }
    return "invalid";
}

std::string renderMappingText(const OrientationToRenderMapping& mapping)
{
    std::ostringstream output;
    output << "sensor X -> " << renderAxisText(mapping.axes.sensorX)
           << ", sensor Y -> " << renderAxisText(mapping.axes.sensorY)
           << ", sensor Z -> " << renderAxisText(mapping.axes.sensorZ)
           << ", right-handed, source=" << mapping.source
           << ", experimental=" << (mapping.experimental ? "yes" : "no")
           << ", verified=" << (mapping.verified ? "yes" : "no");
    return output.str();
}

bool OrientationRenderBridge::publish(RenderOrientationSnapshot snapshot) noexcept
{
    if (snapshot.measuredValid)
    {
        const auto absolute = snapshot.measuredAbsolute.normalized();
        const auto relative = snapshot.measuredRelative.normalized();
        if (!absolute.has_value() || !relative.has_value())
        {
            snapshot.measuredValid = false;
        }
        else
        {
            snapshot.measuredAbsolute = *absolute;
            snapshot.measuredRelative = *relative;
        }
    }
    if (snapshot.predictionValid)
    {
        const auto absolute = snapshot.predictedAbsolute.normalized();
        const auto relative = snapshot.predictedRelative.normalized();
        if (!absolute.has_value() || !relative.has_value())
        {
            snapshot.predictionValid = false;
        }
        else
        {
            snapshot.predictedAbsolute = *absolute;
            snapshot.predictedRelative = *relative;
        }
    }
    if (!snapshot.measuredValid && !snapshot.predictionValid)
    {
        return false;
    }

    const std::scoped_lock lock(mutex_);
    snapshot.sequence = nextSequence_++;
    latest_ = snapshot;
    return true;
}

std::optional<RenderOrientationSnapshot> OrientationRenderBridge::latest() const noexcept
{
    const std::scoped_lock lock(mutex_);
    return latest_;
}

void OrientationRenderBridge::requestRecenter() noexcept
{
    recenterRequested_.store(true, std::memory_order_release);
}

void OrientationRenderBridge::requestClearRecenter() noexcept
{
    clearRecenterRequested_.store(true, std::memory_order_release);
}

bool OrientationRenderBridge::consumeRecenterRequest() noexcept
{
    return recenterRequested_.exchange(false, std::memory_order_acq_rel);
}

bool OrientationRenderBridge::consumeClearRecenterRequest() noexcept
{
    return clearRecenterRequested_.exchange(false, std::memory_order_acq_rel);
}

SelectedRenderOrientation RenderOrientationSelector::select(
    const RenderOrientationSnapshot& snapshot,
    RenderOrientationSource source,
    RenderOrientationFrame frame,
    const OrientationToRenderMapping& mapping) noexcept
{
    const bool available = source == RenderOrientationSource::predicted
        ? snapshot.predictionValid : snapshot.measuredValid;
    if (available)
    {
        const sensors::Quaternion sensorOrientation = source == RenderOrientationSource::predicted
            ? (frame == RenderOrientationFrame::absolute
                   ? snapshot.predictedAbsolute : snapshot.predictedRelative)
            : (frame == RenderOrientationFrame::absolute
                   ? snapshot.measuredAbsolute : snapshot.measuredRelative);
        const auto mapped = mapSensorOrientationToRender(sensorOrientation, mapping);
        if (mapped.has_value())
        {
            lastValid_ = *mapped;
            return {*mapped, true, false, snapshot.sequence};
        }
    }
    if (lastValid_.has_value())
    {
        return {*lastValid_, true, true, snapshot.sequence};
    }
    return {sensors::Quaternion::identity(), false, false, snapshot.sequence};
}

void RenderOrientationSelector::reset() noexcept
{
    lastValid_.reset();
}

} // namespace xreal::rendering
