#include "rendering/Camera.hpp"

#include <cmath>
#include <numbers>

namespace xreal::rendering
{

MatrixResult makePerspectiveProjection(const PerspectiveProjection& projection) noexcept
{
    if (!std::isfinite(projection.verticalFieldOfViewDegrees)
        || projection.verticalFieldOfViewDegrees <= 0.0
        || projection.verticalFieldOfViewDegrees >= 180.0)
    {
        return {std::nullopt, "Field of view must be finite and between 0 and 180 degrees."};
    }
    if (!std::isfinite(projection.aspectRatio) || projection.aspectRatio <= 0.0)
    {
        return {std::nullopt, "Projection aspect ratio must be finite and positive."};
    }
    if (!std::isfinite(projection.nearPlane) || projection.nearPlane <= 0.0)
    {
        return {std::nullopt, "Near plane must be finite and positive."};
    }
    if (!std::isfinite(projection.farPlane)
        || projection.farPlane <= projection.nearPlane)
    {
        return {std::nullopt, "Far plane must be finite and greater than the near plane."};
    }

    const double radians = projection.verticalFieldOfViewDegrees * std::numbers::pi / 180.0;
    const double verticalScale = 1.0 / std::tan(radians * 0.5);
    Matrix4 matrix;
    matrix.at(0, 0) = verticalScale / projection.aspectRatio;
    matrix.at(1, 1) = verticalScale;
    matrix.at(2, 2) = projection.farPlane / (projection.nearPlane - projection.farPlane);
    matrix.at(2, 3) = projection.nearPlane * projection.farPlane
        / (projection.nearPlane - projection.farPlane);
    matrix.at(3, 2) = -1.0;
    return matrix.finite() ? MatrixResult{matrix, {}}
                           : MatrixResult{std::nullopt, "Projection matrix is not finite."};
}

MatrixResult makeHeadViewMatrix(const sensors::Quaternion& worldFromHead) noexcept
{
    const auto headFromWorld = worldFromHead.inverseNormalized();
    if (!headFromWorld.has_value())
    {
        return {std::nullopt, "Head orientation quaternion is invalid."};
    }
    const Matrix4 view = rotationMatrix(*headFromWorld);
    if (!view.finite() || std::abs(rotationDeterminant(view) - 1.0) > 1.0e-6)
    {
        return {std::nullopt, "Head view rotation is not a finite proper rotation."};
    }
    return {view, {}};
}

MatrixResult makeViewProjection(
    const sensors::Quaternion& worldFromHead,
    const PerspectiveProjection& projection) noexcept
{
    const auto view = makeHeadViewMatrix(worldFromHead);
    if (!view.matrix.has_value())
    {
        return view;
    }
    const auto projected = makePerspectiveProjection(projection);
    if (!projected.matrix.has_value())
    {
        return projected;
    }
    Matrix4 combined = *projected.matrix * *view.matrix;
    return combined.finite() ? MatrixResult{combined, {}}
                             : MatrixResult{std::nullopt, "View-projection matrix is not finite."};
}

} // namespace xreal::rendering
