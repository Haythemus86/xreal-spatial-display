#pragma once

#include "rendering/RenderMath.hpp"

#include <optional>
#include <string>

namespace xreal::rendering
{

struct PerspectiveProjection
{
    double verticalFieldOfViewDegrees{60.0};
    double aspectRatio{16.0 / 9.0};
    double nearPlane{0.05};
    double farPlane{100.0};
};

struct MatrixResult
{
    std::optional<Matrix4> matrix;
    std::string error;
};

[[nodiscard]] MatrixResult makePerspectiveProjection(
    const PerspectiveProjection& projection) noexcept;
[[nodiscard]] MatrixResult makeHeadViewMatrix(
    const sensors::Quaternion& worldFromHead) noexcept;
[[nodiscard]] MatrixResult makeViewProjection(
    const sensors::Quaternion& worldFromHead,
    const PerspectiveProjection& projection) noexcept;

} // namespace xreal::rendering
