#pragma once

#include "sensors/GyroscopeOrientation.hpp"

#include <array>
#include <cstddef>
#include <optional>

namespace xreal::rendering
{

struct Vector3
{
    double x{};
    double y{};
    double z{};

    [[nodiscard]] bool finite() const noexcept;
};

// Row-major storage, column-vector multiplication. HLSL consumes the same
// layout through an explicit row_major declaration and mul(matrix, vector).
struct Matrix4
{
    std::array<double, 16> values{};

    [[nodiscard]] static Matrix4 identity() noexcept;
    [[nodiscard]] double& at(std::size_t row, std::size_t column) noexcept;
    [[nodiscard]] double at(std::size_t row, std::size_t column) const noexcept;
    [[nodiscard]] bool finite() const noexcept;
};

[[nodiscard]] Matrix4 operator*(const Matrix4& left, const Matrix4& right) noexcept;
[[nodiscard]] Vector3 transformDirection(const Matrix4& matrix, Vector3 vector) noexcept;
[[nodiscard]] Matrix4 rotationMatrix(const sensors::Quaternion& orientation) noexcept;
[[nodiscard]] std::optional<sensors::Quaternion> quaternionFromRotationMatrix(
    const Matrix4& matrix) noexcept;
[[nodiscard]] double rotationDeterminant(const Matrix4& matrix) noexcept;
[[nodiscard]] bool approximatelyEqual(
    const Matrix4& left,
    const Matrix4& right,
    double tolerance = 1.0e-9) noexcept;

} // namespace xreal::rendering
