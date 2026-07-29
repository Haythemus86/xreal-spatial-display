#include "rendering/RenderMath.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace xreal::rendering
{

bool Vector3::finite() const noexcept
{
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

Matrix4 Matrix4::identity() noexcept
{
    Matrix4 result;
    result.at(0, 0) = 1.0;
    result.at(1, 1) = 1.0;
    result.at(2, 2) = 1.0;
    result.at(3, 3) = 1.0;
    return result;
}

double& Matrix4::at(std::size_t row, std::size_t column) noexcept
{
    return values[row * 4U + column];
}

double Matrix4::at(std::size_t row, std::size_t column) const noexcept
{
    return values[row * 4U + column];
}

bool Matrix4::finite() const noexcept
{
    return std::ranges::all_of(values, [](double value) { return std::isfinite(value); });
}

Matrix4 operator*(const Matrix4& left, const Matrix4& right) noexcept
{
    Matrix4 result;
    for (std::size_t row = 0; row < 4U; ++row)
    {
        for (std::size_t column = 0; column < 4U; ++column)
        {
            for (std::size_t inner = 0; inner < 4U; ++inner)
            {
                result.at(row, column) += left.at(row, inner) * right.at(inner, column);
            }
        }
    }
    return result;
}

Vector3 transformDirection(const Matrix4& matrix, Vector3 vector) noexcept
{
    return {
        matrix.at(0, 0) * vector.x + matrix.at(0, 1) * vector.y
            + matrix.at(0, 2) * vector.z,
        matrix.at(1, 0) * vector.x + matrix.at(1, 1) * vector.y
            + matrix.at(1, 2) * vector.z,
        matrix.at(2, 0) * vector.x + matrix.at(2, 1) * vector.y
            + matrix.at(2, 2) * vector.z,
    };
}

Matrix4 rotationMatrix(const sensors::Quaternion& orientation) noexcept
{
    const auto normalized = orientation.normalized();
    if (!normalized.has_value())
    {
        return {};
    }
    const auto& q = *normalized;
    Matrix4 result = Matrix4::identity();
    result.at(0, 0) = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    result.at(0, 1) = 2.0 * (q.x * q.y - q.z * q.w);
    result.at(0, 2) = 2.0 * (q.x * q.z + q.y * q.w);
    result.at(1, 0) = 2.0 * (q.x * q.y + q.z * q.w);
    result.at(1, 1) = 1.0 - 2.0 * (q.x * q.x + q.z * q.z);
    result.at(1, 2) = 2.0 * (q.y * q.z - q.x * q.w);
    result.at(2, 0) = 2.0 * (q.x * q.z - q.y * q.w);
    result.at(2, 1) = 2.0 * (q.y * q.z + q.x * q.w);
    result.at(2, 2) = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    return result;
}

std::optional<sensors::Quaternion> quaternionFromRotationMatrix(const Matrix4& matrix) noexcept
{
    if (!matrix.finite())
    {
        return std::nullopt;
    }
    sensors::Quaternion result;
    const double trace = matrix.at(0, 0) + matrix.at(1, 1) + matrix.at(2, 2);
    if (trace > 0.0)
    {
        const double scale = std::sqrt(trace + 1.0) * 2.0;
        result.w = 0.25 * scale;
        result.x = (matrix.at(2, 1) - matrix.at(1, 2)) / scale;
        result.y = (matrix.at(0, 2) - matrix.at(2, 0)) / scale;
        result.z = (matrix.at(1, 0) - matrix.at(0, 1)) / scale;
    }
    else if (matrix.at(0, 0) > matrix.at(1, 1)
             && matrix.at(0, 0) > matrix.at(2, 2))
    {
        const double scale = std::sqrt(
            1.0 + matrix.at(0, 0) - matrix.at(1, 1) - matrix.at(2, 2)) * 2.0;
        result.w = (matrix.at(2, 1) - matrix.at(1, 2)) / scale;
        result.x = 0.25 * scale;
        result.y = (matrix.at(0, 1) + matrix.at(1, 0)) / scale;
        result.z = (matrix.at(0, 2) + matrix.at(2, 0)) / scale;
    }
    else if (matrix.at(1, 1) > matrix.at(2, 2))
    {
        const double scale = std::sqrt(
            1.0 + matrix.at(1, 1) - matrix.at(0, 0) - matrix.at(2, 2)) * 2.0;
        result.w = (matrix.at(0, 2) - matrix.at(2, 0)) / scale;
        result.x = (matrix.at(0, 1) + matrix.at(1, 0)) / scale;
        result.y = 0.25 * scale;
        result.z = (matrix.at(1, 2) + matrix.at(2, 1)) / scale;
    }
    else
    {
        const double scale = std::sqrt(
            1.0 + matrix.at(2, 2) - matrix.at(0, 0) - matrix.at(1, 1)) * 2.0;
        result.w = (matrix.at(1, 0) - matrix.at(0, 1)) / scale;
        result.x = (matrix.at(0, 2) + matrix.at(2, 0)) / scale;
        result.y = (matrix.at(1, 2) + matrix.at(2, 1)) / scale;
        result.z = 0.25 * scale;
    }
    return result.normalized();
}

double rotationDeterminant(const Matrix4& matrix) noexcept
{
    return matrix.at(0, 0) * (matrix.at(1, 1) * matrix.at(2, 2)
                              - matrix.at(1, 2) * matrix.at(2, 1))
        - matrix.at(0, 1) * (matrix.at(1, 0) * matrix.at(2, 2)
                             - matrix.at(1, 2) * matrix.at(2, 0))
        + matrix.at(0, 2) * (matrix.at(1, 0) * matrix.at(2, 1)
                             - matrix.at(1, 1) * matrix.at(2, 0));
}

bool approximatelyEqual(const Matrix4& left, const Matrix4& right, double tolerance) noexcept
{
    if (!std::isfinite(tolerance) || tolerance < 0.0)
    {
        return false;
    }
    for (std::size_t index = 0; index < left.values.size(); ++index)
    {
        if (std::abs(left.values[index] - right.values[index]) > tolerance)
        {
            return false;
        }
    }
    return true;
}

} // namespace xreal::rendering
