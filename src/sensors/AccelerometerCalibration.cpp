#include "sensors/AccelerometerCalibration.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>

namespace xreal::sensors
{
namespace
{

constexpr std::size_t coefficientCount = 6;
constexpr double poorRmsResidualThreshold = 0.02;
constexpr double poorMaximumResidualThreshold = 0.05;
constexpr double poorPivotRatioThreshold = 1.0e-10;

using LinearSystem = std::array<std::array<double, coefficientCount + 1>, coefficientCount>;

struct LinearSolveResult
{
    std::optional<std::array<double, coefficientCount>> solution;
    double pivotRatio = 0.0;
};

[[nodiscard]] LinearSolveResult solveLinearSystem(LinearSystem system)
{
    double minimumPivot = std::numeric_limits<double>::max();
    double maximumPivot = 0.0;

    for (std::size_t column = 0; column < coefficientCount; ++column)
    {
        std::size_t pivotRow = column;
        double pivotMagnitude = std::abs(system[column][column]);

        for (std::size_t row = column + 1; row < coefficientCount; ++row)
        {
            const double candidateMagnitude = std::abs(system[row][column]);
            if (candidateMagnitude > pivotMagnitude)
            {
                pivotMagnitude = candidateMagnitude;
                pivotRow = row;
            }
        }

        if (pivotMagnitude <= std::numeric_limits<double>::epsilon())
        {
            return {};
        }

        std::swap(system[column], system[pivotRow]);
        minimumPivot = std::min(minimumPivot, pivotMagnitude);
        maximumPivot = std::max(maximumPivot, pivotMagnitude);

        for (std::size_t row = column + 1; row < coefficientCount; ++row)
        {
            const double factor = system[row][column] / system[column][column];
            system[row][column] = 0.0;
            for (std::size_t entry = column + 1; entry <= coefficientCount; ++entry)
            {
                system[row][entry] -= factor * system[column][entry];
            }
        }
    }

    std::array<double, coefficientCount> solution{};
    for (std::size_t reverseIndex = 0; reverseIndex < coefficientCount; ++reverseIndex)
    {
        const std::size_t row = coefficientCount - 1 - reverseIndex;
        double value = system[row][coefficientCount];
        for (std::size_t column = row + 1; column < coefficientCount; ++column)
        {
            value -= system[row][column] * solution[column];
        }
        solution[row] = value / system[row][row];
    }

    LinearSolveResult result;
    result.solution = solution;
    result.pivotRatio = maximumPivot > 0.0 ? minimumPivot / maximumPivot : 0.0;
    return result;
}

[[nodiscard]] std::size_t skipWhitespace(std::string_view json, std::size_t position)
{
    while (position < json.size())
    {
        const char character = json[position];
        if (character != ' ' && character != '\t' && character != '\r' && character != '\n')
        {
            break;
        }
        ++position;
    }
    return position;
}

[[nodiscard]] std::optional<std::size_t> findValuePosition(
    std::string_view json,
    std::string_view key,
    std::size_t start = 0)
{
    const std::string quotedKey = "\"" + std::string(key) + "\"";
    const std::size_t keyPosition = json.find(quotedKey, start);
    if (keyPosition == std::string_view::npos)
    {
        return std::nullopt;
    }

    const std::size_t colonPosition = json.find(':', keyPosition + quotedKey.size());
    if (colonPosition == std::string_view::npos)
    {
        return std::nullopt;
    }

    return skipWhitespace(json, colonPosition + 1);
}

[[nodiscard]] std::optional<std::string_view> findObject(
    std::string_view json,
    std::string_view key)
{
    const auto valuePosition = findValuePosition(json, key);
    if (!valuePosition.has_value() || *valuePosition >= json.size() || json[*valuePosition] != '{')
    {
        return std::nullopt;
    }

    std::size_t depth = 0;
    bool inString = false;
    bool escaped = false;
    for (std::size_t position = *valuePosition; position < json.size(); ++position)
    {
        const char character = json[position];
        if (inString)
        {
            if (escaped)
            {
                escaped = false;
            }
            else if (character == '\\')
            {
                escaped = true;
            }
            else if (character == '"')
            {
                inString = false;
            }
            continue;
        }

        if (character == '"')
        {
            inString = true;
        }
        else if (character == '{')
        {
            ++depth;
        }
        else if (character == '}')
        {
            --depth;
            if (depth == 0)
            {
                return json.substr(*valuePosition, position - *valuePosition + 1);
            }
        }
    }

    return std::nullopt;
}

[[nodiscard]] std::optional<std::string> parseString(
    std::string_view json,
    std::string_view key)
{
    const auto valuePosition = findValuePosition(json, key);
    if (!valuePosition.has_value() || *valuePosition >= json.size() || json[*valuePosition] != '"')
    {
        return std::nullopt;
    }

    std::string value;
    bool escaped = false;
    for (std::size_t position = *valuePosition + 1; position < json.size(); ++position)
    {
        const char character = json[position];
        if (escaped)
        {
            switch (character)
            {
            case '"':
            case '\\':
            case '/':
                value.push_back(character);
                break;
            case 'n':
                value.push_back('\n');
                break;
            case 'r':
                value.push_back('\r');
                break;
            case 't':
                value.push_back('\t');
                break;
            default:
                return std::nullopt;
            }
            escaped = false;
        }
        else if (character == '\\')
        {
            escaped = true;
        }
        else if (character == '"')
        {
            return value;
        }
        else
        {
            value.push_back(character);
        }
    }

    return std::nullopt;
}

[[nodiscard]] std::optional<bool> parseBoolean(
    std::string_view json,
    std::string_view key)
{
    const auto valuePosition = findValuePosition(json, key);
    if (!valuePosition.has_value())
    {
        return std::nullopt;
    }
    if (json.substr(*valuePosition, 4) == "true")
    {
        return true;
    }
    if (json.substr(*valuePosition, 5) == "false")
    {
        return false;
    }
    return std::nullopt;
}

[[nodiscard]] std::optional<double> parseNumber(
    std::string_view json,
    std::string_view key)
{
    const auto valuePosition = findValuePosition(json, key);
    if (!valuePosition.has_value())
    {
        return std::nullopt;
    }

    const char* begin = json.data() + *valuePosition;
    const char* end = json.data() + json.size();
    double value = 0.0;
    const auto conversion = std::from_chars(begin, end, value);
    if (conversion.ec != std::errc())
    {
        return std::nullopt;
    }
    return value;
}

[[nodiscard]] std::string escapeJson(std::string_view value)
{
    std::ostringstream output;
    for (const char character : value)
    {
        switch (character)
        {
        case '"':
            output << "\\\"";
            break;
        case '\\':
            output << "\\\\";
            break;
        case '\n':
            output << "\\n";
            break;
        case '\r':
            output << "\\r";
            break;
        case '\t':
            output << "\\t";
            break;
        default:
            output << character;
            break;
        }
    }
    return output.str();
}

void writeVector(std::ostringstream& output, const RawVector3d& value)
{
    output << "{\"x\":" << value.x << ",\"y\":" << value.y << ",\"z\":" << value.z << '}';
}

} // namespace

AccelerometerCalibrationFitResult fitAxisAlignedAccelerometerEllipsoid(
    std::span<const StaticAccelerometerCapture> captures)
{
    AccelerometerCalibrationFitResult result;
    std::vector<const StaticAccelerometerCapture*> acceptedCaptures;
    acceptedCaptures.reserve(captures.size());
    for (const auto& capture : captures)
    {
        if (capture.stationaryAccepted)
        {
            acceptedCaptures.push_back(&capture);
        }
    }

    if (acceptedCaptures.size() < coefficientCount)
    {
        result.errorMessage = "At least six accepted stationary captures are required for the ellipsoid fit.";
        return result;
    }

    RawVector3d coordinateCenter;
    for (const auto* capture : acceptedCaptures)
    {
        coordinateCenter.x += capture->accelMeanRaw.x;
        coordinateCenter.y += capture->accelMeanRaw.y;
        coordinateCenter.z += capture->accelMeanRaw.z;
    }
    const double captureCount = static_cast<double>(acceptedCaptures.size());
    coordinateCenter.x /= captureCount;
    coordinateCenter.y /= captureCount;
    coordinateCenter.z /= captureCount;

    double coordinateScale = 0.0;
    for (const auto* capture : acceptedCaptures)
    {
        coordinateScale = std::max(coordinateScale, std::abs(capture->accelMeanRaw.x - coordinateCenter.x));
        coordinateScale = std::max(coordinateScale, std::abs(capture->accelMeanRaw.y - coordinateCenter.y));
        coordinateScale = std::max(coordinateScale, std::abs(capture->accelMeanRaw.z - coordinateCenter.z));
    }
    if (!std::isfinite(coordinateScale) || coordinateScale <= std::numeric_limits<double>::epsilon())
    {
        result.errorMessage = "Accepted captures do not span a usable accelerometer range.";
        return result;
    }

    LinearSystem system{};
    for (const auto* capture : acceptedCaptures)
    {
        const double x = (capture->accelMeanRaw.x - coordinateCenter.x) / coordinateScale;
        const double y = (capture->accelMeanRaw.y - coordinateCenter.y) / coordinateScale;
        const double z = (capture->accelMeanRaw.z - coordinateCenter.z) / coordinateScale;
        const std::array<double, coefficientCount> row{x * x, y * y, z * z, x, y, z};

        for (std::size_t rowIndex = 0; rowIndex < coefficientCount; ++rowIndex)
        {
            for (std::size_t columnIndex = 0; columnIndex < coefficientCount; ++columnIndex)
            {
                system[rowIndex][columnIndex] += row[rowIndex] * row[columnIndex];
            }
            system[rowIndex][coefficientCount] += row[rowIndex];
        }
    }

    const LinearSolveResult solved = solveLinearSystem(system);
    if (!solved.solution.has_value())
    {
        result.errorMessage = "The accepted capture geometry is singular; collect more diverse static orientations.";
        return result;
    }

    const auto& coefficients = *solved.solution;
    const double a = coefficients[0];
    const double b = coefficients[1];
    const double c = coefficients[2];
    if (a <= 0.0 || b <= 0.0 || c <= 0.0)
    {
        result.errorMessage = "The fitted quadratic is not a valid axis-aligned ellipsoid.";
        return result;
    }

    const RawVector3d normalizedOffset{
        -coefficients[3] / (2.0 * a),
        -coefficients[4] / (2.0 * b),
        -coefficients[5] / (2.0 * c)};
    const double radiusTerm = 1.0
        + a * normalizedOffset.x * normalizedOffset.x
        + b * normalizedOffset.y * normalizedOffset.y
        + c * normalizedOffset.z * normalizedOffset.z;
    if (!std::isfinite(radiusTerm) || radiusTerm <= 0.0)
    {
        result.errorMessage = "The fitted ellipsoid has invalid radii.";
        return result;
    }

    AxisAlignedAccelerometerCalibration calibration;
    calibration.offsetRaw = {
        coordinateCenter.x + coordinateScale * normalizedOffset.x,
        coordinateCenter.y + coordinateScale * normalizedOffset.y,
        coordinateCenter.z + coordinateScale * normalizedOffset.z};
    calibration.rawUnitsPerG = {
        coordinateScale * std::sqrt(radiusTerm / a),
        coordinateScale * std::sqrt(radiusTerm / b),
        coordinateScale * std::sqrt(radiusTerm / c)};
    calibration.normalEquationPivotRatio = solved.pivotRatio;

    double sumSquaredResiduals = 0.0;
    for (const auto* capture : acceptedCaptures)
    {
        const double correctedX = (capture->accelMeanRaw.x - calibration.offsetRaw.x) / calibration.rawUnitsPerG.x;
        const double correctedY = (capture->accelMeanRaw.y - calibration.offsetRaw.y) / calibration.rawUnitsPerG.y;
        const double correctedZ = (capture->accelMeanRaw.z - calibration.offsetRaw.z) / calibration.rawUnitsPerG.z;
        const double magnitude = std::sqrt(correctedX * correctedX + correctedY * correctedY + correctedZ * correctedZ);
        const double residual = magnitude - 1.0;
        calibration.captures.push_back({
            capture->name,
            capture->source,
            capture->accelMeanRaw,
            residual,
            magnitude});
        sumSquaredResiduals += residual * residual;
        calibration.maximumResidual = std::max(calibration.maximumResidual, std::abs(residual));
    }
    calibration.rmsResidual = std::sqrt(sumSquaredResiduals / captureCount);

    if (acceptedCaptures.size() == coefficientCount)
    {
        calibration.qualityWarnings.emplace_back(
            "The six captures exactly determine the six model coefficients; residuals are not an independent validation set.");
    }
    if (calibration.normalEquationPivotRatio < poorPivotRatioThreshold)
    {
        calibration.poorQuality = true;
        calibration.qualityWarnings.emplace_back(
            "The capture orientations provide weak numerical coverage; collect additional diverse static orientations.");
    }
    if (calibration.rmsResidual > poorRmsResidualThreshold)
    {
        calibration.poorQuality = true;
        calibration.qualityWarnings.emplace_back("The RMS radial residual exceeds 0.02 g.");
    }
    if (calibration.maximumResidual > poorMaximumResidualThreshold)
    {
        calibration.poorQuality = true;
        calibration.qualityWarnings.emplace_back("The maximum radial residual exceeds 0.05 g.");
    }

    result.calibration = std::move(calibration);
    return result;
}

CalibrationCaptureParseResult parseCalibrationCaptureJson(std::string_view json, std::string source)
{
    CalibrationCaptureParseResult result;
    const auto schemaVersion = parseNumber(json, "schema_version");
    if (!schemaVersion.has_value() || *schemaVersion != 1.0)
    {
        result.errorMessage = "Unsupported or missing calibration schema_version.";
        return result;
    }

    const auto units = parseString(json, "units");
    if (!units.has_value() || *units != "raw")
    {
        result.errorMessage = "Calibration input must use raw units.";
        return result;
    }

    const auto calibrationName = parseString(json, "calibration");
    const auto stationaryObject = findObject(json, "stationary");
    if (!calibrationName.has_value() || !stationaryObject.has_value())
    {
        result.errorMessage = "Missing calibration name or stationary result.";
        return result;
    }

    const auto accepted = parseBoolean(*stationaryObject, "accepted");
    const auto accelMeanObject = findObject(*stationaryObject, "accel_mean_raw");
    if (!accepted.has_value() || !accelMeanObject.has_value())
    {
        result.errorMessage = "Missing stationary acceptance state or accelerometer mean.";
        return result;
    }

    const auto x = parseNumber(*accelMeanObject, "x");
    const auto y = parseNumber(*accelMeanObject, "y");
    const auto z = parseNumber(*accelMeanObject, "z");
    if (!x.has_value() || !y.has_value() || !z.has_value())
    {
        result.errorMessage = "Invalid stationary accelerometer mean.";
        return result;
    }

    result.capture = StaticAccelerometerCapture{
        *calibrationName,
        std::move(source),
        *accepted,
        {*x, *y, *z}};
    return result;
}

std::string serializeAccelerometerCalibrationProfileJson(
    const AxisAlignedAccelerometerCalibration& calibration)
{
    std::ostringstream output;
    output << std::setprecision(17);
    output << "{\n"
           << "  \"schema_version\": 1,\n"
           << "  \"model\": \"axis_aligned_ellipsoid\",\n"
           << "  \"units\": {\"offset\":\"raw\",\"scale\":\"raw_units_per_g\",\"corrected_magnitude\":\"g\"},\n"
           << "  \"offset_raw\": ";
    writeVector(output, calibration.offsetRaw);
    output << ",\n  \"raw_units_per_g\": ";
    writeVector(output, calibration.rawUnitsPerG);
    output << ",\n  \"fit\": {\n"
           << "    \"rms_residual\": " << calibration.rmsResidual << ",\n"
           << "    \"maximum_residual\": " << calibration.maximumResidual << ",\n"
           << "    \"normal_equation_pivot_ratio\": " << calibration.normalEquationPivotRatio << ",\n"
           << "    \"poor_quality\": " << (calibration.poorQuality ? "true" : "false") << ",\n"
           << "    \"warnings\": [";
    for (std::size_t index = 0; index < calibration.qualityWarnings.size(); ++index)
    {
        if (index != 0)
        {
            output << ',';
        }
        output << "\"" << escapeJson(calibration.qualityWarnings[index]) << "\"";
    }
    output << "]\n  },\n  \"captures\": [\n";

    for (std::size_t index = 0; index < calibration.captures.size(); ++index)
    {
        const auto& capture = calibration.captures[index];
        output << "    {\"name\":\"" << escapeJson(capture.name)
               << "\",\"source\":\"" << escapeJson(capture.source)
               << "\",\"stationary_accepted\":true,\"accel_mean_raw\":";
        writeVector(output, capture.accelMeanRaw);
        output << ",\"residual\":" << capture.residual
               << ",\"corrected_gravity_magnitude\":" << capture.correctedGravityMagnitude << '}';
        if (index + 1 != calibration.captures.size())
        {
            output << ',';
        }
        output << '\n';
    }
    output << "  ]\n}\n";
    return output.str();
}

} // namespace xreal::sensors
