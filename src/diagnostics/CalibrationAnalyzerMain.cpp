#include "sensors/AccelerometerCalibration.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace
{

struct CommandLineOptions
{
    std::vector<std::filesystem::path> inputFiles;
    std::filesystem::path outputFile;
    bool showHelp = false;
};

struct CommandLineResult
{
    std::optional<CommandLineOptions> options;
    std::string errorMessage;
};

void printUsage(std::ostream& output)
{
    output << "Usage: xreal-calibration-analyzer <capture.json>... --output <profile.json>\n"
           << "\n"
           << "Fits an axis-aligned accelerometer ellipsoid from accepted stationary\n"
           << "capture means. At least six accepted captures are required.\n";
}

[[nodiscard]] CommandLineResult parseCommandLine(int argumentCount, char** argumentValues)
{
    CommandLineOptions options;
    for (int index = 1; index < argumentCount; ++index)
    {
        const std::string_view argument = argumentValues[index];
        if (argument == "--help" || argument == "-h")
        {
            options.showHelp = true;
        }
        else if (argument == "--output")
        {
            if (index + 1 >= argumentCount)
            {
                return {std::nullopt, "--output requires a file path."};
            }
            options.outputFile = argumentValues[++index];
        }
        else if (argument.starts_with("--"))
        {
            return {std::nullopt, "Unknown option: " + std::string(argument)};
        }
        else
        {
            options.inputFiles.emplace_back(argumentValues[index]);
        }
    }

    if (!options.showHelp)
    {
        if (options.inputFiles.empty())
        {
            return {std::nullopt, "No calibration capture files were provided."};
        }
        if (options.outputFile.empty())
        {
            return {std::nullopt, "--output is required."};
        }
    }
    return {std::move(options), {}};
}

[[nodiscard]] std::optional<std::string> readTextFile(
    const std::filesystem::path& path,
    std::string& errorMessage)
{
    std::ifstream input(path, std::ios::binary);
    if (!input)
    {
        errorMessage = "Could not open input file: " + path.string();
        return std::nullopt;
    }

    std::ostringstream contents;
    contents << input.rdbuf();
    if (input.bad())
    {
        errorMessage = "Could not read input file: " + path.string();
        return std::nullopt;
    }
    return contents.str();
}

[[nodiscard]] bool writeTextFile(
    const std::filesystem::path& path,
    std::string_view contents,
    std::string& errorMessage)
{
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    if (!output)
    {
        errorMessage = "Could not open output file: " + path.string();
        return false;
    }
    output.write(contents.data(), static_cast<std::streamsize>(contents.size()));
    output.flush();
    if (!output)
    {
        errorMessage = "Could not write output file: " + path.string();
        return false;
    }
    return true;
}

} // namespace

int main(int argumentCount, char** argumentValues)
{
    const CommandLineResult commandLine = parseCommandLine(argumentCount, argumentValues);
    if (!commandLine.options.has_value())
    {
        std::cerr << "Error: " << commandLine.errorMessage << "\n\n";
        printUsage(std::cerr);
        return 2;
    }
    if (commandLine.options->showHelp)
    {
        printUsage(std::cout);
        return 0;
    }

    std::vector<xreal::sensors::StaticAccelerometerCapture> captures;
    captures.reserve(commandLine.options->inputFiles.size());
    std::set<std::string> acceptedNames;

    for (const auto& inputPath : commandLine.options->inputFiles)
    {
        std::string errorMessage;
        const auto contents = readTextFile(inputPath, errorMessage);
        if (!contents.has_value())
        {
            std::cerr << "Error: " << errorMessage << '\n';
            return 1;
        }

        auto parsed = xreal::sensors::parseCalibrationCaptureJson(*contents, inputPath.string());
        if (!parsed.capture.has_value())
        {
            std::cerr << "Error: " << inputPath.string() << ": " << parsed.errorMessage << '\n';
            return 1;
        }

        if (!parsed.capture->stationaryAccepted)
        {
            std::cout << "Skipping rejected stationary capture: " << parsed.capture->name
                      << " (" << inputPath.string() << ")\n";
        }
        else if (!acceptedNames.insert(parsed.capture->name).second)
        {
            std::cerr << "Error: duplicate accepted calibration name: " << parsed.capture->name << '\n';
            return 1;
        }
        captures.push_back(std::move(*parsed.capture));
    }

    const auto fit = xreal::sensors::fitAxisAlignedAccelerometerEllipsoid(captures);
    if (!fit.calibration.has_value())
    {
        std::cerr << "Error: " << fit.errorMessage << '\n';
        return 1;
    }

    const auto& calibration = *fit.calibration;
    std::cout << std::fixed << std::setprecision(6)
              << "Accepted captures: " << calibration.captures.size() << '\n'
              << "Offset raw:        X=" << calibration.offsetRaw.x
              << " Y=" << calibration.offsetRaw.y
              << " Z=" << calibration.offsetRaw.z << '\n'
              << "Raw units per g:   X=" << calibration.rawUnitsPerG.x
              << " Y=" << calibration.rawUnitsPerG.y
              << " Z=" << calibration.rawUnitsPerG.z << '\n'
              << "RMS residual:      " << calibration.rmsResidual << " g\n"
              << "Maximum residual:  " << calibration.maximumResidual << " g\n"
              << "\nPer-capture results:\n";

    for (const auto& capture : calibration.captures)
    {
        std::cout << "  " << capture.name
                  << ": magnitude=" << capture.correctedGravityMagnitude << " g"
                  << ", residual=" << capture.residual << " g\n";
    }

    if (!calibration.qualityWarnings.empty())
    {
        std::cout << "\nQuality " << (calibration.poorQuality ? "warning" : "notes") << ":\n";
        for (const auto& warning : calibration.qualityWarnings)
        {
            std::cout << "  - " << warning << '\n';
        }
    }

    std::string errorMessage;
    const std::string json = xreal::sensors::serializeAccelerometerCalibrationProfileJson(calibration);
    if (!writeTextFile(commandLine.options->outputFile, json, errorMessage))
    {
        std::cerr << "Error: " << errorMessage << '\n';
        return 1;
    }

    std::cout << "\nCalibration profile written to: " << commandLine.options->outputFile.string() << '\n';
    return 0;
}
