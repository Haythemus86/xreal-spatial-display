#include "sensors/GyroscopePhysicalUnits.hpp"

#include <charconv>
#include <cmath>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>

namespace
{

void usage()
{
    std::cout << "Usage: xreal-gyro-scale-profile --raw-per-dps <positive-value>"
                 " --experimental --unverified --output <file.json>\n";
}

[[nodiscard]] std::optional<double> positiveDouble(std::string_view text)
{
    double value{};
    const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
    if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size()
        || !std::isfinite(value) || value <= 0.0)
    {
        return std::nullopt;
    }
    return value;
}

} // namespace

int main(int argc, char* argv[])
{
    std::optional<double> rawPerDps;
    std::string outputPath;
    bool experimental{};
    bool unverified{};
    for (int index = 1; index < argc; ++index)
    {
        const std::string_view argument = argv[index];
        if (argument == "--help" || argument == "-h")
        {
            usage();
            return 0;
        }
        if (argument == "--experimental")
        {
            experimental = true;
            continue;
        }
        if (argument == "--unverified")
        {
            unverified = true;
            continue;
        }
        if ((argument == "--raw-per-dps" || argument == "--output") && index + 1 < argc)
        {
            const std::string_view value = argv[++index];
            if (argument == "--raw-per-dps")
            {
                rawPerDps = positiveDouble(value);
            }
            else
            {
                outputPath = value;
            }
            continue;
        }
        std::cerr << "Invalid or incomplete option: " << argument << '\n';
        usage();
        return 2;
    }
    if (!rawPerDps.has_value() || outputPath.empty() || !experimental || !unverified)
    {
        std::cerr << "A positive scale, --experimental, --unverified and --output are required.\n";
        return 2;
    }
    const auto profile = xreal::sensors::makeExperimentalGyroscopeScaleProfile(*rawPerDps);
    const auto validation = xreal::sensors::validateGyroscopeScaleProfile(profile);
    if (!validation.valid)
    {
        std::cerr << "Invalid profile: " << validation.explanation << '\n';
        return 1;
    }
    std::ofstream output(outputPath, std::ios::out | std::ios::trunc);
    if (!output)
    {
        std::cerr << "Failed to open profile output: " << outputPath << '\n';
        return 1;
    }
    output << xreal::sensors::serializeGyroscopeScaleProfileJson(profile);
    if (!output)
    {
        std::cerr << "Failed while writing profile output: " << outputPath << '\n';
        return 1;
    }
    std::cout << "Experimental, unverified gyroscope scale profile written to "
              << outputPath << '\n';
    return 0;
}
