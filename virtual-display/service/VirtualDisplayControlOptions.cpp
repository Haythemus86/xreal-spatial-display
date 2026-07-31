#include "VirtualDisplayControlOptions.hpp"

#include <charconv>
#include <utility>

namespace xreal::virtual_display
{
namespace
{

[[nodiscard]] std::optional<std::uint32_t> parseUnsigned(std::string_view value) noexcept
{
    std::uint32_t parsed{};
    const auto conversion = std::from_chars(value.data(), value.data() + value.size(), parsed);
    if (conversion.ec != std::errc{} || conversion.ptr != value.data() + value.size())
    {
        return std::nullopt;
    }
    return parsed;
}

} // namespace

VirtualDisplayControlParseResult parseVirtualDisplayControlOptions(
    std::span<const std::string_view> arguments)
{
    VirtualDisplayControlParseResult result;
    auto fail = [&](std::string error) {
        result.error = std::move(error);
        return result;
    };
    for (std::size_t index = 0U; index < arguments.size(); ++index)
    {
        const auto argument = arguments[index];
        if (argument == "--help" || argument == "-h")
        {
            result.options.help = true;
            continue;
        }
        if (argument == "--apply-virtual-display-config")
        {
            result.options.apply = true;
            continue;
        }
        if (argument == "--disable-virtual-displays")
        {
            result.options.disable = true;
            continue;
        }
        if (argument == "--show-virtual-display-status")
        {
            result.options.showStatus = true;
            continue;
        }
        if (index + 1U >= arguments.size())
        {
            return fail(std::string(argument) + " requires a value.");
        }
        const auto value = arguments[++index];
        if (argument == "--virtual-displays")
        {
            result.options.countSpecified = true;
            if (value == "0")
            {
                result.options.configuration.enabled = false;
                result.options.configuration.count = VirtualDisplayCount::one;
                continue;
            }
            const auto count = parseVirtualDisplayCount(value);
            if (!count.has_value())
            {
                return fail("--virtual-displays requires 0, 1, 2 or 3.");
            }
            result.options.configuration.enabled = true;
            result.options.configuration.count = *count;
            continue;
        }
        if (argument == "--virtual-display-mode")
        {
            const auto mode = parseVirtualDisplayMode(value);
            if (!mode.has_value())
            {
                return fail("--virtual-display-mode requires extended or mirrored.");
            }
            result.options.configuration.mode = *mode;
            continue;
        }
        const auto parsed = parseUnsigned(value);
        if (!parsed.has_value() || *parsed == 0U)
        {
            return fail(std::string(argument) + " requires a positive integer.");
        }
        if (argument == "--virtual-display-width")
        {
            result.options.configuration.resolution.width = *parsed;
        }
        else if (argument == "--virtual-display-height")
        {
            result.options.configuration.resolution.height = *parsed;
        }
        else if (argument == "--virtual-display-refresh-hz")
        {
            result.options.configuration.resolution.refreshHertz = *parsed;
        }
        else
        {
            return fail("Unknown option: " + std::string(argument));
        }
    }
    if (result.options.apply && result.options.disable)
    {
        return fail("Apply and disable actions are mutually exclusive.");
    }
    if (result.options.apply && !result.options.countSpecified)
    {
        return fail("Applying virtual displays requires an explicit --virtual-displays value.");
    }
    if (!result.options.help && !result.options.apply && !result.options.disable
        && !result.options.showStatus)
    {
        return fail("No action was requested; Windows topology was not modified.");
    }
    if (result.options.apply)
    {
        const auto validation = validateVirtualDisplayConfiguration(
            result.options.configuration);
        if (!validation.valid)
        {
            return fail(validation.error);
        }
    }
    result.success = true;
    return result;
}

std::string virtualDisplayControlUsage()
{
    return
        "xreal-virtual-display-control\n"
        "  --virtual-displays <0|1|2|3>\n"
        "  --virtual-display-mode <extended|mirrored>\n"
        "  --virtual-display-width <pixels>\n"
        "  --virtual-display-height <pixels>\n"
        "  --virtual-display-refresh-hz <hz>\n"
        "  --apply-virtual-display-config\n"
        "  --disable-virtual-displays\n"
        "  --show-virtual-display-status\n";
}

} // namespace xreal::virtual_display
