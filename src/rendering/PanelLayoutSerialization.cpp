#include "rendering/PanelLayoutSerialization.hpp"

#include <charconv>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace xreal::rendering
{
namespace
{

[[nodiscard]] std::string escapeJson(std::string_view value)
{
    std::string result;
    result.reserve(value.size());
    for (const char character : value)
    {
        if (character == '"' || character == '\\')
        {
            result.push_back('\\');
        }
        result.push_back(character);
    }
    return result;
}

[[nodiscard]] std::optional<std::size_t> valuePosition(
    std::string_view json,
    std::string_view key,
    std::size_t start = 0U) noexcept
{
    const std::string quoted = "\"" + std::string(key) + "\"";
    bool insideString{};
    bool escaped{};
    for (std::size_t keyPosition = start; keyPosition < json.size(); ++keyPosition)
    {
        const char character = json[keyPosition];
        if (insideString)
        {
            if (escaped) { escaped = false; }
            else if (character == '\\') { escaped = true; }
            else if (character == '"') { insideString = false; }
            continue;
        }
        if (json.substr(keyPosition, quoted.size()) == quoted)
        {
            std::size_t position = keyPosition + quoted.size();
            while (position < json.size()
                   && (json[position] == ' ' || json[position] == '\t'
                       || json[position] == '\r' || json[position] == '\n'))
            {
                ++position;
            }
            if (position >= json.size() || json[position] != ':')
            {
                continue;
            }
            ++position;
            while (position < json.size()
                   && (json[position] == ' ' || json[position] == '\t'
                       || json[position] == '\r' || json[position] == '\n'))
            {
                ++position;
            }
            return position;
        }
        if (character == '"')
        {
            insideString = true;
        }
    }
    return std::nullopt;
}

template<typename Number>
[[nodiscard]] std::optional<Number> numberValue(
    std::string_view json,
    std::string_view key) noexcept
{
    const auto position = valuePosition(json, key);
    if (!position.has_value())
    {
        return std::nullopt;
    }
    Number value{};
    const char* begin = json.data() + *position;
    const char* end = json.data() + json.size();
    const auto parsed = std::from_chars(begin, end, value);
    if (parsed.ec != std::errc{} || parsed.ptr == begin)
    {
        return std::nullopt;
    }
    return value;
}

[[nodiscard]] std::optional<bool> boolValue(
    std::string_view json,
    std::string_view key) noexcept
{
    const auto position = valuePosition(json, key);
    if (!position.has_value())
    {
        return std::nullopt;
    }
    if (json.substr(*position, 4U) == "true") { return true; }
    if (json.substr(*position, 5U) == "false") { return false; }
    return std::nullopt;
}

[[nodiscard]] std::optional<std::string> stringValue(
    std::string_view json,
    std::string_view key)
{
    const auto position = valuePosition(json, key);
    if (!position.has_value() || *position >= json.size() || json[*position] != '"')
    {
        return std::nullopt;
    }
    std::string result;
    bool escaped{};
    for (std::size_t index = *position + 1U; index < json.size(); ++index)
    {
        const char character = json[index];
        if (escaped)
        {
            if (character != '"' && character != '\\')
            {
                return std::nullopt;
            }
            result.push_back(character);
            escaped = false;
        }
        else if (character == '\\')
        {
            escaped = true;
        }
        else if (character == '"')
        {
            return result;
        }
        else
        {
            result.push_back(character);
        }
    }
    return std::nullopt;
}

[[nodiscard]] std::optional<std::string_view> panelObject(
    std::string_view json,
    std::size_t panelIndex) noexcept
{
    const auto panelsPosition = valuePosition(json, "panels");
    if (!panelsPosition.has_value() || *panelsPosition >= json.size()
        || json[*panelsPosition] != '[')
    {
        return std::nullopt;
    }
    std::size_t objectIndex{};
    bool insideString{};
    bool escaped{};
    for (std::size_t position = *panelsPosition + 1U; position < json.size(); ++position)
    {
        const char character = json[position];
        if (insideString)
        {
            if (escaped) { escaped = false; }
            else if (character == '\\') { escaped = true; }
            else if (character == '"') { insideString = false; }
            continue;
        }
        if (character == '"')
        {
            insideString = true;
            continue;
        }
        if (character == ']')
        {
            return std::nullopt;
        }
        if (character != '{')
        {
            continue;
        }
        const std::size_t begin = position;
        unsigned int depth{1U};
        for (++position; position < json.size(); ++position)
        {
            const char nested = json[position];
            if (insideString)
            {
                if (escaped) { escaped = false; }
                else if (nested == '\\') { escaped = true; }
                else if (nested == '"') { insideString = false; }
                continue;
            }
            if (nested == '"') { insideString = true; }
            else if (nested == '{') { ++depth; }
            else if (nested == '}' && --depth == 0U)
            {
                if (objectIndex == panelIndex)
                {
                    return json.substr(begin, position - begin + 1U);
                }
                ++objectIndex;
                break;
            }
        }
    }
    return std::nullopt;
}

} // namespace

std::string serializePanelLayoutJson(const PanelScene& scene)
{
    if (!validatePanelScene(scene).valid)
    {
        return {};
    }
    std::ostringstream output;
    output << std::setprecision(17)
           << "{\n  \"schema_version\": " << panelLayoutSchemaVersion << ",\n"
           << "  \"units\": \"meters\",\n"
           << "  \"layout\": \"" << panelLayoutPresetText(scene.layout) << "\",\n"
           << "  \"performance_profile\": \""
           << performanceProfileText(scene.performanceProfile) << "\",\n"
           << "  \"panel_count\": " << scene.panelCount << ",\n"
           << "  \"selected_panel\": " << scene.selectedPanel << ",\n"
           << "  \"default_width\": " << scene.defaultWidth << ",\n"
           << "  \"default_height\": " << scene.defaultHeight << ",\n"
           << "  \"default_distance\": " << scene.defaultDistance << ",\n"
           << "  \"gap\": " << scene.gap << ",\n"
           << "  \"curvature_degrees\": " << scene.curvatureDegrees << ",\n"
           << "  \"panels\": [\n";
    for (std::size_t index = 0; index < scene.panelCount; ++index)
    {
        const auto& panel = scene.panels[index];
        output << "    {\"id\":" << static_cast<unsigned int>(panel.id.value)
               << ",\"name\":\"" << escapeJson(panel.displayName) << '"'
               << ",\"enabled\":" << (panel.enabled ? "true" : "false")
               << ",\"x\":" << panel.transform.position.x
               << ",\"y\":" << panel.transform.position.y
               << ",\"z\":" << panel.transform.position.z
               << ",\"yaw_degrees\":" << panel.transform.yawDegrees
               << ",\"pitch_degrees\":" << panel.transform.pitchDegrees
               << ",\"width\":" << panel.dimensions.width
               << ",\"height\":" << panel.dimensions.height
               << ",\"target_fps\":" << panel.targetFramesPerSecond
               << ",\"target_fps_explicit\":"
               << (panel.targetFramesPerSecondExplicit ? "true" : "false")
               << ",\"fit\":\"" << panelFitModeText(panel.fit) << '"'
               << ",\"filter\":\"" << panelFilterModeText(panel.filter) << '"'
               << ",\"overlay_enabled\":"
               << (panel.overlay.enabled ? "true" : "false")
               << ",\"content\":\"" << panelContentKindText(panel.content.kind) << '"';
        if (panel.content.captureMonitorIndex.has_value())
        {
            output << ",\"capture_monitor_index\":"
                   << *panel.content.captureMonitorIndex;
        }
        if (panel.content.captureMonitorDeviceName.has_value())
        {
            output << ",\"capture_monitor_device_name\":\""
                   << escapeJson(*panel.content.captureMonitorDeviceName) << '"';
        }
        output << ",\"source_width\":" << panel.content.requestedWidth
               << ",\"source_height\":" << panel.content.requestedHeight
               << ",\"source_scale\":" << panel.content.requestedScale
               << ",\"transfer_policy\":\""
               << panelTransferPolicyText(panel.content.transferPolicy) << "\"}";
        output << (index + 1U == scene.panelCount ? "\n" : ",\n");
    }
    output << "  ]\n}\n";
    return output.str();
}

PanelLayoutLoadResult loadPanelLayoutJson(std::string_view json)
{
    const auto schema = numberValue<unsigned int>(json, "schema_version");
    const auto count = numberValue<std::size_t>(json, "panel_count");
    const auto selected = numberValue<std::size_t>(json, "selected_panel");
    const auto layoutText = stringValue(json, "layout");
    const auto profileText = stringValue(json, "performance_profile");
    const auto units = stringValue(json, "units");
    if (!schema.has_value() || *schema != panelLayoutSchemaVersion)
    {
        return {std::nullopt, "Unsupported or missing panel layout schema version."};
    }
    if (!count.has_value() || *count == 0U || *count > maximumPanelCount
        || !selected.has_value() || *selected >= *count)
    {
        return {std::nullopt, "Panel count or selected panel is invalid."};
    }
    if (!units.has_value() || *units != "meters")
    {
        return {std::nullopt, "Panel layout unit convention must be meters."};
    }
    const auto layout = layoutText.has_value() ? parsePanelLayoutPreset(*layoutText) : std::nullopt;
    const auto profile = profileText.has_value() ? parsePerformanceProfile(*profileText) : std::nullopt;
    if (!layout.has_value() || !profile.has_value())
    {
        return {std::nullopt, "Panel layout preset or performance profile is invalid."};
    }
    PanelScene scene = makeDefaultPanelScene(*count);
    scene.panelCount = *count;
    scene.selectedPanel = *selected;
    scene.layout = *layout;
    scene.performanceProfile = *profile;
    const auto defaultWidth = numberValue<double>(json, "default_width");
    const auto defaultHeight = numberValue<double>(json, "default_height");
    const auto defaultDistance = numberValue<double>(json, "default_distance");
    const auto gap = numberValue<double>(json, "gap");
    const auto curvature = numberValue<double>(json, "curvature_degrees");
    if (!defaultWidth.has_value() || !defaultHeight.has_value()
        || !defaultDistance.has_value() || !gap.has_value() || !curvature.has_value())
    {
        return {std::nullopt, "Panel layout defaults are incomplete."};
    }
    scene.defaultWidth = *defaultWidth;
    scene.defaultHeight = *defaultHeight;
    scene.defaultDistance = *defaultDistance;
    scene.gap = *gap;
    scene.curvatureDegrees = *curvature;
    for (std::size_t index = 0; index < scene.panelCount; ++index)
    {
        const auto object = panelObject(json, index);
        if (!object.has_value())
        {
            return {std::nullopt, "Panel array contains fewer objects than panel_count."};
        }
        const auto id = numberValue<unsigned int>(*object, "id");
        const auto name = stringValue(*object, "name");
        const auto enabled = boolValue(*object, "enabled");
        const auto x = numberValue<double>(*object, "x");
        const auto y = numberValue<double>(*object, "y");
        const auto z = numberValue<double>(*object, "z");
        const auto yaw = numberValue<double>(*object, "yaw_degrees");
        const auto pitch = numberValue<double>(*object, "pitch_degrees");
        const auto width = numberValue<double>(*object, "width");
        const auto height = numberValue<double>(*object, "height");
        const auto rate = numberValue<double>(*object, "target_fps");
        const auto explicitRate = boolValue(*object, "target_fps_explicit");
        const auto fitText = stringValue(*object, "fit");
        const auto filterText = stringValue(*object, "filter");
        const auto overlayEnabled = boolValue(*object, "overlay_enabled");
        const auto fit = fitText.has_value() ? parsePanelFitMode(*fitText) : std::nullopt;
        const auto filter = filterText.has_value()
            ? parsePanelFilterMode(*filterText) : std::nullopt;
        const auto contentText = stringValue(*object, "content");
        const auto content = contentText.has_value()
            ? parsePanelContentKind(*contentText) : std::nullopt;
        if (!id.has_value() || *id != index || !name.has_value() || name->empty()
            || !enabled.has_value()
            || !x.has_value() || !y.has_value() || !z.has_value()
            || !yaw.has_value() || !pitch.has_value() || !width.has_value()
            || !height.has_value() || !rate.has_value() || !fit.has_value()
            || !filter.has_value() || !overlayEnabled.has_value()
            || !content.has_value())
        {
            return {std::nullopt, "Panel object is incomplete or has an unstable identifier."};
        }
        auto& panel = scene.panels[index];
        panel = {};
        panel.id = PanelId{static_cast<std::uint8_t>(*id)};
        panel.displayName = *name;
        panel.enabled = *enabled;
        panel.transform = {{*x, *y, *z}, *yaw, *pitch};
        panel.dimensions = {*width, *height};
        panel.targetFramesPerSecond = *rate;
        panel.targetFramesPerSecondExplicit = explicitRate.value_or(false);
        panel.fit = *fit;
        panel.filter = *filter;
        panel.overlay.enabled = *overlayEnabled;
        panel.content.kind = *content;
        panel.content.captureMonitorIndex = numberValue<unsigned int>(
            *object, "capture_monitor_index");
        panel.content.captureMonitorDeviceName = stringValue(
            *object, "capture_monitor_device_name");
        panel.content.requestedWidth = numberValue<unsigned int>(
            *object, "source_width").value_or(0U);
        panel.content.requestedHeight = numberValue<unsigned int>(
            *object, "source_height").value_or(0U);
        panel.content.requestedScale = numberValue<double>(
            *object, "source_scale").value_or(1.0);
        const auto transferPolicyText = stringValue(*object, "transfer_policy");
        const auto transferPolicy = transferPolicyText.has_value()
            ? parsePanelTransferPolicy(*transferPolicyText) : std::nullopt;
        if (!transferPolicy.has_value())
        {
            return {std::nullopt, "Panel transfer policy is missing or invalid."};
        }
        panel.content.transferPolicy = *transferPolicy;
    }
    const auto validation = validatePanelScene(scene);
    if (!validation.valid)
    {
        return {std::nullopt, validation.error};
    }
    refreshPanelWorldTransforms(scene);
    return {scene, {}};
}

bool savePanelLayoutFileAtomic(
    const std::string& path,
    const PanelScene& scene,
    bool overwrite,
    std::string& error)
{
    const auto validation = validatePanelScene(scene);
    if (!validation.valid)
    {
        error = "Refusing to save an invalid panel layout: " + validation.error;
        return false;
    }
    const std::filesystem::path destination(path);
    if (destination.empty())
    {
        error = "Panel layout output path is empty.";
        return false;
    }
    std::error_code filesystemError;
    if (!overwrite && std::filesystem::exists(destination, filesystemError))
    {
        error = "Panel layout output already exists; use --overwrite-panel-layout to replace it.";
        return false;
    }
    const std::filesystem::path temporary = destination.string() + ".tmp";
    {
        std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
        if (!output)
        {
            error = "Failed to open temporary panel layout output.";
            return false;
        }
        output << serializePanelLayoutJson(scene);
        if (!output)
        {
            error = "Failed to write temporary panel layout output.";
            return false;
        }
    }
    if (overwrite && std::filesystem::exists(destination, filesystemError))
    {
        std::filesystem::remove(destination, filesystemError);
        if (filesystemError)
        {
            error = "Failed to replace the existing panel layout: " + filesystemError.message();
            return false;
        }
    }
    std::filesystem::rename(temporary, destination, filesystemError);
    if (filesystemError)
    {
        error = "Failed to atomically rename the panel layout: " + filesystemError.message();
        return false;
    }
    return true;
}

} // namespace xreal::rendering
