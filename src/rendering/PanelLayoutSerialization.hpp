#pragma once

#include "rendering/PanelScene.hpp"

#include <optional>
#include <string>
#include <string_view>

namespace xreal::rendering
{

inline constexpr unsigned int panelLayoutSchemaVersion = 1U;

struct PanelLayoutLoadResult
{
    std::optional<PanelScene> scene;
    std::string error;
};

[[nodiscard]] std::string serializePanelLayoutJson(const PanelScene& scene);
[[nodiscard]] PanelLayoutLoadResult loadPanelLayoutJson(std::string_view json);
[[nodiscard]] bool savePanelLayoutFileAtomic(
    const std::string& path,
    const PanelScene& scene,
    bool overwrite,
    std::string& error);

} // namespace xreal::rendering
