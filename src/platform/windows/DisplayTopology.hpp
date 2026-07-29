#pragma once

#include <cstdint>
#include <iomanip>
#include <optional>
#include <span>
#include <sstream>
#include <string>
#include <vector>

namespace xreal::platform::windows
{

struct DxgiAdapterLuid
{
    std::uint32_t lowPart{};
    std::int32_t highPart{};

    [[nodiscard]] bool operator==(const DxgiAdapterLuid&) const noexcept = default;
};

struct DxgiOutputInformation
{
    unsigned int adapterIndex{};
    unsigned int outputIndex{};
    std::string deviceName;
    int left{};
    int top{};
    int right{};
    int bottom{};
    bool attachedToDesktop{};
    std::string adapterDescription;
    DxgiAdapterLuid adapterLuid;
};

struct MonitorInformation
{
    unsigned int index{};
    std::string deviceName;
    int left{};
    int top{};
    int right{};
    int bottom{};
    int workLeft{};
    int workTop{};
    int workRight{};
    int workBottom{};
    bool primary{};
    std::optional<DxgiOutputInformation> dxgiOutput;
};

struct DisplayTopologyResult
{
    std::vector<MonitorInformation> monitors;
    std::string error;
};

struct WindowPlacement
{
    int x{};
    int y{};
    unsigned int width{};
    unsigned int height{};
};

void associateDxgiOutputs(
    std::span<MonitorInformation> monitors,
    std::span<const DxgiOutputInformation> outputs);

[[nodiscard]] std::optional<WindowPlacement> calculateWindowPlacement(
    const MonitorInformation& monitor,
    unsigned int requestedWidth,
    unsigned int requestedHeight,
    std::optional<int> monitorRelativeX,
    std::optional<int> monitorRelativeY,
    bool fullscreen) noexcept;

[[nodiscard]] DisplayTopologyResult enumerateDisplayTopology();
[[nodiscard]] inline std::string dxgiAdapterLuidText(DxgiAdapterLuid luid)
{
    std::ostringstream output;
    output << "0x" << std::hex << std::uppercase << std::setfill('0')
           << std::setw(8) << static_cast<std::uint32_t>(luid.highPart)
           << ':' << std::setw(8) << luid.lowPart;
    return output.str();
}

} // namespace xreal::platform::windows
