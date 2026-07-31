#pragma once

#include "rendering/PanelScene.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace xreal::rendering
{

struct PanelSourceKey
{
    PanelContentKind kind{PanelContentKind::synthetic};
    std::optional<unsigned int> monitorIndex;
    std::string monitorDeviceName;
    unsigned int requestedWidth{};
    unsigned int requestedHeight{};
    double requestedScale{1.0};
    capture::DesktopResolutionPolicy resolutionPolicy{
        capture::DesktopResolutionPolicy::native};
    capture::DesktopCropMode cropMode{capture::DesktopCropMode::full};
    capture::DesktopCaptureRegion customRegion;
    capture::DesktopFit scaleFit{capture::DesktopFit::contain};
    capture::DesktopFilter scaleFilter{capture::DesktopFilter::linear};
    bool allowUpscale{};
    double safetyFactor{1.25};
    std::uint8_t captureBenchmarkInstance{};
    PanelTransferPolicy transferPolicy{PanelTransferPolicy::automatic};

    [[nodiscard]] friend bool operator==(
        const PanelSourceKey&,
        const PanelSourceKey&) noexcept = default;
};

struct PanelSourceEntry
{
    PanelSourceKey key;
    bool active{};
    std::uint8_t consumerMask{};
    double requestedFramesPerSecond{};
    double requestedUploadFramesPerSecond{};
    std::uint64_t captureFrames{};
    std::uint64_t uploadFrames{};
    std::uint64_t repeatedFrames{};
    std::uint64_t droppedFrames{};
    std::uint64_t lastSequence{};
};

struct PanelSourceRegistryResult
{
    bool success{};
    std::string error;
};

class PanelContentRegistry
{
public:
    [[nodiscard]] PanelSourceRegistryResult rebuild(PanelScene& scene);
    [[nodiscard]] std::size_t sourceCount() const noexcept;
    [[nodiscard]] const PanelSourceEntry& source(std::size_t slot) const noexcept;
    [[nodiscard]] PanelSourceEntry& source(std::size_t slot) noexcept;
    [[nodiscard]] std::optional<std::size_t> find(const PanelSourceKey& key) const noexcept;
    void clear() noexcept;

private:
    std::array<PanelSourceEntry, maximumPanelCount> sources_{};
    std::size_t sourceCount_{};
};

[[nodiscard]] PanelSourceKey makePanelSourceKey(const PanelDefinition& panel);
[[nodiscard]] double defaultPanelSourceRate(
    PerformanceProfile profile,
    bool selected,
    bool visible,
    double configuredRate,
    bool explicitRate = false) noexcept;
[[nodiscard]] double defaultPanelSourceRate(
    bool selected,
    bool visible,
    double configuredRate) noexcept;

} // namespace xreal::rendering
