#include "capture/DesktopCaptureManager.hpp"

#include <cmath>
#include <utility>

namespace xreal::capture
{

struct DesktopCaptureManager::Runtime
{
    explicit Runtime(bool syntheticSource) : synthetic(syntheticSource)
    {
        if (synthetic)
        {
            syntheticCapture = std::make_unique<DesktopSyntheticCapture>(bridge);
        }
        else
        {
            duplicationCapture = std::make_unique<DesktopDuplicationCapture>(bridge);
        }
    }

    DesktopCaptureBridge bridge;
    bool synthetic{};
    std::unique_ptr<DesktopDuplicationCapture> duplicationCapture;
    std::unique_ptr<DesktopSyntheticCapture> syntheticCapture;
};

DesktopCaptureManager::DesktopCaptureManager() = default;
DesktopCaptureManager::~DesktopCaptureManager() { stop(); }

bool DesktopCaptureManager::startSource(DesktopCaptureSourceConfig config)
{
    if (config.sourceSlot >= sources_.size() || sources_[config.sourceSlot])
    {
        return false;
    }
    const std::size_t sourceSlot = config.sourceSlot;
    auto runtime = std::make_unique<Runtime>(config.synthetic);
    const bool started = config.synthetic
        ? runtime->syntheticCapture->start(std::move(config.syntheticConfig))
        : runtime->duplicationCapture->start(std::move(config.duplication));
    if (!started)
    {
        lastErrors_[sourceSlot] = config.synthetic
            ? runtime->syntheticCapture->error()
            : runtime->duplicationCapture->error();
        return false;
    }
    lastErrors_[sourceSlot].clear();
    sources_[sourceSlot] = std::move(runtime);
    return true;
}

void DesktopCaptureManager::setMaximumFramesPerSecond(
    std::size_t sourceSlot,
    double value) noexcept
{
    if (sourceSlot < sources_.size() && sources_[sourceSlot]
        && std::isfinite(value) && value >= 0.0)
    {
        if (sources_[sourceSlot]->synthetic)
        {
            sources_[sourceSlot]->syntheticCapture->setMaximumFramesPerSecond(value);
        }
        else
        {
            sources_[sourceSlot]->duplicationCapture->setMaximumFramesPerSecond(value);
        }
    }
}

std::optional<DesktopCaptureFrame> DesktopCaptureManager::tryLatest(
    std::size_t sourceSlot)
{
    return sourceSlot < sources_.size() && sources_[sourceSlot]
        ? sources_[sourceSlot]->bridge.tryLatest() : std::nullopt;
}

DesktopCaptureStatistics DesktopCaptureManager::statistics(std::size_t sourceSlot) const
{
    return sourceSlot < sources_.size() && sources_[sourceSlot]
        ? sources_[sourceSlot]->synthetic
            ? sources_[sourceSlot]->syntheticCapture->statistics()
            : sources_[sourceSlot]->duplicationCapture->statistics()
        : DesktopCaptureStatistics{};
}

DesktopCaptureStatistics DesktopCaptureManager::detailedStatistics(
    std::size_t sourceSlot) const
{
    return sourceSlot < sources_.size() && sources_[sourceSlot]
        ? sources_[sourceSlot]->synthetic
            ? sources_[sourceSlot]->syntheticCapture->detailedStatistics()
            : sources_[sourceSlot]->duplicationCapture->detailedStatistics()
        : DesktopCaptureStatistics{};
}

DesktopCaptureBridgeStatistics DesktopCaptureManager::bridgeStatistics(
    std::size_t sourceSlot) const
{
    return sourceSlot < sources_.size() && sources_[sourceSlot]
        ? sources_[sourceSlot]->bridge.statistics() : DesktopCaptureBridgeStatistics{};
}

std::string DesktopCaptureManager::error(std::size_t sourceSlot) const
{
    return sourceSlot < sources_.size() && sources_[sourceSlot]
        ? sources_[sourceSlot]->synthetic
            ? sources_[sourceSlot]->syntheticCapture->error()
            : sources_[sourceSlot]->duplicationCapture->error()
        : sourceSlot < lastErrors_.size() && !lastErrors_[sourceSlot].empty()
            ? lastErrors_[sourceSlot] : "Desktop capture source is not active.";
}

bool DesktopCaptureManager::active(std::size_t sourceSlot) const noexcept
{
    return sourceSlot < sources_.size() && sources_[sourceSlot] != nullptr;
}

std::size_t DesktopCaptureManager::activeSourceCount() const noexcept
{
    std::size_t result{};
    for (const auto& source : sources_)
    {
        result += static_cast<std::size_t>(source != nullptr);
    }
    return result;
}

void DesktopCaptureManager::stopSource(std::size_t sourceSlot)
{
    if (sourceSlot < sources_.size() && sources_[sourceSlot])
    {
        if (sources_[sourceSlot]->synthetic)
        {
            sources_[sourceSlot]->syntheticCapture->stop();
        }
        else
        {
            sources_[sourceSlot]->duplicationCapture->stop();
        }
        sources_[sourceSlot].reset();
    }
}

void DesktopCaptureManager::stop()
{
    for (std::size_t slot = 0U; slot < sources_.size(); ++slot)
    {
        stopSource(slot);
    }
}

} // namespace xreal::capture
