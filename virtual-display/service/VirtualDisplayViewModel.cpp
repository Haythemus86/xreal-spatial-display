#include "VirtualDisplayViewModel.hpp"

namespace xreal::virtual_display
{

VirtualDisplayViewModel makeVirtualDisplayViewModel(
    const VirtualDisplayConfiguration& requested,
    const VirtualDisplayStatus& actual,
    bool applyInProgress)
{
    VirtualDisplayViewModel result;
    result.selectedCount = requested.count;
    result.selectedMode = requested.mode;
    result.resolution = requested.resolution;
    result.enabled = requested.enabled;
    result.applyInProgress = applyInProgress;
    result.restartRequired = actual.restartRequired;
    result.actualTopology = actual;
    const auto validation = validateVirtualDisplayConfiguration(requested);
    if (!validation.valid)
    {
        result.validationMessages.push_back(validation.error);
    }
    if (!actual.lastError.empty())
    {
        result.validationMessages.push_back(actual.lastError);
    }
    result.applyEnabled = !applyInProgress && validation.valid
        && !sameRequestedTopology(requested, actual.requested);
    return result;
}

} // namespace xreal::virtual_display
