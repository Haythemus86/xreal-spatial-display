#include "VirtualDisplayControlOptions.hpp"
#include "VirtualDisplayService.hpp"
#include "WindowsVirtualDisplayTransport.hpp"

#include <iostream>
#include <string_view>
#include <vector>

namespace
{

void printStatus(const xreal::virtual_display::VirtualDisplayStatus& status)
{
    using namespace xreal::virtual_display;
    std::cout << "virtual_display_enabled=" << (status.actual.enabled ? "yes" : "no")
              << "\nvirtual_display_requested_count="
              << (status.requested.enabled
                      ? virtualDisplayCountValue(status.requested.count) : 0U)
              << "\nvirtual_display_actual_count=" << status.monitorCount
              << "\nvirtual_display_mode=" << virtualDisplayModeText(status.requested.mode)
              << "\nvirtual_display_resolution=" << status.requested.resolution.width
              << 'x' << status.requested.resolution.height
              << "\nvirtual_display_refresh_hz=" << status.requested.resolution.refreshHertz
              << "\nvirtual_display_generation=" << status.actual.applyGeneration
              << "\nvirtual_display_state=" << virtualDisplayStateText(status.state)
              << "\nvirtual_display_restart_required="
              << (status.restartRequired ? "yes" : "no") << '\n';
    for (std::size_t index = 0U; index < status.monitorCount; ++index)
    {
        const auto& monitor = status.monitors[index];
        std::cout << "monitor=" << monitor.logicalIndex + 1U
                  << " stable_id=" << monitor.stableId
                  << " friendly_name=\"" << monitor.friendlyName << "\""
                  << " windows_device="
                  << (monitor.windowsDeviceName.empty()
                          ? "pending_assignment" : monitor.windowsDeviceName)
                  << " windows_identity="
                  << (monitor.windowsMonitorIdentity.empty()
                          ? "pending_assignment" : monitor.windowsMonitorIdentity)
                  << " active=" << (monitor.active ? "yes" : "no")
                  << " mode=" << monitor.resolution.width << 'x'
                  << monitor.resolution.height << '@'
                  << monitor.resolution.refreshHertz << '\n';
    }
    if (!status.lastError.empty())
    {
        std::cerr << "virtual_display_error=" << status.lastError << '\n';
    }
}

} // namespace

int main(int argc, char** argv)
{
    using namespace xreal::virtual_display;
    std::vector<std::string_view> arguments;
    arguments.reserve(argc > 1 ? static_cast<std::size_t>(argc - 1) : 0U);
    for (int index = 1; index < argc; ++index)
    {
        arguments.emplace_back(argv[index]);
    }
    const auto parsed = parseVirtualDisplayControlOptions(arguments);
    if (!parsed.success)
    {
        std::cerr << parsed.error << '\n' << virtualDisplayControlUsage();
        return 2;
    }
    if (parsed.options.help)
    {
        std::cout << virtualDisplayControlUsage();
        return 0;
    }
    WindowsVirtualDisplayTransport transport;
    VirtualDisplayClient client(transport);
    VirtualDisplayClientResult result;
    if (parsed.options.disable
        || (parsed.options.apply && !parsed.options.configuration.enabled))
    {
        result = client.disable();
    }
    else if (parsed.options.apply)
    {
        std::cout << "Applying explicit virtual-display configuration.\n";
        result = client.apply(parsed.options.configuration);
    }
    else
    {
        result = client.status();
    }
    if (!result.accepted)
    {
        std::cerr << (result.error.empty()
                ? "The virtual-display driver rejected the request." : result.error)
                  << '\n';
        return 1;
    }
    if (parsed.options.showStatus || parsed.options.apply || parsed.options.disable)
    {
        printStatus(result.status);
    }
    return 0;
}
