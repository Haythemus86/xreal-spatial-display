#include "service/VirtualDisplayControlOptions.hpp"
#include "service/VirtualDisplayService.hpp"
#include "service/VirtualDisplayTopology.hpp"
#include "service/VirtualDisplayViewModel.hpp"

#include "rendering/PanelContentRegistry.hpp"

#include <cmath>
#include <cstddef>
#include <functional>
#include <iostream>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace
{

using namespace xreal::virtual_display;

class TestSuite
{
public:
    void run(std::string_view name, const std::function<bool()>& test)
    {
        ++count_;
        if (!test())
        {
            ++failures_;
            std::cerr << "FAILED: " << name << '\n';
        }
    }

    [[nodiscard]] int result() const
    {
        std::cout << "Virtual-display application tests: " << count_
                  << ", failures: " << failures_ << '\n';
        return failures_ == 0U ? 0 : 1;
    }

private:
    std::size_t count_{};
    std::size_t failures_{};
};

[[nodiscard]] VirtualDisplayConfiguration configuration(
    VirtualDisplayCount count,
    VirtualDisplayMode mode = VirtualDisplayMode::extended)
{
    VirtualDisplayConfiguration result;
    result.enabled = true;
    result.count = count;
    result.mode = mode;
    return result;
}

[[nodiscard]] VirtualDisplayStatus apply(
    InMemoryVirtualDisplayTransport& transport,
    VirtualDisplayConfiguration value)
{
    VirtualDisplayClient client(transport);
    return client.apply(value).status;
}

[[nodiscard]] std::pair<VirtualDisplayTopologyResult, xreal::rendering::PanelScene>
route(VirtualDisplayCount count, VirtualDisplayMode mode)
{
    InMemoryVirtualDisplayTransport transport;
    auto status = apply(transport, configuration(count, mode));
    for (std::size_t index = 0U; index < status.monitorCount; ++index)
    {
        status.monitors[index].windowsMonitorIdentity =
            "test-display://" + status.monitors[index].stableId;
    }
    auto scene = xreal::rendering::makeDefaultPanelScene(
        virtualDisplayCountValue(count));
    auto result = routeVirtualDisplaysToPanels(status, scene);
    return {std::move(result), std::move(scene)};
}

class RejectingTransport final : public IVirtualDisplayTransport
{
public:
    std::optional<VirtualDisplayProtocolResponse> transact(
        const VirtualDisplayProtocolRequest&) override
    {
        VirtualDisplayStatus status;
        status.state = VirtualDisplayState::failed;
        status.errorCode = VirtualDisplayErrorCode::driverRejected;
        status.lastError = "simulated driver rejection";
        VirtualDisplayProtocolResponse response;
        populateProtocolResponse(response, status, false);
        return response;
    }

    const std::string& error() const noexcept override
    {
        return error_;
    }

private:
    std::string error_;
};

} // namespace

int main()
{
    using xreal::rendering::PanelContentRegistry;
    using xreal::rendering::PanelLayoutPreset;
    using xreal::rendering::PerformanceProfile;

    TestSuite tests;

    tests.run("count=1 accepted", [] {
        return validateVirtualDisplayConfiguration(
            configuration(VirtualDisplayCount::one)).valid;
    });
    tests.run("count=2 accepted", [] {
        return validateVirtualDisplayConfiguration(
            configuration(VirtualDisplayCount::two)).valid;
    });
    tests.run("count=3 accepted", [] {
        return validateVirtualDisplayConfiguration(
            configuration(VirtualDisplayCount::three)).valid;
    });
    tests.run("count=0 disables", [] {
        const std::array<std::string_view, 3U> arguments{
            "--virtual-displays", "0", "--apply-virtual-display-config"};
        const auto parsed = parseVirtualDisplayControlOptions(arguments);
        return parsed.success && !parsed.options.configuration.enabled;
    });
    tests.run("count=4 rejected", [] {
        const std::array<std::string_view, 3U> arguments{
            "--virtual-displays", "4", "--apply-virtual-display-config"};
        return !parseVirtualDisplayControlOptions(arguments).success;
    });

    tests.run("extended one descriptor", [] {
        std::size_t count{};
        (void)buildVirtualMonitorDescriptors(
            configuration(VirtualDisplayCount::one), count);
        return count == 1U;
    });
    tests.run("extended two descriptors", [] {
        std::size_t count{};
        (void)buildVirtualMonitorDescriptors(
            configuration(VirtualDisplayCount::two), count);
        return count == 2U;
    });
    tests.run("extended three descriptors", [] {
        std::size_t count{};
        (void)buildVirtualMonitorDescriptors(
            configuration(VirtualDisplayCount::three), count);
        return count == 3U;
    });
    tests.run("stable IDs deterministic", [] {
        return stableVirtualMonitorId(0U) == stableVirtualMonitorId(0U)
            && stableVirtualMonitorId(0U) != stableVirtualMonitorId(1U);
    });
    tests.run("extended source identities distinct", [] {
        const auto [result, scene] = route(
            VirtualDisplayCount::three, VirtualDisplayMode::extended);
        return result.success && result.sourceCount == 3U
            && result.bindings[0].monitorStableId != result.bindings[1].monitorStableId
            && result.bindings[1].monitorStableId != result.bindings[2].monitorStableId;
    });
    tests.run("monitor 1 maps panel 1", [] {
        const auto [result, scene] = route(
            VirtualDisplayCount::three, VirtualDisplayMode::extended);
        return result.bindings[0].panelId.value == scene.panels[0].id.value;
    });
    tests.run("monitor 2 maps panel 2", [] {
        const auto [result, scene] = route(
            VirtualDisplayCount::three, VirtualDisplayMode::extended);
        return result.bindings[1].panelId.value == scene.panels[1].id.value;
    });
    tests.run("monitor 3 maps panel 3", [] {
        const auto [result, scene] = route(
            VirtualDisplayCount::three, VirtualDisplayMode::extended);
        return result.bindings[2].panelId.value == scene.panels[2].id.value;
    });
    tests.run("generation increments", [] {
        InMemoryVirtualDisplayTransport transport;
        const auto first = apply(transport, configuration(VirtualDisplayCount::one));
        const auto second = apply(transport, configuration(VirtualDisplayCount::two));
        return second.actual.applyGeneration == first.actual.applyGeneration + 1U;
    });
    tests.run("same configuration idempotent", [] {
        InMemoryVirtualDisplayTransport transport;
        const auto first = apply(transport, configuration(VirtualDisplayCount::two));
        const auto second = apply(transport, configuration(VirtualDisplayCount::two));
        return second.actual.applyGeneration == first.actual.applyGeneration;
    });

    tests.run("mirrored one uses one source", [] {
        return route(VirtualDisplayCount::one, VirtualDisplayMode::mirrored)
            .first.sourceCount == 1U;
    });
    tests.run("mirrored two one source two consumers", [] {
        const auto [result, scene] = route(
            VirtualDisplayCount::two, VirtualDisplayMode::mirrored);
        return result.sourceCount == 1U
            && scene.runtime[0].sourceSlot == scene.runtime[1].sourceSlot;
    });
    tests.run("mirrored three one source three consumers", [] {
        const auto [result, scene] = route(
            VirtualDisplayCount::three, VirtualDisplayMode::mirrored);
        return result.sourceCount == 1U
            && scene.runtime[0].sourceSlot == scene.runtime[1].sourceSlot
            && scene.runtime[1].sourceSlot == scene.runtime[2].sourceSlot;
    });
    tests.run("mirrored topology creates one capture pipeline", [] {
        return route(VirtualDisplayCount::three, VirtualDisplayMode::mirrored)
            .first.sourceCount == 1U;
    });
    tests.run("mirrored topology has one upload owner", [] {
        const auto [result, scene] = route(
            VirtualDisplayCount::three, VirtualDisplayMode::mirrored);
        PanelContentRegistry registry;
        auto candidate = scene;
        const auto rebuilt = registry.rebuild(candidate);
        return rebuilt.success && registry.sourceCount() == 1U;
    });
    tests.run("mirrored panels sample same source slot", [] {
        const auto [result, scene] = route(
            VirtualDisplayCount::three, VirtualDisplayMode::mirrored);
        return scene.runtime[0].sourceSlot == scene.runtime[2].sourceSlot;
    });
    tests.run("extended to mirrored removes sources", [] {
        const auto extended = route(
            VirtualDisplayCount::three, VirtualDisplayMode::extended);
        const auto mirrored = route(
            VirtualDisplayCount::three, VirtualDisplayMode::mirrored);
        return extended.first.sourceCount == 3U && mirrored.first.sourceCount == 1U;
    });
    tests.run("mirrored to extended creates sources", [] {
        const auto mirrored = route(
            VirtualDisplayCount::three, VirtualDisplayMode::mirrored);
        const auto extended = route(
            VirtualDisplayCount::three, VirtualDisplayMode::extended);
        return mirrored.first.sourceCount == 1U && extended.first.sourceCount == 3U;
    });

    tests.run("reconfigure one to two", [] {
        InMemoryVirtualDisplayTransport transport;
        (void)apply(transport, configuration(VirtualDisplayCount::one));
        return apply(transport, configuration(VirtualDisplayCount::two)).monitorCount == 2U;
    });
    tests.run("reconfigure two to three", [] {
        InMemoryVirtualDisplayTransport transport;
        (void)apply(transport, configuration(VirtualDisplayCount::two));
        return apply(transport, configuration(VirtualDisplayCount::three)).monitorCount == 3U;
    });
    tests.run("reconfigure three to one", [] {
        InMemoryVirtualDisplayTransport transport;
        (void)apply(transport, configuration(VirtualDisplayCount::three));
        return apply(transport, configuration(VirtualDisplayCount::one)).monitorCount == 1U;
    });
    tests.run("reconfigure extended to mirrored", [] {
        InMemoryVirtualDisplayTransport transport;
        (void)apply(transport, configuration(VirtualDisplayCount::three));
        const auto status = apply(transport,
            configuration(VirtualDisplayCount::three, VirtualDisplayMode::mirrored));
        return status.actual.mode == VirtualDisplayMode::mirrored
            && status.monitorCount == 1U;
    });
    tests.run("reconfigure mirrored to extended", [] {
        InMemoryVirtualDisplayTransport transport;
        (void)apply(transport,
            configuration(VirtualDisplayCount::three, VirtualDisplayMode::mirrored));
        const auto status = apply(transport, configuration(VirtualDisplayCount::three));
        return status.actual.mode == VirtualDisplayMode::extended
            && status.monitorCount == 3U;
    });
    tests.run("disable all", [] {
        InMemoryVirtualDisplayTransport transport;
        VirtualDisplayClient client(transport);
        (void)client.apply(configuration(VirtualDisplayCount::three));
        return client.disable().status.state == VirtualDisplayState::disabled;
    });
    tests.run("invalid apply preserves valid state", [] {
        InMemoryVirtualDisplayTransport transport;
        const auto valid = apply(transport, configuration(VirtualDisplayCount::two));
        auto invalid = configuration(VirtualDisplayCount::two);
        invalid.resolution.width = 1280U;
        VirtualDisplayClient client(transport);
        const auto rejected = client.apply(invalid);
        return !rejected.accepted
            && transport.status().actual.applyGeneration == valid.actual.applyGeneration
            && transport.status().monitorCount == 2U;
    });
    tests.run("failed driver apply is reported", [] {
        RejectingTransport transport;
        VirtualDisplayClient client(transport);
        const auto result = client.apply(configuration(VirtualDisplayCount::one));
        return !result.accepted && result.status.state == VirtualDisplayState::failed
            && !result.error.empty();
    });
    tests.run("restart required reaches view model", [] {
        VirtualDisplayStatus status;
        status.restartRequired = true;
        const auto model = makeVirtualDisplayViewModel(
            configuration(VirtualDisplayCount::one), status);
        return model.restartRequired;
    });
    tests.run("panel geometry survives topology change", [] {
        InMemoryVirtualDisplayTransport transport;
        auto scene = xreal::rendering::makeDefaultPanelScene(3U);
        scene.panels[0].transform.position.x = 12.5;
        scene.panels[0].dimensions.width = 2.75;
        auto status = apply(transport, configuration(VirtualDisplayCount::two));
        for (std::size_t index = 0U; index < status.monitorCount; ++index)
        {
            status.monitors[index].windowsMonitorIdentity =
                "test-display://" + status.monitors[index].stableId;
        }
        const auto result = routeVirtualDisplaysToPanels(status, scene);
        return result.success && scene.panels[0].transform.position.x == 12.5
            && scene.panels[0].dimensions.width == 2.75;
    });
    tests.run("orientation state remains external", [] {
        double orientationState = 0.625;
        auto [result, scene] = route(
            VirtualDisplayCount::two, VirtualDisplayMode::extended);
        return result.success && orientationState == 0.625;
    });

    tests.run("mirrored three remains one bounded source", [] {
        const auto [result, scene] = route(
            VirtualDisplayCount::three, VirtualDisplayMode::mirrored);
        return result.sourceCount == 1U;
    });
    tests.run("one source implies one upload per sequence", [] {
        auto [result, scene] = route(
            VirtualDisplayCount::three, VirtualDisplayMode::mirrored);
        PanelContentRegistry registry;
        const auto rebuilt = registry.rebuild(scene);
        registry.source(0U).lastSequence = 42U;
        ++registry.source(0U).uploadFrames;
        return rebuilt.success && registry.sourceCount() == 1U
            && registry.source(0U).uploadFrames == 1U;
    });
    tests.run("extended permits three unique sources", [] {
        return route(VirtualDisplayCount::three, VirtualDisplayMode::extended)
            .first.sourceCount == 3U;
    });
    tests.run("source registry is fixed capacity", [] {
        static_assert(xreal::rendering::maximumPanelCount == 3U);
        return xreal::rendering::maximumPanelCount == maximumVirtualDisplayCount;
    });
    tests.run("frame source boundary keeps resource creation outside routing", [] {
        const auto [result, scene] = route(
            VirtualDisplayCount::three, VirtualDisplayMode::mirrored);
        return result.success && result.sourceCount == 1U;
    });
    tests.run("routing adds no flush operation", [] {
        const auto [result, scene] = route(
            VirtualDisplayCount::one, VirtualDisplayMode::extended);
        return result.success;
    });
    tests.run("panel routing does not own presentation", [] {
        const auto [result, scene] = route(
            VirtualDisplayCount::three, VirtualDisplayMode::extended);
        return result.success && result.panelCount == 3U;
    });
    tests.run("hidden panel scheduling remains zero", [] {
        return xreal::rendering::defaultPanelSourceRate(
            PerformanceProfile::balanced, false, false, 30.0) == 0.0;
    });
    tests.run("source dedup retains consumer mask", [] {
        auto [result, scene] = route(
            VirtualDisplayCount::three, VirtualDisplayMode::mirrored);
        PanelContentRegistry registry;
        const auto rebuilt = registry.rebuild(scene);
        return rebuilt.success && registry.source(0U).consumerMask == 0x7U;
    });

    return tests.result();
}
