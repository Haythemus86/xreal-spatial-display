#include "protocol/VirtualDisplayProtocol.hpp"
#include "service/VirtualDisplayService.hpp"

#include <array>
#include <cstddef>
#include <cstring>
#include <functional>
#include <iostream>
#include <span>
#include <string_view>

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
        std::cout << "Virtual-display driver/protocol tests: " << count_
                  << ", failures: " << failures_ << '\n';
        return failures_ == 0U ? 0 : 1;
    }

private:
    std::size_t count_{};
    std::size_t failures_{};
};

[[nodiscard]] VirtualDisplayConfiguration validConfiguration()
{
    VirtualDisplayConfiguration result;
    result.enabled = true;
    result.count = VirtualDisplayCount::three;
    result.mode = VirtualDisplayMode::extended;
    return result;
}

[[nodiscard]] VirtualDisplayProtocolValidation validate(
    const VirtualDisplayProtocolRequest& request)
{
    return validateProtocolRequest(std::as_bytes(std::span{&request, 1U}));
}

} // namespace

int main()
{
    TestSuite tests;

    tests.run("protocol version valid", [] {
        const auto request = makeProtocolRequest(VirtualDisplayCommand::getStatus);
        const auto bytes = std::as_bytes(std::span{&request, 1U});
        return validate(request).valid
            && !validateProtocolRequest(bytes.first(bytes.size() - 1U)).valid;
    });
    tests.run("invalid protocol version rejected", [] {
        auto request = makeProtocolRequest(VirtualDisplayCommand::getStatus);
        ++request.version;
        return !validate(request).valid;
    });
    tests.run("malformed count rejected", [] {
        auto request = makeProtocolRequest(
            VirtualDisplayCommand::applyConfiguration, validConfiguration());
        request.configuration.count = 4U;
        const auto result = validate(request);
        return !result.valid && result.errorCode == VirtualDisplayErrorCode::invalidCount;
    });
    tests.run("malformed mode rejected", [] {
        auto request = makeProtocolRequest(
            VirtualDisplayCommand::applyConfiguration, validConfiguration());
        request.configuration.mode = 99U;
        const auto result = validate(request);
        return !result.valid && result.errorCode == VirtualDisplayErrorCode::invalidMode;
    });
    tests.run("unsupported resolution rejected", [] {
        auto request = makeProtocolRequest(
            VirtualDisplayCommand::applyConfiguration, validConfiguration());
        request.configuration.width = 1280U;
        const auto result = validate(request);
        return !result.valid
            && result.errorCode == VirtualDisplayErrorCode::unsupportedResolution;
    });
    tests.run("unsupported refresh rejected", [] {
        auto request = makeProtocolRequest(
            VirtualDisplayCommand::applyConfiguration, validConfiguration());
        request.configuration.refreshHertz = 120U;
        const auto result = validate(request);
        return !result.valid
            && result.errorCode == VirtualDisplayErrorCode::unsupportedRefreshRate;
    });
    tests.run("response includes actual topology", [] {
        VirtualDisplayStateMachine model;
        const auto response = model.process(makeProtocolRequest(
            VirtualDisplayCommand::applyConfiguration, validConfiguration()));
        return response.accepted == 1U && response.monitorCount == 3U
            && response.actual.count == 3U;
    });
    tests.run("response includes error code", [] {
        VirtualDisplayStateMachine model;
        auto request = makeProtocolRequest(
            VirtualDisplayCommand::applyConfiguration, validConfiguration());
        request.configuration.width = 17U;
        const auto response = model.process(request);
        return response.accepted == 0U
            && response.errorCode == static_cast<std::uint32_t>(
                VirtualDisplayErrorCode::unsupportedResolution);
    });
    tests.run("response numeric fields are finite fixed integers", [] {
        VirtualDisplayStateMachine model;
        const auto response = model.process(makeProtocolRequest(
            VirtualDisplayCommand::applyConfiguration, validConfiguration()));
        return response.byteSize == sizeof(response)
            && response.monitorCount <= maximumVirtualDisplayCount
            && response.actual.width == defaultVirtualDisplayWidth
            && response.actual.height == defaultVirtualDisplayHeight
            && response.actual.refreshHertz == defaultVirtualDisplayRefreshHertz;
    });

    return tests.result();
}
