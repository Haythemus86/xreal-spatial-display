#include "platform/windows/DisplayTopology.hpp"

#include <array>
#include <iostream>

namespace
{

int failures{};

void expect(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << "FAILED: " << message << '\n';
        ++failures;
    }
}

void testDxgiAssociation()
{
    using namespace xreal::platform::windows;
    std::array monitors{
        MonitorInformation{0U, R"(\\.\DISPLAY1)", 0, 0, 2560, 1440,
                           0, 0, 2560, 1400, true, std::nullopt},
        MonitorInformation{1U, R"(\\.\DISPLAY5)", 2560, 0, 4480, 1080,
                           2560, 0, 4480, 1040, false, std::nullopt},
    };
    std::array outputs{
        DxgiOutputInformation{1U, 0U, R"(\\.\display5)", 2560, 0, 4480, 1080,
                              true, "Integrated GPU", {0x55667788U, 0x11223344}},
        DxgiOutputInformation{0U, 0U, R"(\\.\DISPLAY1)", 0, 0, 2560, 1440,
                              true, "Discrete GPU", {0xAABBCCDDU, 0x01020304}},
    };
    associateDxgiOutputs(monitors, outputs);
    expect(monitors[0].dxgiOutput.has_value()
               && monitors[0].dxgiOutput->adapterDescription == "Discrete GPU"
               && monitors[0].dxgiOutput->adapterIndex == 0U,
           "primary monitor matches its owning DXGI adapter by device name");
    expect(monitors[1].dxgiOutput.has_value()
               && monitors[1].dxgiOutput->adapterDescription == "Integrated GPU"
               && monitors[1].dxgiOutput->adapterLuid == outputs[0].adapterLuid,
           "secondary monitor matches case-insensitively without using enumeration order");

    outputs[0].attachedToDesktop = false;
    associateDxgiOutputs(monitors, outputs);
    expect(!monitors[1].dxgiOutput.has_value(),
           "an inactive DXGI output is not silently associated by matching bounds");
}

void testWindowPlacement()
{
    using namespace xreal::platform::windows;
    const MonitorInformation monitor{
        1U, R"(\\.\DISPLAY5)", 2560, -200, 4480, 880,
        2560, -200, 4480, 840, false, std::nullopt};
    const auto centered = calculateWindowPlacement(
        monitor, 1280U, 720U, std::nullopt, std::nullopt, false);
    expect(centered.has_value() && centered->x == 2880 && centered->y == -20
               && centered->width == 1280U && centered->height == 720U,
           "windowed rendering is centered inside selected monitor bounds");

    const auto relative = calculateWindowPlacement(monitor, 800U, 600U, 100, 50, false);
    expect(relative.has_value() && relative->x == 2660 && relative->y == -150,
           "explicit window coordinates are relative to the selected monitor");

    const auto clamped = calculateWindowPlacement(monitor, 800U, 600U, 5000, -5000, false);
    expect(clamped.has_value() && clamped->x == 3680 && clamped->y == -200,
           "window placement is clamped instead of escaping to another display");

    const auto oversized = calculateWindowPlacement(
        monitor, 4000U, 2000U, std::nullopt, std::nullopt, false);
    expect(oversized.has_value() && oversized->x == 2560 && oversized->y == -200
               && oversized->width == 1920U && oversized->height == 1080U,
           "oversized windows are constrained to the selected monitor");

    const auto fullscreen = calculateWindowPlacement(monitor, 1U, 1U, 100, 100, true);
    expect(fullscreen.has_value() && fullscreen->x == 2560 && fullscreen->y == -200
               && fullscreen->width == 1920U && fullscreen->height == 1080U,
           "fullscreen uses the complete selected monitor bounds");

    auto invalid = monitor;
    invalid.right = invalid.left;
    expect(!calculateWindowPlacement(
                invalid, 1280U, 720U, std::nullopt, std::nullopt, false).has_value(),
           "invalid monitor geometry is rejected");
}

void testLuidText()
{
    using namespace xreal::platform::windows;
    expect(dxgiAdapterLuidText({0x89ABCDEFU, 0x01234567}) == "0x01234567:89ABCDEF",
           "adapter LUID text preserves high and low 32-bit parts");
}

void testStableIdentitySelection()
{
    using namespace xreal::platform::windows;
    std::array monitors{
        MonitorInformation{0U, R"(\\.\DISPLAY1)", 0, 0, 1920, 1080,
            0, 0, 1920, 1040, true, std::nullopt, "stable-one", "First"},
        MonitorInformation{1U, R"(\\.\DISPLAY2)", 1920, 0, 3840, 1080,
            1920, 0, 3840, 1040, false, std::nullopt, "stable-two", "Second"},
    };
    expect(findMonitorByStableIdentity(monitors, "stable-two") == &monitors[1],
        "stable identity resolves independently from monitor index");
    expect(findMonitorByStableIdentity(monitors, "missing") == nullptr,
        "unknown stable identity fails without a fallback");
}

} // namespace

int main()
{
    testDxgiAssociation();
    testWindowPlacement();
    testLuidText();
    testStableIdentitySelection();
    if (failures != 0)
    {
        std::cerr << failures << " display topology test(s) failed.\n";
        return 1;
    }
    std::cout << "All display topology tests passed.\n";
    return 0;
}
