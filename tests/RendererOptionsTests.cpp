#include "rendering/RendererOptions.hpp"

#include <iostream>
#include <string>
#include <vector>

namespace
{
int failures{};
void expect(bool condition, const char* message)
{
    if (!condition) { std::cerr << "FAILED: " << message << '\n'; ++failures; }
}

xreal::rendering::RendererOptionResult parse(std::initializer_list<const char*> values)
{
    std::vector<std::string> storage{"renderer"};
    for (const char* value : values) { storage.emplace_back(value); }
    std::vector<char*> arguments;
    for (auto& value : storage) { arguments.push_back(value.data()); }
    return xreal::rendering::parseRendererOptions(
        static_cast<int>(arguments.size()), arguments.data());
}
}

int main()
{
    using namespace xreal::rendering;
    const auto demo = parse({"--orientation-demo-mode"});
    expect(demo.options.has_value() && demo.options->windowWidth == 1280U
               && demo.options->windowHeight == 720U && demo.options->vsync,
           "demo defaults are valid");
    expect(!parse({"--orientation-demo-mode", "--window-width", "0"}).options.has_value(),
           "invalid window width is rejected");
    expect(!parse({"--orientation-demo-mode", "--monitor-index", "-1"}).options.has_value(),
           "invalid monitor index syntax is rejected");
    expect(!parse({"--orientation-demo-mode", "--field-of-view-degrees", "180"}).options.has_value(),
           "invalid FOV is rejected");
    expect(!parse({"--orientation-demo-mode", "--panel-width", "0"}).options.has_value(),
           "invalid panel size is rejected");
    expect(!parse({"--orientation-demo-mode", "--render-orientation-source", "future"})
                .options.has_value(),
           "invalid orientation source is rejected");
    expect(!parse({"--orientation-demo-mode", "--render-orientation-frame", "camera"})
                .options.has_value(),
           "invalid orientation frame is rejected");
    const auto relative = parse({"--orientation-demo-mode", "--render-orientation-frame",
        "relative", "--recenter-on-start"});
    expect(relative.options.has_value() && relative.options->recenterOnStart,
           "relative frame supports recenter");
    expect(!parse({}).options.has_value(),
           "hardware mode requires explicit calibration inputs");
    expect(!parse({"--apply-gyro-bias", "--accelerometer-profile", "a.json"})
                .options.has_value(),
           "hardware mode requires explicit gyro scale");
    expect(!parse({"--apply-gyro-bias", "--gyro-scale-raw-per-dps", "4090"})
                .options.has_value(),
           "hardware mode requires accelerometer profile");
    const auto measuredHardware = parse({"--apply-gyro-bias", "--gyro-scale-raw-per-dps",
        "4090", "--accelerometer-profile", "a.json", "--render-orientation-source", "measured"});
    expect(measuredHardware.options.has_value() && !measuredHardware.options->predictOrientation,
           "measured hardware mode works without prediction");
    expect(!parse({"--apply-gyro-bias", "--gyro-scale-raw-per-dps", "4090",
                  "--accelerometer-profile", "a.json", "--render-orientation-source", "predicted"})
                .options.has_value(),
           "predicted hardware mode requires explicit prediction");
    expect(!parse({"--orientation-demo-mode", "--render-orientation-source", "measured",
                  "--prediction-horizon-ms", "15"}).options.has_value(),
           "prediction options require predicted source");
    const auto smoke = parse({"--smoke-test", "--smoke-test-frames", "3"});
    expect(smoke.options.has_value() && smoke.options->orientationDemoMode,
           "explicit smoke test uses demo mode without HID");
    if (failures != 0) { return 1; }
    std::cout << "All renderer option tests passed.\n";
    return 0;
}
