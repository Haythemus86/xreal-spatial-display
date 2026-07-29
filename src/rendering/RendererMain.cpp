#include "rendering/RendererApplication.hpp"
#include "rendering/RendererOptions.hpp"

#include <iostream>

int main(int argc, char* argv[])
{
    const auto parsed = xreal::rendering::parseRendererOptions(argc, argv);
    if (!parsed.options.has_value())
    {
        std::cerr << parsed.error << '\n' << xreal::rendering::rendererUsage();
        return 2;
    }
    if (parsed.options->help)
    {
        std::cout << xreal::rendering::rendererUsage();
        return 0;
    }
    xreal::rendering::RendererApplication application(*parsed.options);
    return application.run();
}
