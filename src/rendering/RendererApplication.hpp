#pragma once

#include "rendering/RendererOptions.hpp"

#include <string>

namespace xreal::rendering
{

class RendererApplication
{
public:
    explicit RendererApplication(RendererOptions options);
    [[nodiscard]] int run();

private:
    RendererOptions options_;
};

} // namespace xreal::rendering
