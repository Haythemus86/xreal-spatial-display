#pragma once

#include "platform/windows/DisplayTopology.hpp"

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace xreal::platform::windows
{

struct RenderWindowConfig
{
    unsigned int width{1280};
    unsigned int height{720};
    std::optional<int> x;
    std::optional<int> y;
    unsigned int monitorIndex{};
    std::string monitorDeviceName;
    bool fullscreen{};
    bool borderless{};
    bool hidden{};
    std::string title{"XREAL Spatial Renderer"};
};

struct WindowResize
{
    unsigned int width{};
    unsigned int height{};
};

using KeyHandler = std::function<void(unsigned int virtualKey)>;

class RenderWindow
{
public:
    RenderWindow();
    ~RenderWindow();
    RenderWindow(const RenderWindow&) = delete;
    RenderWindow& operator=(const RenderWindow&) = delete;
    RenderWindow(RenderWindow&&) noexcept;
    RenderWindow& operator=(RenderWindow&&) noexcept;

    [[nodiscard]] bool create(const RenderWindowConfig& config, KeyHandler keyHandler);
    [[nodiscard]] bool processMessages() noexcept;
    void waitForMessageWhenMinimized() const noexcept;
    [[nodiscard]] bool minimized() const noexcept;
    [[nodiscard]] std::optional<WindowResize> takePendingResize() noexcept;
    [[nodiscard]] void* nativeHandle() const noexcept;
    void setTitle(const std::string& title) noexcept;
    [[nodiscard]] const std::string& error() const noexcept;

private:
    class Implementation;
    std::unique_ptr<Implementation> implementation_;
};

} // namespace xreal::platform::windows
