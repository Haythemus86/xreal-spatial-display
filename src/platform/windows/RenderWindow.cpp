#include "platform/windows/RenderWindow.hpp"

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

#include <array>
#include <algorithm>
#include <cctype>
#include <utility>

namespace xreal::platform::windows
{
namespace
{

[[nodiscard]] std::string utf8(const wchar_t* text)
{
    if (text == nullptr || *text == L'\0')
    {
        return {};
    }
    const int size = WideCharToMultiByte(CP_UTF8, 0, text, -1, nullptr, 0, nullptr, nullptr);
    if (size <= 1)
    {
        return {};
    }
    std::string result(static_cast<std::size_t>(size), '\0');
    (void)WideCharToMultiByte(CP_UTF8, 0, text, -1, result.data(), size, nullptr, nullptr);
    result.pop_back();
    return result;
}

[[nodiscard]] std::wstring wide(const std::string& text)
{
    if (text.empty())
    {
        return {};
    }
    const int size = MultiByteToWideChar(CP_UTF8, 0, text.data(),
        static_cast<int>(text.size()), nullptr, 0);
    std::wstring result(static_cast<std::size_t>(size), L'\0');
    (void)MultiByteToWideChar(CP_UTF8, 0, text.data(),
        static_cast<int>(text.size()), result.data(), size);
    return result;
}

[[nodiscard]] bool sameDeviceName(const std::string& left, const std::string& right)
{
    if (left.size() != right.size())
    {
        return false;
    }
    return std::equal(left.begin(), left.end(), right.begin(), [](char leftCharacter, char rightCharacter) {
        return std::toupper(static_cast<unsigned char>(leftCharacter))
            == std::toupper(static_cast<unsigned char>(rightCharacter));
    });
}

struct MonitorRecord
{
    HMONITOR handle{};
    MonitorInformation information;
};

BOOL CALLBACK collectMonitor(HMONITOR handle, HDC, LPRECT, LPARAM context)
{
    auto& records = *reinterpret_cast<std::vector<MonitorRecord>*>(context);
    MONITORINFOEXW information{};
    information.cbSize = sizeof(information);
    if (!GetMonitorInfoW(handle, &information))
    {
        return TRUE;
    }
    MonitorInformation result;
    result.index = static_cast<unsigned int>(records.size());
    result.deviceName = utf8(information.szDevice);
    result.left = information.rcMonitor.left;
    result.top = information.rcMonitor.top;
    result.right = information.rcMonitor.right;
    result.bottom = information.rcMonitor.bottom;
    result.workLeft = information.rcWork.left;
    result.workTop = information.rcWork.top;
    result.workRight = information.rcWork.right;
    result.workBottom = information.rcWork.bottom;
    result.primary = (information.dwFlags & MONITORINFOF_PRIMARY) != 0U;
    records.push_back({handle, std::move(result)});
    return TRUE;
}

[[nodiscard]] std::vector<MonitorRecord> monitorRecords()
{
    std::vector<MonitorRecord> result;
    (void)EnumDisplayMonitors(nullptr, nullptr, collectMonitor,
        reinterpret_cast<LPARAM>(&result));
    std::stable_sort(result.begin(), result.end(), [](const auto& left, const auto& right) {
        return left.information.primary && !right.information.primary;
    });
    for (std::size_t index = 0; index < result.size(); ++index)
    {
        result[index].information.index = static_cast<unsigned int>(index);
    }
    return result;
}

} // namespace

class RenderWindow::Implementation
{
public:
    ~Implementation()
    {
        if (window_ != nullptr)
        {
            DestroyWindow(window_);
        }
        if (classRegistered_)
        {
            UnregisterClassW(className, instance_);
        }
    }

    [[nodiscard]] bool create(const RenderWindowConfig& config, KeyHandler handler)
    {
        if (config.width == 0U || config.height == 0U)
        {
            error_ = "Window dimensions must be positive.";
            return false;
        }
        const auto monitors = monitorRecords();
        const auto selected = !config.monitorDeviceName.empty()
            ? std::find_if(monitors.begin(), monitors.end(), [&](const auto& monitor) {
                return sameDeviceName(monitor.information.deviceName, config.monitorDeviceName);
            })
            : (config.monitorIndex < monitors.size()
                ? monitors.begin() + config.monitorIndex : monitors.end());
        if (selected == monitors.end())
        {
            error_ = config.monitorDeviceName.empty()
                ? "Requested monitor index does not exist."
                : "The selected physical monitor " + config.monitorDeviceName
                    + " is no longer active; the window was not moved to another display.";
            return false;
        }
        keyHandler_ = std::move(handler);
        instance_ = GetModuleHandleW(nullptr);
        WNDCLASSEXW windowClass{};
        windowClass.cbSize = sizeof(windowClass);
        windowClass.style = CS_HREDRAW | CS_VREDRAW;
        windowClass.lpfnWndProc = &Implementation::windowProcedure;
        windowClass.hInstance = instance_;
        windowClass.hCursor = LoadCursorW(nullptr, MAKEINTRESOURCEW(32512));
        windowClass.hbrBackground = static_cast<HBRUSH>(GetStockObject(BLACK_BRUSH));
        windowClass.lpszClassName = className;
        if (RegisterClassExW(&windowClass) == 0)
        {
            error_ = "RegisterClassExW failed with Win32 error " + std::to_string(GetLastError()) + ".";
            return false;
        }
        classRegistered_ = true;

        const auto& monitor = selected->information;
        DWORD style = config.fullscreen || config.borderless
            ? WS_POPUP : WS_OVERLAPPEDWINDOW;
        RECT rectangle{0, 0, static_cast<LONG>(config.width), static_cast<LONG>(config.height)};
        if (!config.fullscreen && !AdjustWindowRectEx(&rectangle, style, FALSE, 0))
        {
            error_ = "AdjustWindowRectEx failed with Win32 error " + std::to_string(GetLastError()) + ".";
            return false;
        }
        const unsigned int requestedWidth = config.fullscreen
            ? config.width : static_cast<unsigned int>(rectangle.right - rectangle.left);
        const unsigned int requestedHeight = config.fullscreen
            ? config.height : static_cast<unsigned int>(rectangle.bottom - rectangle.top);
        const auto placement = calculateWindowPlacement(
            monitor, requestedWidth, requestedHeight, config.x, config.y, config.fullscreen);
        if (!placement.has_value())
        {
            error_ = "The selected monitor has invalid bounds or the window dimensions are invalid.";
            return false;
        }
        const std::wstring title = wide(config.title);
        window_ = CreateWindowExW(0, className, title.c_str(), style,
            placement->x, placement->y,
            static_cast<int>(placement->width), static_cast<int>(placement->height),
            nullptr, nullptr, instance_, this);
        if (window_ == nullptr)
        {
            error_ = "CreateWindowExW failed with Win32 error " + std::to_string(GetLastError()) + ".";
            return false;
        }
        const HMONITOR actualMonitor = MonitorFromWindow(window_, MONITOR_DEFAULTTONULL);
        MONITORINFOEXW actualInformation{};
        actualInformation.cbSize = sizeof(actualInformation);
        if (actualMonitor == nullptr || !GetMonitorInfoW(actualMonitor, &actualInformation)
            || !sameDeviceName(utf8(actualInformation.szDevice), monitor.deviceName))
        {
            error_ = "The renderer window did not remain on selected monitor "
                + monitor.deviceName + "; refusing to continue on another display.";
            DestroyWindow(window_);
            window_ = nullptr;
            return false;
        }
        RECT clientRectangle{};
        if (!GetClientRect(window_, &clientRectangle))
        {
            error_ = "GetClientRect failed with Win32 error " + std::to_string(GetLastError()) + ".";
            DestroyWindow(window_);
            window_ = nullptr;
            return false;
        }
        clientWidth_ = static_cast<unsigned int>(clientRectangle.right - clientRectangle.left);
        clientHeight_ = static_cast<unsigned int>(clientRectangle.bottom - clientRectangle.top);
        pendingResize_ = WindowResize{clientWidth_, clientHeight_};
        if (!config.hidden)
        {
            ShowWindow(window_, SW_SHOW);
            UpdateWindow(window_);
        }
        return true;
    }

    [[nodiscard]] bool processMessages() noexcept
    {
        MSG message{};
        while (PeekMessageW(&message, nullptr, 0, 0, PM_REMOVE))
        {
            if (message.message == WM_QUIT)
            {
                running_ = false;
            }
            TranslateMessage(&message);
            DispatchMessageW(&message);
        }
        return running_;
    }

    void waitWhenMinimized() const noexcept
    {
        if (minimized_)
        {
            (void)MsgWaitForMultipleObjects(0, nullptr, FALSE, 50, QS_ALLINPUT);
        }
    }

    static LRESULT CALLBACK windowProcedure(HWND window, UINT message, WPARAM wParam, LPARAM lParam)
    {
        Implementation* self = nullptr;
        if (message == WM_NCCREATE)
        {
            const auto* create = reinterpret_cast<CREATESTRUCTW*>(lParam);
            self = static_cast<Implementation*>(create->lpCreateParams);
            SetWindowLongPtrW(window, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(self));
            self->window_ = window;
        }
        else
        {
            self = reinterpret_cast<Implementation*>(GetWindowLongPtrW(window, GWLP_USERDATA));
        }
        if (self == nullptr)
        {
            return DefWindowProcW(window, message, wParam, lParam);
        }
        switch (message)
        {
        case WM_SIZE:
            self->minimized_ = wParam == SIZE_MINIMIZED;
            if (!self->minimized_)
            {
                self->clientWidth_ = static_cast<unsigned int>(LOWORD(lParam));
                self->clientHeight_ = static_cast<unsigned int>(HIWORD(lParam));
                if (self->clientWidth_ > 0U && self->clientHeight_ > 0U)
                {
                    self->pendingResize_ = WindowResize{self->clientWidth_, self->clientHeight_};
                }
            }
            return 0;
        case WM_KEYDOWN:
        case WM_SYSKEYDOWN:
            if ((lParam & (1LL << 30)) == 0 && self->keyHandler_)
            {
                self->keyHandler_(static_cast<unsigned int>(wParam));
            }
            break;
        case WM_CLOSE:
            DestroyWindow(window);
            return 0;
        case WM_DESTROY:
            self->window_ = nullptr;
            self->running_ = false;
            PostQuitMessage(0);
            return 0;
        default:
            break;
        }
        return DefWindowProcW(window, message, wParam, lParam);
    }

    HINSTANCE instance_{};
    HWND window_{};
    bool classRegistered_{};
    bool running_{true};
    bool minimized_{};
    unsigned int clientWidth_{};
    unsigned int clientHeight_{};
    std::optional<WindowResize> pendingResize_;
    KeyHandler keyHandler_;
    std::string error_;
    static constexpr wchar_t className[] = L"XrealSpatialRendererWindow";
};

RenderWindow::RenderWindow() : implementation_(std::make_unique<Implementation>()) {}
RenderWindow::~RenderWindow() = default;
RenderWindow::RenderWindow(RenderWindow&&) noexcept = default;
RenderWindow& RenderWindow::operator=(RenderWindow&&) noexcept = default;

bool RenderWindow::create(const RenderWindowConfig& config, KeyHandler keyHandler)
{
    return implementation_->create(config, std::move(keyHandler));
}

bool RenderWindow::processMessages() noexcept { return implementation_->processMessages(); }
void RenderWindow::waitForMessageWhenMinimized() const noexcept { implementation_->waitWhenMinimized(); }
bool RenderWindow::minimized() const noexcept { return implementation_->minimized_; }

std::optional<WindowResize> RenderWindow::takePendingResize() noexcept
{
    const auto result = implementation_->pendingResize_;
    implementation_->pendingResize_.reset();
    return result;
}

void* RenderWindow::nativeHandle() const noexcept { return implementation_->window_; }

void RenderWindow::setTitle(const std::string& title) noexcept
{
    if (implementation_->window_ != nullptr)
    {
        const std::wstring value = wide(title);
        (void)SetWindowTextW(implementation_->window_, value.c_str());
    }
}

const std::string& RenderWindow::error() const noexcept { return implementation_->error_; }

} // namespace xreal::platform::windows
