#pragma once

#include "capture/DesktopCaptureFrame.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace xreal::capture
{

struct PackedBgraLayout
{
    std::uint32_t stride{};
    std::size_t bufferSize{};
};

struct BmpWriteResult
{
    bool success{};
    std::string error;
};

struct BgraPixel
{
    std::uint8_t blue{};
    std::uint8_t green{};
    std::uint8_t red{};
    std::uint8_t alpha{};

    [[nodiscard]] bool operator==(const BgraPixel&) const noexcept = default;
};

struct DesktopFrameContentSummary
{
    std::uint64_t checksum{};
    std::size_t byteCount{};
    std::size_t distinctColorCount{};
    BgraPixel firstPixel;
    BgraPixel centerPixel;
    BgraPixel lastPixel;
    bool allZero{true};
    bool allOpaque{true};
};

[[nodiscard]] std::optional<PackedBgraLayout> calculatePackedBgraLayout(
    std::uint32_t width,
    std::uint32_t height) noexcept;
[[nodiscard]] bool repackBgraRows(
    const std::byte* source,
    std::size_t sourceSize,
    std::uint32_t sourceRowPitch,
    std::span<std::byte> destination,
    std::uint32_t width,
    std::uint32_t height) noexcept;
[[nodiscard]] std::optional<DesktopCaptureFrame> makeDesktopCheckerboardFrame(
    std::uint32_t width,
    std::uint32_t height);
[[nodiscard]] std::uint64_t stableBgraChecksum(std::span<const std::byte> bytes) noexcept;
[[nodiscard]] std::optional<DesktopFrameContentSummary> analyzeDesktopFrameContent(
    const DesktopCaptureFrame& frame);
[[nodiscard]] BmpWriteResult writeDesktopFrameBmp(
    const DesktopCaptureFrame& frame,
    const std::string& path);

} // namespace xreal::capture
