#include "capture/DesktopCpuFrame.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <fstream>
#include <limits>
#include <memory>
#include <unordered_set>
#include <vector>

namespace xreal::capture
{

std::optional<PackedBgraLayout> calculatePackedBgraLayout(
    std::uint32_t width,
    std::uint32_t height) noexcept
{
    if (width == 0U || height == 0U
        || width > std::numeric_limits<std::uint32_t>::max() / 4U)
    {
        return std::nullopt;
    }
    const std::uint32_t stride = width * 4U;
    if (static_cast<std::size_t>(height)
        > std::numeric_limits<std::size_t>::max() / stride)
    {
        return std::nullopt;
    }
    return PackedBgraLayout{stride, static_cast<std::size_t>(stride) * height};
}

bool repackBgraRows(
    const std::byte* source,
    std::size_t sourceSize,
    std::uint32_t sourceRowPitch,
    std::span<std::byte> destination,
    std::uint32_t width,
    std::uint32_t height) noexcept
{
    const auto layout = calculatePackedBgraLayout(width, height);
    if (!layout.has_value() || source == nullptr
        || sourceRowPitch < layout->stride
        || static_cast<std::size_t>(height)
            > std::numeric_limits<std::size_t>::max() / sourceRowPitch
        || static_cast<std::size_t>(sourceRowPitch) * height > sourceSize
        || destination.size() < layout->bufferSize)
    {
        return false;
    }
    for (std::uint32_t row = 0U; row < height; ++row)
    {
        std::memcpy(destination.data() + static_cast<std::size_t>(row) * layout->stride,
            source + static_cast<std::size_t>(row) * sourceRowPitch, layout->stride);
    }
    return true;
}

std::optional<DesktopCaptureFrame> makeDesktopCheckerboardFrame(
    std::uint32_t width,
    std::uint32_t height)
{
    const auto layout = calculatePackedBgraLayout(width, height);
    if (!layout.has_value())
    {
        return std::nullopt;
    }
    auto pixels = std::make_shared<std::vector<std::byte>>(layout->bufferSize);
    constexpr std::uint32_t tileSize = 40U;
    constexpr std::uint32_t markerSize = 32U;
    for (std::uint32_t y = 0U; y < height; ++y)
    {
        for (std::uint32_t x = 0U; x < width; ++x)
        {
            const bool magenta = ((x / tileSize) + (y / tileSize)) % 2U == 0U;
            const std::size_t offset = static_cast<std::size_t>(y) * layout->stride
                + static_cast<std::size_t>(x) * 4U;
            BgraPixel color = magenta
                ? BgraPixel{0xFFU, 0x00U, 0xFFU, 0xFFU}
                : BgraPixel{0x00U, 0xFFU, 0x00U, 0xFFU};
            if (x < markerSize && y < markerSize)
            {
                color = {0x00U, 0x00U, 0xFFU, 0xFFU};
            }
            else if (x >= width - std::min(width, markerSize) && y < markerSize)
            {
                color = {0xFFU, 0x00U, 0x00U, 0xFFU};
            }
            const std::uint32_t centerX = width / 2U;
            const std::uint32_t centerY = height / 2U;
            if (x >= centerX - std::min(centerX, markerSize / 2U)
                && x < centerX + std::min(width - centerX, markerSize / 2U)
                && y >= centerY - std::min(centerY, markerSize / 2U)
                && y < centerY + std::min(height - centerY, markerSize / 2U))
            {
                color = {0xFFU, 0xFFU, 0xFFU, 0xFFU};
            }
            (*pixels)[offset + 0U] = static_cast<std::byte>(color.blue);
            (*pixels)[offset + 1U] = static_cast<std::byte>(color.green);
            (*pixels)[offset + 2U] = static_cast<std::byte>(color.red);
            (*pixels)[offset + 3U] = std::byte{0xFFU};
        }
    }
    DesktopCaptureFrame frame;
    frame.sequence = 1U;
    frame.captureHostTimestamp = std::chrono::steady_clock::now();
    frame.sourceWidth = width;
    frame.sourceHeight = height;
    frame.sourceFormat = 87U; // DXGI_FORMAT_B8G8R8A8_UNORM, kept portable here.
    frame.cpuPixels = std::move(pixels);
    frame.cpuRowPitch = layout->stride;
    frame.valid = true;
    frame.status = DesktopCaptureStatus::active;
    frame.sourceMonitorDeviceName = "desktop-debug-checkerboard";
    return frame;
}

std::uint64_t stableBgraChecksum(std::span<const std::byte> bytes) noexcept
{
    constexpr std::uint64_t offsetBasis = 14695981039346656037ULL;
    constexpr std::uint64_t prime = 1099511628211ULL;
    std::uint64_t result = offsetBasis;
    for (const std::byte value : bytes)
    {
        result ^= static_cast<std::uint8_t>(value);
        result *= prime;
    }
    return result;
}

std::optional<DesktopFrameContentSummary> analyzeDesktopFrameContent(
    const DesktopCaptureFrame& frame)
{
    const auto layout = calculatePackedBgraLayout(frame.sourceWidth, frame.sourceHeight);
    if (!frame.valid || !frame.cpuPixels || !layout.has_value()
        || frame.cpuRowPitch < layout->stride
        || frame.cpuPixels->size() < static_cast<std::size_t>(frame.cpuRowPitch)
            * frame.sourceHeight)
    {
        return std::nullopt;
    }
    std::vector<std::byte> packed(layout->bufferSize);
    if (!repackBgraRows(frame.cpuPixels->data(), frame.cpuPixels->size(),
            frame.cpuRowPitch, packed, frame.sourceWidth, frame.sourceHeight))
    {
        return std::nullopt;
    }
    const auto pixelAt = [&packed](std::size_t pixelIndex) {
        const std::size_t offset = pixelIndex * 4U;
        return BgraPixel{
            static_cast<std::uint8_t>(packed[offset + 0U]),
            static_cast<std::uint8_t>(packed[offset + 1U]),
            static_cast<std::uint8_t>(packed[offset + 2U]),
            static_cast<std::uint8_t>(packed[offset + 3U]),
        };
    };
    DesktopFrameContentSummary result;
    result.byteCount = packed.size();
    result.checksum = stableBgraChecksum(packed);
    const std::size_t pixelCount = packed.size() / 4U;
    result.firstPixel = pixelAt(0U);
    result.centerPixel = pixelAt(static_cast<std::size_t>(frame.sourceHeight / 2U)
        * frame.sourceWidth + frame.sourceWidth / 2U);
    result.lastPixel = pixelAt(pixelCount - 1U);
    std::unordered_set<std::uint32_t> colors;
    for (std::size_t index = 0U; index < pixelCount; ++index)
    {
        const auto pixel = pixelAt(index);
        result.allZero = result.allZero
            && pixel.blue == 0U && pixel.green == 0U && pixel.red == 0U && pixel.alpha == 0U;
        result.allOpaque = result.allOpaque && pixel.alpha == 0xFFU;
        colors.insert(static_cast<std::uint32_t>(pixel.blue)
            | (static_cast<std::uint32_t>(pixel.green) << 8U)
            | (static_cast<std::uint32_t>(pixel.red) << 16U)
            | (static_cast<std::uint32_t>(pixel.alpha) << 24U));
    }
    result.distinctColorCount = colors.size();
    return result;
}

BmpWriteResult writeDesktopFrameBmp(const DesktopCaptureFrame& frame, const std::string& path)
{
    const auto layout = calculatePackedBgraLayout(frame.sourceWidth, frame.sourceHeight);
    if (!frame.valid || !frame.cpuPixels || !layout.has_value()
        || frame.cpuRowPitch < layout->stride
        || frame.cpuPixels->size() < static_cast<std::size_t>(frame.cpuRowPitch) * frame.sourceHeight)
    {
        return {false, "Desktop frame is not a valid BGRA CPU frame."};
    }
    if (layout->bufferSize > std::numeric_limits<std::uint32_t>::max() - 54U)
    {
        return {false, "Desktop frame is too large for the BMP file format."};
    }
    if (frame.sourceWidth > static_cast<std::uint32_t>(std::numeric_limits<std::int32_t>::max())
        || frame.sourceHeight > static_cast<std::uint32_t>(std::numeric_limits<std::int32_t>::max()))
    {
        return {false, "Desktop frame dimensions exceed signed BMP limits."};
    }
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    if (!output)
    {
        return {false, "Failed to open BMP output: " + path};
    }
    const auto write16 = [&output](std::uint16_t value) {
        const std::array bytes{static_cast<char>(value), static_cast<char>(value >> 8U)};
        output.write(bytes.data(), bytes.size());
    };
    const auto write32 = [&output](std::uint32_t value) {
        const std::array bytes{static_cast<char>(value), static_cast<char>(value >> 8U),
            static_cast<char>(value >> 16U), static_cast<char>(value >> 24U)};
        output.write(bytes.data(), bytes.size());
    };
    output.write("BM", 2);
    write32(static_cast<std::uint32_t>(54U + layout->bufferSize));
    write32(0U);
    write32(54U);
    write32(40U);
    write32(frame.sourceWidth);
    write32(static_cast<std::uint32_t>(-static_cast<std::int32_t>(frame.sourceHeight)));
    write16(1U);
    write16(32U);
    write32(0U);
    write32(static_cast<std::uint32_t>(layout->bufferSize));
    write32(0U); write32(0U); write32(0U); write32(0U);
    for (std::uint32_t row = 0U; row < frame.sourceHeight; ++row)
    {
        output.write(reinterpret_cast<const char*>(frame.cpuPixels->data()
            + static_cast<std::size_t>(row) * frame.cpuRowPitch), layout->stride);
    }
    return output ? BmpWriteResult{true, {}}
                  : BmpWriteResult{false, "Failed while writing BMP output: " + path};
}

} // namespace xreal::capture
