#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace xreal::sensors
{

struct XrealDeviceInfo
{
    std::uint16_t vendorId{};
    std::uint16_t productId{};
    int interfaceNumber{};
    std::wstring manufacturerName;
    std::wstring productName;
    std::wstring serialNumber;
    std::string path;
};

class XrealDevice
{
public:
    XrealDevice();
    ~XrealDevice();

    XrealDevice(const XrealDevice&) = delete;
    XrealDevice& operator=(const XrealDevice&) = delete;
    XrealDevice(XrealDevice&&) = delete;
    XrealDevice& operator=(XrealDevice&&) = delete;

    [[nodiscard]] std::vector<XrealDeviceInfo> enumerate() const;

private:
    static constexpr std::uint16_t xrealVendorId = 0x3318;
};

} // namespace xreal::sensors
