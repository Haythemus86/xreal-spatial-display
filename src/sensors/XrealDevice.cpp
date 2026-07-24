#include "sensors/XrealDevice.hpp"

#include <hidapi.h>

#include <memory>
#include <stdexcept>

namespace xreal::sensors
{
namespace
{

std::wstring copyString(const wchar_t* value)
{
    return value != nullptr ? value : L"";
}

std::string copyString(const char* value)
{
    return value != nullptr ? value : "";
}

struct DeviceListDeleter
{
    void operator()(hid_device_info* devices) const noexcept
    {
        hid_free_enumeration(devices);
    }
};

using DeviceList = std::unique_ptr<hid_device_info, DeviceListDeleter>;

} // namespace

XrealDevice::XrealDevice()
{
    if (hid_init() != 0)
    {
        throw std::runtime_error("Failed to initialize HIDAPI.");
    }
}

XrealDevice::~XrealDevice()
{
    hid_exit();
}

std::vector<XrealDeviceInfo> XrealDevice::enumerate() const
{
    DeviceList devices{hid_enumerate(xrealVendorId, 0)};
    std::vector<XrealDeviceInfo> result;

    for (const hid_device_info* device = devices.get(); device != nullptr; device = device->next)
    {
        result.push_back(XrealDeviceInfo{
            device->vendor_id,
            device->product_id,
            device->interface_number,
            copyString(device->manufacturer_string),
            copyString(device->product_string),
            copyString(device->serial_number),
            copyString(device->path),
        });
    }

    return result;
}

} // namespace xreal::sensors
