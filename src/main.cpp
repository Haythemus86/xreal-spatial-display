#include "sensors/XrealDevice.hpp"

#include <iomanip>
#include <iostream>
#include <stdexcept>

int main()
{
    try
    {
        const xreal::sensors::XrealDevice xrealDevice;
        const auto devices = xrealDevice.enumerate();

        if (devices.empty())
        {
            std::cout << "No XREAL HID interfaces found.\n";
            return 0;
        }

        std::cout << devices.size() << " XREAL HID interface(s) found.\n";

        for (std::size_t index = 0; index < devices.size(); ++index)
        {
            const auto& device = devices[index];

            std::cout << "\nInterface " << index + 1 << '\n'
                      << "  Vendor ID:       0x" << std::hex << std::uppercase
                      << std::setw(4) << std::setfill('0') << device.vendorId << '\n'
                      << "  Product ID:      0x" << std::setw(4) << device.productId << '\n'
                      << std::dec << std::setfill(' ')
                      << "  Interface number: " << device.interfaceNumber << '\n';
            std::wcout << L"  Manufacturer:    " << device.manufacturerName << L'\n'
                       << L"  Product:         " << device.productName << L'\n'
                       << L"  Serial number:   " << device.serialNumber << L'\n';
            std::cout << "  Device path:     " << device.path << '\n';
        }

        return 0;
    }
    catch (const std::exception& exception)
    {
        std::cerr << "Error: " << exception.what() << '\n';
        return 1;
    }
}
