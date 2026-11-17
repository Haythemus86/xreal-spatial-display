#include "CameraIdentity.hpp"
#include <windows.h>
#include <dshow.h>
#include <wrl/client.h>
#include <algorithm>
#include <cwctype>
#include <stdexcept>
#include <string>

namespace xreal::stereo
{
using Microsoft::WRL::ComPtr;

static std::wstring property(IPropertyBag& bag, const wchar_t* key)
{
    VARIANT v;
    VariantInit(&v);
    std::wstring value;
    if (SUCCEEDED(bag.Read(key, &v, nullptr)) && v.vt == VT_BSTR && v.bstrVal)
        value = v.bstrVal;
    VariantClear(&v);
    return value;
}

void requireUltraCamera()
{
    if (FAILED(CoInitializeEx(nullptr, COINIT_MULTITHREADED)))
        throw std::runtime_error("Could not initialize DirectShow enumeration");
    struct Apartment { ~Apartment() { CoUninitialize(); } } apartment;
    ComPtr<ICreateDevEnum> devices;
    ComPtr<IEnumMoniker> enumerator;
    if (FAILED(CoCreateInstance(CLSID_SystemDeviceEnum, nullptr, CLSCTX_INPROC_SERVER,
            IID_PPV_ARGS(devices.GetAddressOf())))
        || devices->CreateClassEnumerator(CLSID_VideoInputDeviceCategory, &enumerator, 0) != S_OK)
        throw std::runtime_error("No video capture devices available");
    ComPtr<IMoniker> moniker;
    unsigned matchingNames{};
    bool ultra{};
    while (enumerator->Next(1, moniker.ReleaseAndGetAddressOf(), nullptr) == S_OK)
    {
        ComPtr<IPropertyBag> bag;
        if (FAILED(moniker->BindToStorage(nullptr, nullptr, IID_PPV_ARGS(bag.GetAddressOf()))))
            continue;
        if (property(*bag.Get(), L"FriendlyName").find(L"UVC Camera 0") == std::wstring::npos)
            continue;
        ++matchingNames;
        auto path = property(*bag.Get(), L"DevicePath");
        std::transform(path.begin(), path.end(), path.begin(), [](wchar_t c) {
            return static_cast<wchar_t>(std::towlower(c));
        });
        ultra = path.find(L"vid_3318&pid_0426") != std::wstring::npos;
    }
    if (matchingNames != 1 || !ultra)
        throw std::runtime_error("Expected one UVC Camera 0 belonging to USB 3318:0426");
}
}
