#include "Driver.hpp"

#include "protocol/VirtualDisplayProtocol.hpp"
#include "protocol/VirtualDisplayTypes.hpp"
#include "service/VirtualDisplayService.hpp"

#include <d3d11.h>
#include <dxgi1_6.h>
#include <wrl/client.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <cwchar>
#include <exception>
#include <memory>
#include <mutex>
#include <new>
#include <span>
#include <stop_token>
#include <thread>
#include <utility>

namespace
{

using Microsoft::WRL::ComPtr;
using namespace xreal::virtual_display;

constexpr wchar_t controlDeviceName[] = L"\\DosDevices\\XrealVirtualDisplay";
void trace(const wchar_t* message) noexcept
{
    OutputDebugStringW(L"[xreal-virtual-display] ");
    OutputDebugStringW(message);
    OutputDebugStringW(L"\n");
}

void traceConfiguration(const VirtualDisplayStatus& status) noexcept
{
    wchar_t message[192]{};
    const wchar_t* mode = status.requested.mode == VirtualDisplayMode::extended
        ? L"extended" : L"mirrored";
    (void)swprintf_s(
        message,
        L"configuration generation=%llu requested_count=%zu actual_count=%zu "
        L"mode=%s restart_required=%s",
        static_cast<unsigned long long>(status.actual.applyGeneration),
        status.requested.enabled
            ? virtualDisplayCountValue(status.requested.count) : 0U,
        status.monitorCount,
        mode,
        status.restartRequired ? L"yes" : L"no");
    trace(message);
}

[[nodiscard]] DISPLAYCONFIG_VIDEO_SIGNAL_INFO makeSignalInfo() noexcept
{
    DISPLAYCONFIG_VIDEO_SIGNAL_INFO signal{};
    signal.pixelRate = 148'500'000ULL;
    signal.hSyncFreq = {67'500U, 1U};
    signal.vSyncFreq = {60'000U, 1'000U};
    signal.activeSize = {defaultVirtualDisplayWidth, defaultVirtualDisplayHeight};
    signal.totalSize = {2200U, 1125U};
    signal.videoStandard = D3DKMDT_VSS_OTHER;
    signal.scanLineOrdering = DISPLAYCONFIG_SCANLINE_ORDERING_PROGRESSIVE;
    return signal;
}

[[nodiscard]] IDDCX_TARGET_MODE makeTargetMode() noexcept
{
    IDDCX_TARGET_MODE result{};
    result.Size = sizeof(result);
    result.TargetVideoSignalInfo = makeSignalInfo();
    return result;
}

[[nodiscard]] consteval std::array<std::uint8_t, 128U> makeMonitorEdid(
    std::uint8_t logicalIndex)
{
    std::array<std::uint8_t, 128U> result{};
    constexpr std::array<std::uint8_t, 18U> timing{
        0x02U, 0x3AU, 0x80U, 0x18U, 0x71U, 0x38U, 0x2DU, 0x40U, 0x58U,
        0x2CU, 0x45U, 0x00U, 0xFDU, 0x1EU, 0x11U, 0x00U, 0x00U, 0x1AU,
    };
    constexpr std::array<char, 12U> namePrefix{
        'X', 'R', 'E', 'A', 'L', ' ', 'V', 'D', 'I', 'S', 'P', ' ',
    };
    result[0] = 0x00U;
    result[1] = 0xFFU;
    result[2] = 0xFFU;
    result[3] = 0xFFU;
    result[4] = 0xFFU;
    result[5] = 0xFFU;
    result[6] = 0xFFU;
    result[7] = 0x00U;
    result[8] = 0x62U; // EISA manufacturer code XRL, big-endian.
    result[9] = 0x4CU;
    result[10] = static_cast<std::uint8_t>(logicalIndex + 1U);
    result[12] = static_cast<std::uint8_t>(logicalIndex + 1U);
    result[16] = 1U;
    result[17] = 36U; // 2026 relative to the EDID epoch.
    result[18] = 1U;
    result[19] = 4U;
    result[20] = 0x80U; // Digital input, SDR only.
    result[21] = 52U;
    result[22] = 29U;
    result[23] = 120U;
    result[24] = 0x0AU;
    for (std::size_t index = 38U; index < 54U; index += 2U)
    {
        result[index] = 0x01U;
        result[index + 1U] = 0x01U;
    }
    for (std::size_t index = 0U; index < timing.size(); ++index)
    {
        result[54U + index] = timing[index];
    }
    result[75] = 0xFCU; // Display-product-name descriptor.
    for (std::size_t index = 0U; index < namePrefix.size(); ++index)
    {
        result[77U + index] = static_cast<std::uint8_t>(namePrefix[index]);
    }
    result[89] = static_cast<std::uint8_t>('1' + logicalIndex);
    result[93] = 0xFFU; // Display-product-serial-number descriptor.
    constexpr std::array<char, 12U> serialPrefix{
        'X', 'R', 'E', 'A', 'L', '-', 'V', 'D', 'I', 'S', 'P', '-',
    };
    for (std::size_t index = 0U; index < serialPrefix.size(); ++index)
    {
        result[95U + index] = static_cast<std::uint8_t>(serialPrefix[index]);
    }
    result[107] = static_cast<std::uint8_t>('1' + logicalIndex);
    result[111] = 0xFD; // Range-limits descriptor.
    result[113] = 50U;
    result[114] = 60U;
    result[115] = 30U;
    result[116] = 80U;
    result[117] = 15U;
    result[126] = 0U;
    std::uint8_t checksum{};
    for (std::size_t index = 0U; index < 127U; ++index)
    {
        checksum = static_cast<std::uint8_t>(checksum + result[index]);
    }
    result[127] = static_cast<std::uint8_t>(0U - checksum);
    return result;
}

inline constexpr std::array monitorEdids{
    makeMonitorEdid(0U), makeMonitorEdid(1U), makeMonitorEdid(2U),
};

class SwapChainProcessor
{
public:
    SwapChainProcessor(IDDCX_SWAPCHAIN swapChain, LUID renderAdapter, HANDLE frameEvent)
        : swapChain_(swapChain), renderAdapter_(renderAdapter), frameEvent_(frameEvent),
          thread_([this](std::stop_token stopToken) { run(stopToken); })
    {
    }

    ~SwapChainProcessor()
    {
        thread_.request_stop();
        SetEvent(frameEvent_);
    }

    SwapChainProcessor(const SwapChainProcessor&) = delete;
    SwapChainProcessor& operator=(const SwapChainProcessor&) = delete;

private:
    void run(std::stop_token stopToken) noexcept
    {
        ComPtr<IDXGIFactory6> factory;
        ComPtr<IDXGIAdapter1> adapter;
        ComPtr<ID3D11Device> device;
        ComPtr<ID3D11DeviceContext> context;
        if (FAILED(CreateDXGIFactory2(0U, IID_PPV_ARGS(&factory)))
            || FAILED(factory->EnumAdapterByLuid(renderAdapter_, IID_PPV_ARGS(&adapter)))
            || FAILED(D3D11CreateDevice(
                adapter.Get(), D3D_DRIVER_TYPE_UNKNOWN, nullptr,
                D3D11_CREATE_DEVICE_BGRA_SUPPORT, nullptr, 0U, D3D11_SDK_VERSION,
                &device, nullptr, &context)))
        {
            trace(L"frame pipeline failed to create its D3D11 device");
            return;
        }

        IDARG_IN_SWAPCHAINSETDEVICE setDevice{};
        setDevice.pDevice = device.Get();
        if (FAILED(IddCxSwapChainSetDevice(swapChain_, &setDevice)))
        {
            trace(L"IddCxSwapChainSetDevice failed");
            return;
        }

        trace(L"frame pipeline started");
        while (!stopToken.stop_requested())
        {
            IDARG_OUT_RELEASEANDACQUIREBUFFER acquired{};
            const HRESULT result = IddCxSwapChainReleaseAndAcquireBuffer(
                swapChain_, &acquired);
            if (result == E_PENDING)
            {
                WaitForSingleObject(frameEvent_, 100U);
                continue;
            }
            if (FAILED(result))
            {
                trace(L"frame acquisition stopped after an IddCx error");
                break;
            }

            // The compositor surface remains GPU-owned. This prototype deliberately
            // performs no CPU readback, allocation, copy, or Flush in this loop.
            // A future direct IPanelFrameSource transport can consume the surface.
            acquired.MetaData.pSurface->Release();
            if (FAILED(IddCxSwapChainFinishedProcessingFrame(swapChain_)))
            {
                trace(L"IddCxSwapChainFinishedProcessingFrame failed");
                break;
            }
        }
        trace(L"frame pipeline stopped");
    }

    IDDCX_SWAPCHAIN swapChain_{};
    LUID renderAdapter_{};
    HANDLE frameEvent_{};
    std::jthread thread_;
};

struct MonitorContext
{
    std::mutex mutex;
    std::unique_ptr<SwapChainProcessor> processor;
};

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(MonitorContext, monitorContext);

struct DeviceContext
{
    IDDCX_ADAPTER adapter{};
    std::mutex mutex;
    VirtualDisplayStateMachine stateMachine;
    std::array<IDDCX_MONITOR, maximumVirtualDisplayCount> monitors{};
};

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(DeviceContext, deviceContext);

[[nodiscard]] GUID monitorContainerId(std::size_t index) noexcept
{
    GUID result{0x7d5529c1U, 0x9b47U, 0x4fc4U,
        {0xa2U, 0x01U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x01U}};
    result.Data4[7] = static_cast<unsigned char>(index + 1U);
    return result;
}

[[nodiscard]] NTSTATUS createMonitor(DeviceContext& context, std::size_t index)
{
    if (index >= context.monitors.size() || context.monitors[index] != nullptr)
    {
        return index < context.monitors.size() ? STATUS_SUCCESS : STATUS_INVALID_PARAMETER;
    }

    IDDCX_MONITOR_INFO monitorInfo{};
    monitorInfo.Size = sizeof(monitorInfo);
    monitorInfo.MonitorType = DISPLAYCONFIG_OUTPUT_TECHNOLOGY_OTHER;
    monitorInfo.ConnectorIndex = static_cast<UINT>(index);
    monitorInfo.MonitorDescription.Type = IDDCX_MONITOR_DESCRIPTION_TYPE_EDID;
    monitorInfo.MonitorDescription.DataSize = static_cast<UINT>(monitorEdids[index].size());
    monitorInfo.MonitorDescription.pData = const_cast<BYTE*>(monitorEdids[index].data());
    monitorInfo.MonitorContainerId = monitorContainerId(index);

    WDF_OBJECT_ATTRIBUTES attributes;
    WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&attributes, MonitorContext);
    attributes.EvtCleanupCallback = XrealVirtualDisplayMonitorCleanup;
    IDARG_IN_MONITORCREATE create{};
    create.ObjectAttributes = &attributes;
    create.pMonitorInfo = &monitorInfo;
    IDARG_OUT_MONITORCREATE created{};
    NTSTATUS status = IddCxMonitorCreate(context.adapter, &create, &created);
    if (!NT_SUCCESS(status))
    {
        trace(L"monitor creation failed");
        return status;
    }
    new (monitorContext(created.MonitorObject)) MonitorContext{};
    status = IddCxMonitorArrival(created.MonitorObject);
    if (!NT_SUCCESS(status))
    {
        WdfObjectDelete(created.MonitorObject);
        trace(L"monitor arrival failed");
        return status;
    }
    context.monitors[index] = created.MonitorObject;
    trace(L"monitor created");
    return STATUS_SUCCESS;
}

void removeMonitor(DeviceContext& context, std::size_t index) noexcept
{
    if (index >= context.monitors.size() || context.monitors[index] == nullptr)
    {
        return;
    }
    const auto monitor = context.monitors[index];
    context.monitors[index] = nullptr;
    IddCxMonitorDeparture(monitor);
    trace(L"monitor removed");
}

[[nodiscard]] NTSTATUS applyTopology(
    DeviceContext& context,
    const VirtualDisplayStatus& desired) noexcept
{
    const std::size_t desiredCount = desired.actual.enabled
        ? virtualDisplayCountValue(desired.actual.count) : 0U;
    for (std::size_t index = desiredCount; index < context.monitors.size(); ++index)
    {
        removeMonitor(context, index);
    }
    for (std::size_t index = 0U; index < desiredCount; ++index)
    {
        const NTSTATUS status = createMonitor(context, index);
        if (!NT_SUCCESS(status))
        {
            return status;
        }
    }
    return STATUS_SUCCESS;
}

} // namespace

extern "C" NTSTATUS DriverEntry(PDRIVER_OBJECT driverObject, PUNICODE_STRING registryPath)
{
    WDF_DRIVER_CONFIG config;
    WDF_DRIVER_CONFIG_INIT(&config, XrealVirtualDisplayDeviceAdd);
    trace(L"driver loaded");
    return WdfDriverCreate(
        driverObject, registryPath, WDF_NO_OBJECT_ATTRIBUTES, &config, WDF_NO_HANDLE);
}

NTSTATUS XrealVirtualDisplayDeviceAdd(WDFDRIVER, PWDFDEVICE_INIT deviceInit)
{
    IDDCX_CLIENT_CONFIG iddConfig{};
    IDDCX_CLIENT_CONFIG_INIT(&iddConfig);
    iddConfig.EvtIddCxAdapterInitFinished = XrealVirtualDisplayAdapterInitFinished;
    iddConfig.EvtIddCxParseMonitorDescription = XrealVirtualDisplayParseMonitorDescription;
    iddConfig.EvtIddCxMonitorGetDefaultDescriptionModes =
        XrealVirtualDisplayGetDefaultDescriptionModes;
    iddConfig.EvtIddCxMonitorQueryTargetModes = XrealVirtualDisplayQueryTargetModes;
    iddConfig.EvtIddCxAdapterCommitModes = XrealVirtualDisplayCommitModes;
    iddConfig.EvtIddCxMonitorAssignSwapChain = XrealVirtualDisplayAssignSwapChain;
    iddConfig.EvtIddCxMonitorUnassignSwapChain = XrealVirtualDisplayUnassignSwapChain;
    NTSTATUS status = IddCxDeviceInitConfig(deviceInit, &iddConfig);
    if (!NT_SUCCESS(status))
    {
        return status;
    }

    WDF_PNPPOWER_EVENT_CALLBACKS powerCallbacks;
    WDF_PNPPOWER_EVENT_CALLBACKS_INIT(&powerCallbacks);
    powerCallbacks.EvtDeviceD0Entry = XrealVirtualDisplayDeviceD0Entry;
    WdfDeviceInitSetPnpPowerEventCallbacks(deviceInit, &powerCallbacks);

    WDF_OBJECT_ATTRIBUTES attributes;
    WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&attributes, DeviceContext);
    attributes.EvtCleanupCallback = XrealVirtualDisplayDeviceCleanup;
    WDFDEVICE device{};
    status = WdfDeviceCreate(&deviceInit, &attributes, &device);
    if (!NT_SUCCESS(status))
    {
        return status;
    }
    new (deviceContext(device)) DeviceContext{};

    UNICODE_STRING symbolicLink;
    RtlInitUnicodeString(&symbolicLink, controlDeviceName);
    status = WdfDeviceCreateSymbolicLink(device, &symbolicLink);
    if (!NT_SUCCESS(status))
    {
        return status;
    }

    WDF_IO_QUEUE_CONFIG queueConfig;
    WDF_IO_QUEUE_CONFIG_INIT_DEFAULT_QUEUE(&queueConfig, WdfIoQueueDispatchSequential);
    queueConfig.EvtIoDeviceControl = XrealVirtualDisplayDeviceControl;
    return WdfIoQueueCreate(device, &queueConfig, WDF_NO_OBJECT_ATTRIBUTES, nullptr);
}

void XrealVirtualDisplayDeviceCleanup(WDFOBJECT object)
{
    deviceContext(reinterpret_cast<WDFDEVICE>(object))->~DeviceContext();
}

void XrealVirtualDisplayMonitorCleanup(WDFOBJECT object)
{
    monitorContext(reinterpret_cast<IDDCX_MONITOR>(object))->~MonitorContext();
}

NTSTATUS XrealVirtualDisplayDeviceD0Entry(WDFDEVICE device, WDF_POWER_DEVICE_STATE)
{
    IDDCX_ADAPTER_CAPS capabilities{};
    capabilities.Size = sizeof(capabilities);
    capabilities.MaxMonitorsSupported = static_cast<UINT>(maximumVirtualDisplayCount);
    capabilities.EndPointDiagnostics.Size = sizeof(capabilities.EndPointDiagnostics);
    capabilities.EndPointDiagnostics.GammaSupport = IDDCX_FEATURE_IMPLEMENTATION_NONE;
    capabilities.EndPointDiagnostics.TransmissionType = IDDCX_TRANSMISSION_TYPE_WIRED_OTHER;

    IDARG_IN_ADAPTER_INIT init{};
    init.WdfDevice = device;
    init.pCaps = &capabilities;
    init.ObjectAttributes = WDF_NO_OBJECT_ATTRIBUTES;
    trace(L"adapter creation requested");
    return IddCxAdapterInitAsync(&init);
}

void XrealVirtualDisplayAdapterInitFinished(
    IDDCX_ADAPTER adapter,
    const IDARG_IN_ADAPTER_INIT_FINISHED* finished)
{
    if (!NT_SUCCESS(finished->AdapterInitStatus))
    {
        trace(L"adapter creation failed");
        return;
    }
    auto* context = deviceContext(finished->WdfDevice);
    std::scoped_lock lock(context->mutex);
    context->adapter = adapter;
    trace(L"adapter created; topology remains disabled until explicit Apply");
}

void XrealVirtualDisplayDeviceControl(
    WDFQUEUE queue,
    WDFREQUEST request,
    size_t outputBufferLength,
    size_t inputBufferLength,
    ULONG ioControlCode)
{
    try
    {
    if (ioControlCode != virtualDisplayIoControlCode
        || inputBufferLength != sizeof(VirtualDisplayProtocolRequest)
        || outputBufferLength < sizeof(VirtualDisplayProtocolResponse))
    {
        WdfRequestComplete(request, STATUS_INVALID_BUFFER_SIZE);
        return;
    }
    void* input{};
    void* output{};
    size_t inputSize{};
    size_t outputSize{};
    NTSTATUS status = WdfRequestRetrieveInputBuffer(
        request, sizeof(VirtualDisplayProtocolRequest), &input, &inputSize);
    if (NT_SUCCESS(status))
    {
        status = WdfRequestRetrieveOutputBuffer(
            request, sizeof(VirtualDisplayProtocolResponse), &output, &outputSize);
    }
    if (!NT_SUCCESS(status))
    {
        WdfRequestComplete(request, status);
        return;
    }

    auto requestBytes = std::span(
        static_cast<const std::byte*>(input), inputSize);
    const auto validation = validateProtocolRequest(requestBytes);
    if (!validation.valid)
    {
        WdfRequestComplete(request, STATUS_INVALID_PARAMETER);
        return;
    }
    VirtualDisplayProtocolRequest wireRequest{};
    memcpy(&wireRequest, input, sizeof(wireRequest));
    auto* context = deviceContext(WdfIoQueueGetDevice(queue));
    std::scoped_lock lock(context->mutex);
    auto response = context->stateMachine.process(wireRequest);
    if (response.accepted != 0U
        && (wireRequest.command == static_cast<std::uint16_t>(
                VirtualDisplayCommand::applyConfiguration)
            || wireRequest.command == static_cast<std::uint16_t>(
                VirtualDisplayCommand::disableVirtualDisplays)))
    {
        status = applyTopology(*context, context->stateMachine.status());
        if (!NT_SUCCESS(status))
        {
            trace(L"configuration accepted but topology application failed");
            WdfRequestComplete(request, status);
            return;
        }
        traceConfiguration(context->stateMachine.status());
    }
    memcpy(output, &response, sizeof(response));
    WdfRequestCompleteWithInformation(request, STATUS_SUCCESS, sizeof(response));
    }
    catch (const std::exception&)
    {
        trace(L"configuration processing failed with a C++ exception");
        WdfRequestComplete(request, STATUS_INSUFFICIENT_RESOURCES);
    }
    catch (...)
    {
        trace(L"configuration processing failed with an unknown exception");
        WdfRequestComplete(request, STATUS_UNSUCCESSFUL);
    }
}

NTSTATUS XrealVirtualDisplayParseMonitorDescription(
    const IDARG_IN_PARSEMONITORDESCRIPTION*,
    IDARG_OUT_PARSEMONITORDESCRIPTION* output)
{
    output->MonitorModeBufferOutputCount = 0U;
    output->PreferredMonitorModeIdx = 0U;
    return STATUS_NOT_SUPPORTED;
}

NTSTATUS XrealVirtualDisplayGetDefaultDescriptionModes(
    const IDARG_IN_GETDEFAULTDESCRIPTIONMODES* input,
    IDARG_OUT_GETDEFAULTDESCRIPTIONMODES* output)
{
    output->DefaultMonitorModeBufferOutputCount = 1U;
    output->PreferredMonitorModeIdx = 0U;
    if (input->DefaultMonitorModeBufferInputCount == 0U)
    {
        return STATUS_BUFFER_TOO_SMALL;
    }
    input->pDefaultMonitorModes[0].Size = sizeof(IDDCX_MONITOR_MODE);
    input->pDefaultMonitorModes[0].Origin = IDDCX_MONITOR_MODE_ORIGIN_DRIVER;
    input->pDefaultMonitorModes[0].MonitorVideoSignalInfo = makeSignalInfo();
    trace(L"default 1920x1080@60 mode queried");
    return STATUS_SUCCESS;
}

NTSTATUS XrealVirtualDisplayQueryTargetModes(
    const IDARG_IN_QUERYTARGETMODES* input,
    IDARG_OUT_QUERYTARGETMODES* output)
{
    output->TargetModeBufferOutputCount = 1U;
    if (input->TargetModeBufferInputCount == 0U)
    {
        return STATUS_BUFFER_TOO_SMALL;
    }
    input->pTargetModes[0] = makeTargetMode();
    trace(L"target 1920x1080@60 mode queried");
    return STATUS_SUCCESS;
}

NTSTATUS XrealVirtualDisplayCommitModes(
    const IDARG_IN_COMMITMODES*)
{
    trace(L"SDR BGRA topology mode committed");
    return STATUS_SUCCESS;
}

NTSTATUS XrealVirtualDisplayAssignSwapChain(
    IDDCX_MONITOR monitor,
    const IDARG_IN_SETSWAPCHAIN* input)
{
    try
    {
        auto* context = monitorContext(monitor);
        std::scoped_lock lock(context->mutex);
        context->processor = std::make_unique<SwapChainProcessor>(
            input->hSwapChain, input->RenderAdapterLuid, input->hNextSurfaceAvailable);
        return STATUS_SUCCESS;
    }
    catch (const std::exception&)
    {
        trace(L"frame pipeline allocation or thread creation failed");
        return STATUS_INSUFFICIENT_RESOURCES;
    }
    catch (...)
    {
        trace(L"frame pipeline creation failed with an unknown exception");
        return STATUS_UNSUCCESSFUL;
    }
}

void XrealVirtualDisplayUnassignSwapChain(IDDCX_MONITOR monitor)
{
    auto* context = monitorContext(monitor);
    std::scoped_lock lock(context->mutex);
    context->processor.reset();
}
