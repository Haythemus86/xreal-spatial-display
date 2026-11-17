# AGENTS.md

## Project overview

This repository contains `xreal-spatial-display`, an independent open-source
Windows application for XREAL glasses.

The goal is to provide a low-latency spatial virtual display with:

- XREAL sensor access
- 3DoF head tracking
- adjustable virtual screen size
- adjustable distance and position
- manual recentering
- low-latency Windows desktop and game capture
- support for Intel, AMD, and NVIDIA GPUs
- Windows 10 and Windows 11 x64 support

This project is not affiliated with or endorsed by XREAL.

---

## Current project phase

The project is currently in the hardware validation phase.

The immediate objective is to build a console application named:

`xreal-sensor-diagnostic`

It must:

1. initialize HIDAPI
2. detect connected XREAL HID interfaces
3. display:
   - vendor ID
   - product ID
   - interface number
   - manufacturer name
   - product name
   - serial number
   - HID device path
4. shut down HIDAPI cleanly
5. compile successfully with MSVC on Windows x64

Do not implement desktop capture, Direct3D rendering, OpenXR, a GUI, or IMU
packet decoding until the HID detection phase has been validated on real
hardware.

---

## Target platform

Primary target:

- Windows 11 x64

Secondary target:

- Windows 10 x64

Supported GPU vendors:

- Intel
- AMD
- NVIDIA

Do not introduce mandatory dependencies on vendor-specific APIs such as:

- CUDA
- NVAPI
- AMD AGS
- Intel-specific GPU extensions

Vendor-specific optimizations may be added later only as optional modules.

---

## Technology stack

Use the following technologies unless a change is explicitly approved:

- C++20
- CMake
- MSVC v143
- Windows SDK
- Win32
- HIDAPI
- Direct3D 11
- DXGI
- HLSL
- Dear ImGui for the future configuration interface

Planned capture backends:

- DXGI Desktop Duplication for low-latency monitor capture
- Windows Graphics Capture as an alternative backend

Do not introduce Unity, Unreal Engine, Electron, .NET, Qt, Vulkan, or Direct3D
12 without first explaining the concrete benefit and the maintenance cost.

---

## Build environment

The reference build environment is:

- Windows 11
- Visual Studio Build Tools 2022
- MSVC v143
- Windows 11 SDK
- CMake 3.24 or newer
- VS Code with CMake Tools

Reference commands:

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Debug