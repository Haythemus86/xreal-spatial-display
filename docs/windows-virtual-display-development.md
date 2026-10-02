# Virtual-display driver development

## Prerequisites

Install Visual Studio 2022 Build Tools or Visual Studio 2022 with C++ desktop
development, the matching Windows 11 SDK, the Windows Driver Kit (WDK), Spectre
libraries required by the selected WDK, and the Windows Driver Kit Visual
Studio extension. The application needs CMake 3.24 or newer.

The reference machine used for this change has Windows SDK 10.0.26100 but no
WDK headers, IddCx import library or driver MSBuild integration. Consequently,
the application and protocol are validated there, but the driver binary is
not. Do not interpret a CMake success as a driver build.

The prototype project requests UMDF 2, the Universal target and IddCx 1.10. Its
implemented feature set is deliberately basic: it does not depend on HDR or
newer optional IddCx capabilities. Compatibility with each supported Windows
11 release must be validated with the matching WDK before distribution.

## Build

Open a Developer PowerShell for Visual Studio after installing the WDK:

```powershell
msbuild .\virtual-display\driver\XrealVirtualDisplayDriver.sln `
  /p:Configuration=Debug /p:Platform=x64
```

Use `Release` only after Debug validation. The generated package must pass
InfVerif/Inf2Cat and be signed before installation. Exact output paths depend
on the installed WDK and MSBuild configuration; inspect MSBuild output rather
than assuming a package exists.

## Development signing and installation

Test signing weakens normal boot policy. Nothing in this repository enables it
or changes BCD. On a dedicated test machine, an administrator may create and
trust a development certificate, sign the generated catalog with `signtool`,
and manually enable test-signing if required:

```powershell
bcdedit /set testsigning on
```

A reboot is required. Secure Boot policy can prevent this command. Follow
Microsoft's driver-signing documentation and never do this on a production
machine. Restore normal policy manually after testing:

```powershell
bcdedit /set testsigning off
```

Install the signed package and create the root-enumerated prototype device from
an elevated WDK command prompt:

```powershell
pnputil /add-driver .\XrealVirtualDisplayDriver.inf /install
devcon install .\XrealVirtualDisplayDriver.inf Root\XrealVirtualDisplay
```

`devcon` is a WDK development tool. For uninstall, first identify the exact
device and published INF name; do not guess `oemNN.inf`:

```powershell
devcon remove Root\XrealVirtualDisplay
pnputil /enum-drivers
pnputil /delete-driver oemNN.inf /uninstall
```

Device restart, when needed, remains explicit:

```powershell
pnputil /restart-device "ROOT\XREALVIRTUALDISPLAY\..."
```

No command above is executed by the application.

## Validation and diagnostics

Check these in order:

1. Device Manager shows `XREAL Virtual Display Prototype Adapter` with no
   error status.
2. Windows Settings > System > Display shows only the explicitly applied
   `XREAL VDISP` targets at 1920x1080/60.
3. `--show-virtual-display-status` reports requested and actual counts,
   generation, state, restart requirement and Windows identities.
4. The renderer monitor list shows each target in Win32 and DXGI enumeration.
5. Desktop Duplication can independently capture each Extended target.

Use DebugView or WinDbg to capture `OutputDebugString` messages from the UMDF
host. Relevant events include driver/adapter load, monitor arrival/departure,
mode queries, swap-chain start/stop, generation application and errors. The
swap-chain loop does not log per frame.

Microsoft's [indirect-display debugging guide](https://learn.microsoft.com/windows-hardware/drivers/display/indirect-display-debugging)
describes UMDF host debugging and IddCx tracing. Production work still needs
fault injection, sleep/resume, device restart, upgrade/uninstall and verifier
coverage.
