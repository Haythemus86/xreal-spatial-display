# Windows virtual-display prototype

## Objective

The prototype exposes Windows desktop targets that can be routed through the
existing panel-content registry. It uses Microsoft's Indirect Display Driver
Class Extension (IddCx) with a UMDF 2 driver. Obsolete mirror drivers, XPDM and
undocumented display hooks are intentionally excluded.

This milestone targets Windows 11 x64. It is not a signed, installable product
release.

## Topology and spatial layout are separate

`VirtualDisplayConfiguration` describes Windows topology. `PanelScene`
describes where panels appear in XREAL space. Changing panel position, size or
curvature never changes the Windows desktop arrangement.

Extended mode creates one Windows monitor for each requested screen. Each
monitor has an independent compositor surface and a distinct capture-source
identity. Counts of one, two and three are supported.

Mirrored mode creates one Windows monitor and routes that one content source to
the requested one, two or three spatial panels. The panel registry deduplicates
the source, so the panels share one capture session, bridge, upload and SRV.
This is an efficient spatial mirror, not three Windows clone targets.

## Ownership

- The IddCx driver owns monitor arrival/departure, EDID identity, the supported
  mode and IddCx swap-chain consumption.
- The control process owns validation, explicit Apply/Disable actions, status
  reporting and discovery of the Windows-assigned monitor paths.
- Windows owns desktop topology placement and compositor output.
- The existing capture manager owns Desktop Duplication scheduling, GPU
  crop/downscale, fixed staging rings and cross-adapter policy.
- `PanelScene` and the D3D11 renderer own spatial layout and rendering.
- XREAL IMU, fusion, prediction and recentering never enter the driver.

## Identity and mode

The three logical monitors have deterministic UUIDs and connector indices:

- `7d5529c1-9b47-4fc4-a201-000000000001`
- `7d5529c1-9b47-4fc4-a201-000000000002`
- `7d5529c1-9b47-4fc4-a201-000000000003`

Their EDID product names are `XREAL VDISP 1`, `XREAL VDISP 2` and
`XREAL VDISP 3`; the application-facing names are `XREAL Virtual Display 1`,
`2` and `3`. Connector identity is never renumbered when a higher-numbered
monitor is removed.

Only SDR 1920x1080 at 60 Hz is advertised. Requests for other sizes or refresh
rates fail. 1280x720 and 2560x1440 remain future modes and are not advertised.

The renderer resolves a live monitor using
`DISPLAYCONFIG_TARGET_DEVICE_NAME::monitorDevicePath` when Windows has assigned
one. It does not silently fall back to a monitor index when a stable identity
was explicitly requested.

## Applying configuration

Starting the renderer or the control executable without an action does not
change topology. Example after installing the prototype driver:

```powershell
.\build-msvc-x64\Debug\xreal-virtual-display-control.exe `
  --virtual-displays 3 `
  --virtual-display-mode extended `
  --virtual-display-width 1920 `
  --virtual-display-height 1080 `
  --virtual-display-refresh-hz 60 `
  --apply-virtual-display-config
```

The driver applies `1 -> 2`, `2 -> 3`, `3 -> 1`, mode changes and Disable by
monitor arrival/departure without reinstalling. The user-mode layer then
re-enumerates display topology and associates the Windows stable path with the
logical UUID. If Windows has not assigned the target yet, routing fails with a
diagnostic instead of selecting another display.

Panel geometry and external IMU/recenter state are preserved when the routing
model is rebuilt. The prototype currently requires the caller to trigger that
rebuild; a production UI and live display-change observer remain future work.

## Known limitations

- The IddCx project still requires a WDK build and real Windows installation
  test; CMake does not build the driver.
- Desktop Duplication visibility of the new outputs must be verified on the
  target hybrid-GPU machine.
- Direct compositor-surface delivery to the renderer is only represented by
  the generic `IPanelFrameSource` boundary.
- EDID, topology transitions, sleep/resume, crash recovery and upgrades need
  production hardening.
- No production signing, packaging, installer, HDR or additional modes exist.

Architecture references: [Microsoft IddCx overview](https://learn.microsoft.com/windows-hardware/drivers/display/indirect-display-driver-model-overview),
[IddCx objects](https://learn.microsoft.com/windows-hardware/drivers/display/iddcx-objects),
and the [official Indirect Display sample](https://github.com/microsoft/Windows-driver-samples/tree/main/video/IndirectDisplay).
