# Desktop Duplication panel

## Purpose

This experimental milestone maps a selected Windows desktop onto the existing
world-locked Direct3D 11 panel. Synthetic content remains the default. Desktop
capture starts only with `--panel-content desktop` and an explicit capture
monitor selector.

No stereoscopic rendering, 6DoF, OpenXR, HDR conversion or Windows Graphics
Capture is included.

## Monitor and adapter selection

Render and capture selection are independent:

- `--monitor-index` remains an alias for `--render-monitor-index`;
- `--render-monitor-device-name` overrides the render index;
- `--capture-monitor-device-name` overrides the capture index;
- a missing named output is an error and never falls back to another index.

Indices follow the current active Win32 monitor enumeration and may change
after reconnects. Device names are preferable for repeatable hardware tests,
but Windows may also reassign those names.

The shared `DisplayTopology` enumeration resolves each monitor to its Win32
bounds, `HMONITOR` identity, DXGI output, owning adapter and adapter LUID. The
renderer device is created on the render-output adapter. The capture device is
created separately on the capture-output adapter.

For the validated Lenovo topology, the intended route is:

```text
DISPLAY1 / NVIDIA RTX 5070 / HDMI desktop
    -> Desktop Duplication capture device
    -> owned shared texture
    -> NT shared handle plus keyed mutex
    -> render-owned Intel texture
    -> DISPLAY5 / Intel Graphics / XREAL
```

## Duplication and texture ownership

The capture thread owns `IDXGIOutputDuplication` and performs
`AcquireNextFrame`. It copies a new acquired texture into an owned resource
before `ReleaseFrame`; duplication-owned resources never escape the acquisition
scope.

The default GPU transfer creates the owned texture with
`D3D11_RESOURCE_MISC_SHARED_NTHANDLE` and
`D3D11_RESOURCE_MISC_SHARED_KEYEDMUTEX`. The render device opens that handle and,
under keyed-mutex synchronization, copies it into a render-owned shader-resource
texture. The shader samples only the local render texture. This is a GPU copy,
not a verified zero-copy path.

Same-adapter and cross-adapter LUIDs are reported separately. `auto` and
`shared-handle` attempt the explicit NT-handle path. If the drivers do not allow
the requested sharing, the error is reported; capture does not silently switch
to CPU or GDI.

`cpu-fallback` is available only when both of these are present:

```text
--desktop-capture-cross-adapter cpu-fallback
--allow-desktop-capture-cpu-fallback
```

That path uses one reusable staging texture and three fixed reusable host
buffers. It performs GPU-to-CPU and CPU-to-GPU copies and reports bytes copied.
It is intentionally disabled by default and has a latency/bandwidth cost.

## Threads and publication

- The sensor thread remains unchanged and publishes the latest orientation.
- The capture thread owns duplication, acquisition, metadata and capture-device
  copies.
- The render thread owns the window, swap chain, render context and SRV use.

`DesktopCaptureBridge` stores one latest frame. A new publication replaces an
unconsumed frame and increments the dropped-publication counter. `ComPtr`
ownership is hidden behind a shared surface lifetime token. There is no
unbounded frame queue and the capture thread never calls `Present`.

## Panel mapping

`contain` preserves aspect ratio and letterboxes or pillarboxes. `cover`
preserves aspect ratio and crops. `stretch` is the only mode that distorts.
Rotation values 0, 90, 180 and 270 degrees are mapped explicitly. Texture
coordinates use the D3D top-left convention; no implicit vertical inversion is
applied. `--desktop-flip-y` is the only vertical-flip control.

Point and linear sampling are supported. BGRA UNORM and BGRA UNORM sRGB are
accepted. Other formats, including unsupported HDR paths, fail visibly rather
than being reinterpreted.

Desktop Duplication does not guarantee a meaningful alpha channel. Captured
BGRA pixels are therefore sampled as RGB with alpha forced to `1.0` before the
existing `SRC_ALPHA` panel blend. Without this rule, a valid desktop whose alpha
bytes are zero becomes invisible while the separately drawn border and
crosshair remain visible.

`--desktop-debug-checkerboard` skips duplication and publishes a generated,
tightly packed 640x360 BGRA checkerboard through the same latest-frame bridge, Intel
upload texture, `UpdateSubresource`, SRV and desktop pixel-shader branch. It is
the renderer-only isolation test for this path. The pattern contains alternating
magenta/green tiles, red and blue corner markers, a white center marker and
opaque alpha on every pixel. Its stable FNV-1a checksum is
`0xF104DB9F3DA2C325`.

`--orientation-demo-static` selects identity measured and predicted demo
orientations. It removes the normal demo animation from render-path diagnosis.

## Renderer texture diagnostics

The panel shader contracts are explicit: `desktopTexture` is `t0`,
`desktopSampler` is `s0`, and `CameraConstants` is `b0`. C++ binds `b0` to both
the vertex and pixel stages. The constants use 32-bit integer flags and a
144-byte, 16-byte-aligned layout; field offsets are compile-time checked.

An earlier checkerboard failure was caused by binding `b0` only with
`VSSetConstantBuffers`. The pixel shader therefore read zero for
`desktopEnabled` and returned the synthetic vertex color even though the SRV
was valid and bound. The renderer now calls both `VSSetConstantBuffers` and
`PSSetConstantBuffers` for slot zero.

The following mutually exclusive options force the base panel pixel shader:

- `--desktop-debug-shader-solid-red` returns opaque red;
- `--desktop-debug-shader-uv` visualizes interpolated UV coordinates;
- `--desktop-debug-shader-sample` samples `t0` without normal compositing;
- `--desktop-debug-shader-sample-no-overlay` samples `t0` and suppresses the
  border/crosshair overlay.

`--desktop-debug-opaque-base` disables blending for the base panel and keeps
all render-target color writes enabled. Border, crosshair, optional grid and
axes use a separate overlay pixel shader and the alpha blend state in a second
pass. The desktop base always outputs alpha one and is never covered by a later
opaque synthetic panel.

`--desktop-debug-readback-upload <file.bmp>` copies the render-adapter upload
texture once to a staging texture, handles mapped `RowPitch`, writes a BMP and
compares its checksum with the packed CPU source. A mismatch is an error.
`--desktop-debug-dump-render-target <file.bmp>` similarly dumps the swap-chain
render target once after the panel and overlay draws but before `Present`.
Both diagnostics are disabled by default and intentionally synchronize the GPU,
so they are not performance modes.

The first diagnostic frame records this order: texture update, SRV ready,
constant update, geometry and shader binding, SRV/sampler/VS+PS constant-buffer
binding, state binding, base `DrawIndexed`, optional overlay draw, SRV unbind,
then `Present`. The upload texture and SRV descriptors are printed and checked
against a single-mip BGRA UNORM `D3D11_USAGE_DEFAULT` shader resource.

`--desktop-capture-dump-first-frame <file.bmp>` writes the first valid CPU frame
once. Rows are repacked to `width * 4` before publication and the BMP uses a
negative height, so it is top-down and does not reverse the captured image.

The current cursor support records visibility and position metadata but does
not render pointer shapes. `--desktop-hide-cursor` suppresses the requested
cursor behavior; full color/monochrome shape composition remains future work.
Dirty and move rectangle counts are collected, but this milestone performs a
full-frame copy and does not claim partial-update optimization.

## Recovery

Capture states are `disabled`, `initializing`, `active`, `wait_timeout`,
`access_lost`, `recreating`, `output_missing`, `unsupported`, `fatal_error` and
`shutting_down`.

`DXGI_ERROR_WAIT_TIMEOUT` is non-fatal. Access loss, device removal and device
reset recreate duplication after the configured bounded delay. Recreation
resolves the same adapter LUID and output device name; it never selects a
different monitor. While no valid texture exists, the world-locked panel uses a
deterministic unavailable pattern. The last render-owned frame can be repeated
while capture waits.

## Diagnostics and JSON

`--desktop-capture-diagnostics` reports capture state, source size/format,
rotation, FPS, source age, repeated frames, dropped publications, access loss
and recovery generation at the renderer diagnostic rate.

The first frame also emits one trace for acquisition, successful staging map,
publication, renderer consumption, upload-texture creation,
`UpdateSubresource`, SRV creation, SRV binding, pixel-shader selection and
desktop rendering. All capture, upload, SRV and effective-panel counters are
included in both render and capture JSON outputs.

`--desktop-capture-json-output` writes schema version 1 with monitor and adapter
identities, LUIDs, transfer mode, CPU fallback use, duplication state, format,
dimensions, rotation, frame/recovery counters, metadata counters and errors.
Unavailable floating-point values are serialized as `null`; NaN and infinity
are never emitted. Motion-to-photon latency is explicitly marked unmeasured.

## First Lenovo hardware command

```powershell
.\build-msvc-x64\Debug\xreal-spatial-renderer.exe `
  --render-monitor-index 1 `
  --capture-monitor-index 0 `
  --fullscreen `
  --panel-content desktop `
  --desktop-fit contain `
  --desktop-filter linear `
  --desktop-show-cursor `
  --desktop-capture-cross-adapter auto `
  --gyro-calibrate-seconds 2 `
  --gyro-warmup-seconds 1 `
  --apply-gyro-bias `
  --gyro-scale-raw-per-dps 4090 `
  --accelerometer-profile xreal-air2-ultra-calibration.json `
  --fusion-mode complementary `
  --fusion-startup gravity `
  --accelerometer-correction-time-constant 2.0 `
  --accelerometer-max-correction-dps 10 `
  --predict-orientation `
  --prediction-mode constant-velocity `
  --prediction-horizon-ms 15 `
  --prediction-angular-velocity-smoothing-seconds 0.01 `
  --prediction-max-angular-speed-dps 1000 `
  --prediction-max-angle-degrees 30 `
  --prediction-limit-behavior reject `
  --render-orientation-source predicted `
  --render-orientation-frame relative `
  --recenter-on-start `
  --vsync `
  --show-render-diagnostics `
  --desktop-capture-diagnostics `
  --render-json-output orientation-desktop-render.json `
  --desktop-capture-json-output desktop-capture.json
```

Expected diagnostics identify DISPLAY1/NVIDIA as capture, DISPLAY5/Intel as
render, `cross-adapter`, and `shared_nt_handle_cross_adapter`. The HDMI desktop
should update inside the world-locked XREAL panel while the HDMI monitor remains
connected.

## Hardware checks

1. Keep the HDMI monitor connected, close the laptop lid, and connect XREAL.
2. Verify the printed render/capture device names, adapter descriptions and
   LUIDs before accepting the result.
3. Move windows and the pointer on the HDMI desktop; verify continuous texture
   updates. Pointer shape rendering is not expected yet.
4. After calibration, press `R` and verify the panel stays world-locked in all
   validated yaw, pitch and roll directions.
5. Press `P`; capture must continue unchanged while measured/predicted
   orientation switches.
6. If safe, change the capture display mode and verify recovery or the explicit
   unavailable pattern.
7. Run at least five minutes and check stable render/capture FPS, bounded memory,
   no access-loss loop, finite diagnostics and clean shutdown.

Known limitations are driver-dependent cross-adapter NT-handle support, no HDR
conversion, metadata-only cursor support, full-frame copies, and no measured
motion-to-photon latency.
