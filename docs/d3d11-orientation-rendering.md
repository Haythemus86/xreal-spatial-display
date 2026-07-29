# D3D11 orientation-driven rendering

## Purpose

`xreal-spatial-renderer` is the first rendering milestone. It renders a
synthetic world-locked panel and uses the existing fused XREAL orientation to
rotate the camera. It deliberately does not capture the desktop.

The renderer is experimental. The gyroscope scale, accelerometer mappings and
sensor-to-render mapping are not yet hardware-verified as a complete chain.

## Architecture

Two rates are kept separate:

- `XrealImuStream` owns the HID reader thread. That thread calibrates the gyro,
  updates complementary fusion and optionally predicts orientation.
- The Win32 thread owns the window, D3D11 device, immediate context, swap
  chain, keyboard processing and rendering.

`OrientationRenderBridge` publishes one mutex-protected value. Each publication
replaces the previous snapshot; there is no pose queue. The render loop always
uses the newest coherent snapshot and never blocks the sensor thread with GPU
work.

The snapshot contains measured and predicted absolute/relative quaternions,
device and host timestamps, prediction horizon, recenter generation and IMU
health counters. A missing predicted pose preserves the last valid rendered
pose and is reported as a fallback; it is never silently replaced by measured
orientation.

## Window and monitor selection

The application enumerates every active Win32 monitor and associates it with
its DXGI output by device name. Startup output prints the renderer index,
Win32 name, bounds, resolution, primary state, owning DXGI adapter and output,
adapter description and LUID. The primary monitor remains index zero;
`--monitor-index` selects another physical display explicitly.

The selected Win32 device name is retained while the window is created, so a
topology change cannot silently reinterpret the numeric index. Windowed and
borderless dimensions are constrained to the selected monitor. Explicit
`--window-x` and `--window-y` values are offsets relative to that monitor, not
virtual-desktop coordinates. Fullscreen uses the complete selected bounds and
passes the selected `IDXGIOutput` to the swap chain.

Windowed and borderless flip-model swap chains use the frame-latency waitable
object with maximum latency one. Direct3D 11 does not support that DXGI flag in
exclusive fullscreen, so fullscreen sets maximum latency to one through
`IDXGIDevice1` and resizes the flip-model buffers after the transition.

The D3D11 device is created on the adapter LUID that owns the selected output.
The renderer reports both the requested topology and actual device adapter. A
mismatch is never silent; it is expected only if the explicitly enabled WARP
fallback is used. If the selected physical display disappears before window
creation, startup fails instead of moving the renderer elsewhere.

On hybrid-GPU laptops, the HDMI monitor and XREAL display may belong to
different DXGI adapters even when Windows extends the same desktop across
both. Select the XREAL entry printed at startup, for example:

```powershell
.\build-msvc-x64\Debug\xreal-spatial-renderer.exe `
  --monitor-index 1 `
  --borderless `
  <the existing orientation and calibration options>
```

The numeric index is topology-dependent and must be checked again after
connecting, disconnecting or rearranging displays. Same-adapter rendering is
the normal optimized path. Future desktop capture must separately detect and
report when its source adapter differs from this destination adapter.

The Win32 window handles resize, close, destroy, Alt+F4 and minimized states.
Render targets are recreated after a nonzero `WM_SIZE`. A minimized window
waits for messages rather than spinning.

## D3D11 initialization

The renderer creates:

- a hardware D3D11 device and immediate context;
- a two-buffer `DXGI_SWAP_EFFECT_FLIP_DISCARD` swap chain;
- a frame-latency waitable object and maximum frame latency of one;
- BGRA render target and D24S8 depth target;
- viewport, rasterizer and alpha blend states;
- immutable panel and line vertex buffers;
- a dynamic camera constant buffer;
- vertex and pixel shaders.

WARP is attempted only with `--allow-warp-fallback`. Debug builds request the
D3D11 debug layer, retry without it when the SDK debug component is missing,
and report the result. Every failing HRESULT is returned with its operation and
numeric value.

`shaders/Panel.hlsl` is copied beside the executable and compiled at startup.
Compiler diagnostics are printed when compilation fails.

## Scene

The deterministic scene contains a 1.6 by 0.9 panel by default, placed two
world units forward. The panel has a crosshair and differently colored top,
bottom, left and right borders. Optional grid and world axes make rotation
errors visible. There are no textures or lighting.

## Quaternion, view and matrix convention

Sensor orientation uses the established convention:

- quaternion order `wxyz`;
- Hamilton multiplication;
- right-handed coordinates;
- body-to-world orientation;
- positive rotations follow the right-hand rule.

The camera needs world-to-head rotation, so the view transform is constructed
from the normalized inverse:

```text
world_from_head = render_orientation
head_from_world = inverse(world_from_head)
view_rotation = rotation_matrix(head_from_world)
```

No Euler conversion is used in the camera path.

Matrices use row-major CPU storage and column-vector multiplication. HLSL
declares the constant as `row_major float4x4` and evaluates
`mul(viewProjection, position)`, so the CPU does not transpose the matrix.
D3D depth is mapped to the 0-to-1 range by a right-handed perspective matrix;
the camera looks along render `-Z`.

## Coordinate mapping

The initial explicit mapping is:

```text
sensor +X -> render +X
sensor +Y -> render -Z
sensor +Z -> render +Y
```

This is a proper right-handed basis rotation. Quaternion mapping uses the basis
change `M * R * inverse(M)`; axis signs are not hidden in camera matrix code.
The mapping is printed and marked experimental.

The first Air 2 Ultra hardware rendering test validated yaw, roll, recenter and
clear-recenter behavior, but found pitch inverted. Inspection showed that
flipping only one rotational degree of freedom cannot be expressed as a proper
render basis change while preserving the other two. The render basis above
therefore remains unchanged. The concrete defect is corrected before fusion by
these explicit experimental measurement mappings:

```text
gyroscope:     sensor X -> body -X, sensor Y -> body +Y, sensor Z -> body +Z
accelerometer: sensor X -> body +X, sensor Y -> body -Y, sensor Z -> body +Z
```

Gyroscope X carries the physical pitch angular rate. Accelerometer Y carries
the gravity component that constrains the same rotation. Both signs are
corrected together so complementary correction does not oppose integration.
Yaw-related gyroscope Z and roll-related gyroscope Y are unchanged. The
second Air 2 Ultra hardware test confirmed corrected pitch without regressing
yaw, roll, recenter or clear-recenter behavior. The mapping remains marked
experimental because validation currently covers one device and one Windows
hardware configuration.

## Orientation selection and recentering

`--render-orientation-source` selects `measured` or `predicted`.
`--render-orientation-frame` selects `absolute` or `relative`. The recommended
initial rendering horizon is 15 ms.

Absolute orientation is never overwritten by recentering. `R` stores the
current fused absolute orientation as the relative reference. `C` clears the
reference, making relative and absolute orientations equivalent again. The
recenter generation travels with each snapshot.

Hardware predicted rendering requires `--predict-orientation`. Measured mode
works without prediction. Both fusion and prediction remain experimental.

## Startup calibration

Hardware startup passes through:

```text
initializing_renderer -> opening_imu -> warming_up_gyro -> calibrating_gyro
-> initializing_fusion -> ready
```

Failures enter `error`; cleanup enters `shutting_down`. While calibration is
not ready, the window shows a static panel on an orange background and reports
the state in its title. Keep the glasses completely still until calibration is
accepted. Head-driven rendering never begins with rejected calibration.

## Keyboard controls

- `Escape`: exit
- `R`: recenter
- `C`: clear recenter
- `P`: toggle measured/predicted
- `F`: toggle absolute/relative
- `G`: toggle the grid
- `X`: toggle world axes
- `V`: toggle vsync
- `D`: toggle console diagnostics
- Demo only: arrows change yaw/pitch, `Q`/`E` change roll, `Space` resets

Key-down transitions are debounced; holding a key does not repeatedly recenter.

## Demo and smoke modes

`--orientation-demo-mode` never initializes HID. It publishes deterministic
synthetic measured and predicted quaternions and accepts keyboard offsets.
Hardware errors never activate demo mode automatically.

`--smoke-test --smoke-test-frames 10` explicitly enables demo mode, creates a
hidden window, renders the requested number of D3D11 frames and exits. It is
not registered as CTest because graphical availability is not reliable on
headless systems.

## Timing diagnostics

The final statistics include frame/present counts, average/minimum/maximum FPS,
average/maximum frame time, slow frames, sensor publication rate, repeated or
invalid snapshots and prediction fallbacks.

For each rendered snapshot:

```text
snapshot_age_ms = render_host_time - snapshot_publish_host_time
approximate_effective_lead_ms = prediction_horizon_ms - snapshot_age_ms
```

This host-side diagnostic does not alter IMU integration and is not a
motion-to-photon measurement.

## JSON output

`--render-json-output <file.json>` writes adapter, window, monitor, swap chain,
fusion, prediction, mapping, calibration, frame timing, snapshot statistics,
IMU counters and shutdown details. Non-finite numbers serialize as `null`.

## Demo command

```powershell
.\build-msvc-x64\Debug\xreal-spatial-renderer.exe `
  --orientation-demo-mode `
  --window-width 1280 `
  --window-height 720 `
  --render-orientation-source predicted `
  --render-orientation-frame relative `
  --prediction-horizon-ms 15 `
  --background-grid `
  --world-axes `
  --vsync `
  --show-render-diagnostics
```

## First hardware command

```powershell
.\build-msvc-x64\Debug\xreal-spatial-renderer.exe `
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
  --window-width 1280 `
  --window-height 720 `
  --background-grid `
  --world-axes `
  --vsync `
  --show-render-diagnostics `
  --render-json-output orientation-render.json
```

## Hardware validation

1. Put the glasses flat and still, start the renderer and wait for `ready`.
2. Wear them, look forward and press `R`.
3. Turn right: the panel should move left. Turn left: it should move right.
4. Look up: the panel should move down. Look down: it should move up.
5. Tilt clockwise: the world should rotate counter-clockwise.
6. Press `P`, compare quick turns in measured then predicted mode.
7. Press `R` in a new direction, then `C`, and verify the documented relative
   reference behavior.
8. Remain centered for 30 seconds and check stability, frame rate and absence
   of jumps or non-finite output.

## Limitations

- experimental gyroscope scale and axis mappings;
- yaw is not absolutely corrected;
- orientation-only 3DoF, no positional 6DoF;
- no desktop capture or virtual desktop texture;
- no stereoscopic/per-eye rendering or lens distortion correction;
- no OpenXR, spatial anchors, hand tracking or production UI;
- no motion-to-photon measurement.
