# Multi-panel layout

This milestone composes one to three spatial panels in one D3D11 scene. Scene
units are meters. The renderer owns one D3D11 device, one flip-discard swap
chain and one presentation path. All panels share immutable unit-quad geometry,
shaders, samplers and pipeline states. Each visible panel receives one cached
world transform and one per-frame constant-buffer update for shared camera
state. Static world matrices refresh only when layout geometry changes.

`PanelScene` and `PanelContentRegistry` use fixed arrays with a capacity of
three. Panel IDs are stable zero-based values internally and the CLI names
panels 1 through 3. Iteration order is deterministic. Synthetic and procedural
checkerboard sources are shared by type. Equal desktop monitor selectors map to
one capture session, bridge, upload texture and SRV.

The implemented layouts are `single`, `dual-flat`, `triple-flat`,
`triple-angled`, and `custom`. The angled preset uses symmetric side-panel yaw.
Position uses +X right, +Y up and -Z forward from the camera.

Representative options:

```powershell
--panel-count 3
--panel-layout triple-angled
--panel-width-m 1.6
--panel-height-m 0.9
--panel-distance-m 2.0
--panel-gap-m 0.12
--panel-curvature-degrees 18
--panel-1-content desktop
--panel-1-capture-monitor-index 0
--panel-2-content checkerboard
--panel-3-content synthetic
```

Per-panel `width-m`, `height-m`, `position-x-m`, `position-y-m`,
`position-z-m`, `yaw-degrees`, `pitch-degrees`, `target-fps`, capture monitor,
source target dimensions and source scale are accepted using the
`--panel-N-...` form.

Runtime keys:

- `1`, `2`, `3`: select a panel
- `Tab`: select the next panel
- arrows: move selected panel horizontally/vertically
- `[` / `]`: resize selected panel
- `I` / `O`: increase/decrease distance
- `Q` / `E`: yaw; `W` / `S`: pitch
- `,` / `.`: change layout gap
- `H`: hide/show; `L`: switch flat/angled preset; `Z`: reset layout
- `U`: reset only the selected panel
- `K`: save to `--panel-layout-save-file`

Layout JSON schema version 1 records meter units, preset, global defaults,
stable IDs, display names, enabled state, transforms, sizes, content/source
identity, fit, filter, overlay state, transfer policy, target rates and
requested source sizing. Loading rejects unsupported versions,
non-finite values, invalid counts and unstable IDs. Save uses a temporary file
and rename. Existing files require `--overwrite-panel-layout`; they are never
silently replaced.

Current limitation: all desktop panels may share one unique Desktop
Duplication source. A second distinct desktop output is rejected explicitly.
This keeps capture and uploads honest while the renderer has one production
desktop SRV slot. Synthetic, checkerboard and one real desktop source can be
mixed independently.

Panel world size, source resolution, and composed output resolution are
independent quantities. A larger panel changes angular size, not capture
resolution. A higher-resolution source can improve sampling but cannot exceed
the 1920x1080 XREAL output. Rate-limited diagnostics label angular width and
approximate output pixel coverage; they do not describe a partially covered
panel as a native-resolution monitor.

The Windows virtual-display driver is deliberately deferred. These panels are
compositor objects, not separate monitors exposed to Windows. A future driver
prototype can publish independent Windows surfaces into the source registry
without adding D3D devices, swap chains, or render loops.

## Lenovo/XREAL validation commands

These commands use the validated topology: render monitor 1 is the XREAL
1920x1080 Intel output and capture monitor 0 is the 5120x1440 NVIDIA output.

Test A, one-panel checkerboard regression:

```powershell
.\build-msvc-x64\Release\xreal-spatial-renderer.exe `
  --render-monitor-index 1 --fullscreen --orientation-demo-static `
  --panel-count 1 --panel-layout single --panel-1-content checkerboard `
  --vsync --show-render-diagnostics
```

Test B, triple-angled synthetic panels, first static and then with the IMU:

```powershell
.\build-msvc-x64\Release\xreal-spatial-renderer.exe `
  --render-monitor-index 1 --fullscreen --orientation-demo-static `
  --panel-count 3 --panel-layout triple-angled `
  --panel-1-content synthetic --panel-2-content checkerboard `
  --panel-3-content synthetic --vsync --show-render-diagnostics

.\build-msvc-x64\Release\xreal-spatial-renderer.exe `
  --render-monitor-index 1 --fullscreen `
  --panel-count 3 --panel-layout triple-angled `
  --panel-1-content synthetic --panel-2-content checkerboard `
  --panel-3-content synthetic `
  --gyro-calibrate-seconds 2 --gyro-warmup-seconds 1 --apply-gyro-bias `
  --gyro-scale-raw-per-dps 4090 `
  --accelerometer-profile xreal-air2-ultra-calibration.json `
  --fusion-mode complementary --fusion-startup gravity `
  --accelerometer-correction-time-constant 2.0 `
  --accelerometer-max-correction-dps 10 --predict-orientation `
  --prediction-mode constant-velocity --prediction-horizon-ms 15 `
  --prediction-angular-velocity-smoothing-seconds 0.01 `
  --prediction-max-angular-speed-dps 1000 `
  --prediction-max-angle-degrees 30 --prediction-limit-behavior reject `
  --render-orientation-source predicted --render-orientation-frame relative `
  --recenter-on-start --vsync --show-render-diagnostics
```

Test C, one real desktop source plus two low-cost independent sources:

```powershell
.\build-msvc-x64\Release\xreal-spatial-renderer.exe `
  --render-monitor-index 1 --fullscreen --orientation-demo-static `
  --panel-count 3 --panel-layout triple-angled `
  --panel-1-content desktop --panel-1-capture-monitor-index 0 `
  --panel-2-content checkerboard --panel-3-content synthetic `
  --desktop-capture-cross-adapter cpu-fallback `
  --allow-desktop-capture-cpu-fallback --desktop-show-cursor `
  --performance-profile balanced --vsync --show-render-diagnostics `
  --desktop-capture-diagnostics
```

Test D, two consumers of the same desktop source:

```powershell
.\build-msvc-x64\Release\xreal-spatial-renderer.exe `
  --render-monitor-index 1 --fullscreen --orientation-demo-static `
  --panel-count 3 --panel-layout triple-angled `
  --panel-1-content desktop --panel-1-capture-monitor-index 0 `
  --panel-2-content desktop --panel-2-capture-monitor-index 0 `
  --panel-3-content synthetic `
  --desktop-capture-cross-adapter cpu-fallback `
  --allow-desktop-capture-cpu-fallback --performance-profile balanced `
  --vsync --show-render-diagnostics --desktop-capture-diagnostics
```

Test E uses Test B's static command plus
`--panel-layout-save-file xreal-three-panel-layout.json
--overwrite-panel-layout`. Exercise the printed key map, press `K`, exit, then
reload exactly with:

```powershell
.\build-msvc-x64\Release\xreal-spatial-renderer.exe `
  --render-monitor-index 1 --fullscreen --orientation-demo-static `
  --panel-layout-file xreal-three-panel-layout.json --vsync
```

Test F, ten-minute real-IMU mixed-content run:

```powershell
.\build-msvc-x64\Release\xreal-spatial-renderer.exe `
  --render-monitor-index 1 --fullscreen --render-duration 600 `
  --panel-count 3 --panel-layout triple-angled `
  --panel-1-content desktop --panel-1-capture-monitor-index 0 `
  --panel-2-content checkerboard --panel-3-content synthetic `
  --desktop-capture-cross-adapter cpu-fallback `
  --allow-desktop-capture-cpu-fallback --desktop-show-cursor `
  --gyro-calibrate-seconds 2 --gyro-warmup-seconds 1 --apply-gyro-bias `
  --gyro-scale-raw-per-dps 4090 `
  --accelerometer-profile xreal-air2-ultra-calibration.json `
  --fusion-mode complementary --fusion-startup gravity `
  --accelerometer-correction-time-constant 2.0 `
  --accelerometer-max-correction-dps 10 --predict-orientation `
  --prediction-mode constant-velocity --prediction-horizon-ms 15 `
  --prediction-angular-velocity-smoothing-seconds 0.01 `
  --prediction-max-angular-speed-dps 1000 `
  --prediction-max-angle-degrees 30 --prediction-limit-behavior reject `
  --render-orientation-source predicted --render-orientation-frame relative `
  --recenter-on-start --performance-profile balanced --vsync `
  --show-render-diagnostics --render-diagnostics-rate 1 `
  --desktop-capture-diagnostics `
  --performance-json-output multi-panel-long-run.json
```
