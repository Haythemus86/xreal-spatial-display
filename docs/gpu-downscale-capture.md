# Capture-side GPU downscale

The cross-adapter CPU fallback scales and crops on the capture adapter before
the staging copy. The validated Lenovo path is therefore:

```text
NVIDIA Desktop Duplication texture
  -> capture-owned BGRA texture
  -> fullscreen-triangle scale/crop draw on NVIDIA
  -> target-sized staging texture
  -> bounded CPU BGRA buffer
  -> Intel render upload texture and panel SRV
```

No CPU image scaler exists in this path. `DesktopCaptureScaler` accepts only
`DXGI_FORMAT_B8G8R8A8_UNORM`, preserves BGRA ordering without implicit sRGB/HDR
conversion, accepts point or linear sampling, and owns three reusable staging
textures and three reusable packed CPU buffers. Textures, RTVs, SRVs, constant
buffers, shaders, and samplers are created only at initialization or after a
source/target/format/crop change. The shader bytecode is compiled once in
process-wide immutable state. Capture devices never call `Present` or `Flush`.

## Source, crop, and target

The source rectangle is the complete DXGI output. The crop rectangle selects a
validated subset. The target is the texture size copied to staging and uploaded
to the render adapter. These are reported independently.

`contain` preserves aspect ratio inside the requested target bounds. `cover`
uses a deterministic centered crop before filling the requested dimensions.
`stretch` is the only mode allowed to distort. One explicit dimension preserves
the crop aspect ratio. Explicit dimensions override scale. Upscaling is rejected
unless `--desktop-source-N-allow-upscale` is present.

For the current 5120x1440 source:

- `full` is 5120x1440;
- `center-16x9` is the centered rectangle `[1280,0,2560,1440]`;
- `full` with a 1920x540 target retains the 32:9 view;
- `center-16x9` with a 1920x1080 target retains 16:9 without distortion.

Custom rectangles use `--desktop-source-N-region x y width height` and must be
fully inside the output.

## Automatic resolution

`--desktop-resolution-policy` accepts `native`, `fixed`, or `panel-aware`.
Panel-aware planning estimates useful XREAL output coverage from panel angular
width and applies `--desktop-resolution-safety-factor` (default `1.25`). It does
not resolve below 320x180 by default. The portable transition policy uses 15%
hysteresis and a one-second minimum interval; a static plan never recreates
resources. The current renderer resolves panel-aware size at source startup.
Continuous head-motion-driven resizing is intentionally not enabled yet, so it
cannot churn capture resources.

## Threading and staging

Each `DesktopDuplicationCapture` worker owns its D3D11 device and immediate
context exclusively. No immediate context is used concurrently, no
`ID3D11Multithread` protection is needed, and the render device is never used by
a capture thread. Separate devices for sources on the same adapter deliberately
trade some device-level duplication for independent, lock-free worker progress;
sharing one immediate context would serialize sources and deferred contexts
cannot perform the required staging `Map` path directly. Immutable compiled
shader bytecode is shared across workers, while device objects are not.

The staging capacity is fixed at three and CPU buffers are not reused while a
published frame still owns one. Ring contention drops that candidate rather
than allocating or blocking another source. The current correctness-first
implementation submits `CopyResource` and then maps the selected slot in the
same worker iteration. It measures the `Map` wait explicitly, but does not yet
map an older slot asynchronously. This remaining synchronization point must be
profiled before a more complex query/command pipeline is introduced.

The capture-side scale stage currently serves the CPU fallback. The existing
same-adapter/shared-handle fast path remains native-resolution; target-sized
same-adapter GPU sharing is future work.
