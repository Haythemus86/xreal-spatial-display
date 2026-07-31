# Multi-source Desktop Duplication

`DesktopCaptureManager` owns up to three independent source runtimes. Each
runtime has one Desktop Duplication object, worker thread, capture D3D11 device
and immediate context, scaler, fixed staging ring, and bounded latest-frame
bridge. Workers never call `Present`, never access render-device state, and stop
independently through joined `std::jthread` ownership.

The panel content registry forms source identity from:

- physical monitor selector;
- transfer policy;
- resolution policy and safety factor;
- crop mode and custom region;
- target width, height, and scale;
- scale fit, filter, and explicit upscale permission.

Identical keys share one capture runtime and one render upload slot. A changed
crop or target creates a distinct pipeline because it produces different
pixels. Three unique sources have three render-side textures/SRVs. The renderer
uploads only a new sequence and all panels consuming that slot sample the same
SRV. Capacity is fixed at three; a fourth source configuration is rejected.

## Scheduling

The highest requested rate among consumers wins for a shared source. The
profiles resolve to:

| Profile | Selected | Visible non-selected | Hidden-only |
|---|---:|---:|---:|
| quality | 60 Hz | 30 Hz | 0 Hz |
| balanced | 30 Hz | 20 Hz | 0 Hz |
| performance | 30 Hz | 15 Hz | 0 Hz |

Explicit per-source capture/upload options override profile defaults:

```text
--desktop-source-N-capture-fps <hz>
--desktop-source-N-upload-fps <hz>
```

Workers use monotonic condition-variable deadlines, including an event-based
pause at 0 Hz. The render loop never waits for a source: it performs a
non-blocking latest-frame read and keeps rendering the existing SRV between
updates.

Runtime layout controls can change selection and therefore source rates. A
content identity or monitor change currently requires restarting the renderer;
hot creation/release of a completely new duplication topology is not yet
exposed by the interactive controls.

## Static and stale desktops

DXGI `WAIT_TIMEOUT` means no new desktop image, not invalid content. The
retention state distinguishes `no_frame_ever`, `active_unchanged`,
`stale_but_valid`, `unavailable`, and `access_lost`. After one successful
upload, the renderer retains the texture indefinitely. Staleness remains
diagnostic metadata and never changes effective panel content to unavailable.
Access loss has a separate recovery state and source identity remains stable.

## Future virtual displays

The manager already treats monitor identity and source policy independently,
so one, two, or three future Windows virtual outputs can map to the three fixed
slots without changing the render scene. A virtual-display driver is not part
of this milestone. Hot-plug topology reconciliation and a target-sized
same-adapter shared-texture path should be completed before driver integration
is considered production-ready.

Release capture benchmark mode assigns a non-persistent benchmark instance ID
to each requested panel so three otherwise identical configurations do not
deduplicate. Instance 1 remains real Desktop Duplication; instances 2 and 3 are
clearly labelled synthetic GPU pipelines when extra physical outputs are not
available. Normal rendering never sets this ID, so production deduplication is
unchanged.
