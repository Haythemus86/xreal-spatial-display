# Multi-panel performance policy

The target is stable 60 FPS: average frame time below 16.67 ms, p95 below
18 ms, p99 below 22 ms, and no recurring frames above 33 ms. Conclusions must
come from Release builds on the same machine and display topology.

The D3D11 strategy is one indexed base draw and one optional line-overlay draw
per visible panel, capped at three of each, followed by one `Present`. A single
unit mesh is reused. D3D11 instancing was evaluated but not selected: panels may
bind different SRVs, which still requires draw grouping by SRV, while the
maximum instance count is three. The simple bounded draw loop has lower state
and maintenance cost.

The frame path uses fixed-capacity storage. It creates no panel geometry,
shader, sampler, state or container per frame and never calls `Flush`. Upload
textures/SRVs are created once per source dimensions/format and recreated only
on a real source change. The benchmark reports any steady-state recreation.

The default desktop cadence is time-based: selected visible consumer 30 FPS,
non-selected visible consumer 20 FPS, hidden/no-consumer 0 FPS. A shared source
uses the highest consumer request. An explicit `--panel-N-target-fps` takes
precedence over the selected profile. Desktop Duplication still waits through
`AcquireNextFrame`; cadence limiting uses a stop-aware timed condition-variable
wait on its worker thread, not polling or the render thread.
The render loop uses a non-blocking latest-frame bridge read; if publication
briefly owns the bridge lock, the existing texture is reused and contention is
counted instead of waiting on the capture thread.

BGRA bandwidth is estimated with overflow checks. A 5120x1440 source costs
29,491,200 bytes per CPU readback (about 843.8 MiB/s at 30 FPS). Capture-side GPU
downscaling is not implemented, so `source-target-width`,
`source-target-height`, and `source-scale` do not reduce CPU fallback bandwidth
yet. The application prints that limitation and never claims zero-copy for the
NVIDIA-to-Intel fallback.

`--multi-panel-benchmark` performs a configurable warmup, then records into a
fixed 1024-sample ring. It reports average/p50/p95/p99/max frame, Present, CPU
update and draw submission times; frames above 33 ms; draw/Present/constant
buffer/upload counts; capture repeats/drops; working set/private bytes; tracked
D3D resource creations; and Flush calls. Percentiles are sorted only for
rate-limited diagnostics or shutdown, not each frame.
Memory "peak" is the maximum of startup, enabled diagnostic samples, and
shutdown; it is a sampled process metric rather than an OS lifetime peak.

Example Release benchmark:

```powershell
.\build-msvc-x64\Release\xreal-spatial-renderer.exe `
  --render-monitor-index 1 `
  --multi-panel-benchmark `
  --multi-panel-benchmark-seconds 60 `
  --multi-panel-benchmark-warmup-seconds 2 `
  --multi-panel-benchmark-panels 3 `
  --multi-panel-benchmark-content synthetic `
  --multi-panel-benchmark-json multi-panel-release.json `
  --vsync
```

Debug benchmark output is explicitly marked diagnostic. Results from different
machines, adapters, output resolutions or display modes are not comparable.

## Measured Release comparison

Short five-second runs on the Lenovo topology above produced the following
machine-local results. The hidden benchmark window does not represent visible
60 Hz presentation; these numbers compare CPU/D3D composition paths only.

| Scene | Frames | Average | p50 | p95 | p99 | Maximum | Draws/frame | Uploads |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| One checkerboard | 601 | 8.332 ms | 8.330 | 8.764 | 9.012 | 9.214 | 2 | 0 |
| Three synthetic | 601 | 8.331 ms | 8.332 | 8.796 | 8.981 | 10.062 | 6 | 0 |
| Desktop + checkerboard + synthetic | 601 | 8.331 ms | 8.367 | 9.948 | 10.238 | 10.479 | 6 | 150 |
| Shared desktop on panels 1/2 + synthetic | 601 | 8.332 ms | 8.396 | 9.899 | 10.215 | 10.396 | 6 | 150 |

Every run used one Present per measured frame, created no tracked D3D resource
in steady state, called no `Flush`, and had no frame above 33 ms. The shared
desktop case used one source slot and 150 uploads, not one upload per consumer.
The mixed CPU-fallback process reached roughly 498 MB private bytes because a
5120x1440 capture requires several bounded staging/publication/upload buffers.
This is bounded but remains a performance cost to reduce in a future
capture-side scaling milestone.
