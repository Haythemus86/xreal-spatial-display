# Release baseline before capture-side downscale

Baseline collected from commit `9410b37` before capture-pipeline changes. The
machine is the validated Lenovo hybrid-GPU system: `\\.\DISPLAY1` is a
5120x1440 NVIDIA RTX 5070 Laptop GPU source and `\\.\DISPLAY5` is the
1920x1080 Intel Graphics XREAL output. Cross-adapter transfer used the explicit
CPU fallback. Each run used a one-second warmup and five measured seconds.

The benchmark window is hidden, so these results compare renderer and transfer
costs on this machine; they are not visible-display latency measurements.

| Scene | Frames | Average | p50 | p95 | p99 | Maximum | Captures | Uploads | Repeats | Private peak |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| One physical desktop | 601 | 8.332 ms | 8.371 | 9.950 | 10.290 | 14.049 | 173 | 150 | 515 | 500.4 MB |
| Desktop + checkerboard + synthetic | 601 | 8.334 ms | 8.372 | 9.990 | 10.298 | 15.142 | 174 | 150 | 516 | 539.2 MB |
| Two panels sharing one desktop + synthetic | 601 | 8.331 ms | 8.345 | 10.061 | 10.291 | 10.535 | 176 | 150 | 524 | 499.7 MB |

Every run issued one `Present` per measured frame, performed no tracked D3D11
resource creation in steady state, called no `Flush`, and reported no dropped
publication. Shared-source deduplication remained effective: the two desktop
panels still produced only 150 uploads.

The current pipeline reads the complete 5120x1440 BGRA source before any size
reduction: 29,491,200 bytes (28.125 MiB) per acquired frame and an estimated
843.75 MiB/s at the configured 30 FPS. The old benchmark did not expose exact
Map, GPU-scale, CPU-copy, publication, source-age percentile, or measured
readback/upload MiB/s histories. Those values are unavailable rather than
zero.

The raw JSON reports are intentionally stored below the untracked
`build-msvc-x64` directory:

- `baseline-pre-downscale-one-desktop.json`
- `baseline-pre-downscale-mixed.json`
- `baseline-pre-downscale-shared.json`

Command shape for the one-source baseline:

```powershell
.\build-msvc-x64\Release\xreal-spatial-renderer.exe `
  --render-monitor-index 1 --capture-monitor-index 0 `
  --multi-panel-benchmark --multi-panel-benchmark-seconds 5 `
  --multi-panel-benchmark-warmup-seconds 1 `
  --panel-count 1 --panel-layout single --panel-1-content desktop `
  --panel-1-capture-monitor-index 0 `
  --desktop-capture-cross-adapter cpu-fallback `
  --allow-desktop-capture-cpu-fallback --performance-profile balanced `
  --multi-panel-benchmark-json build-msvc-x64\baseline-pre-downscale-one-desktop.json
```
