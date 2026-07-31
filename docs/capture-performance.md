# Desktop capture performance

Performance measurements must use Release builds. Debug runs validate behavior,
not latency or bandwidth.

## Bandwidth budget

Exact BGRA byte counts are `width * height * 4`:

| Transfer size | Bytes/frame | MiB/frame | MiB/s at 30 Hz |
|---|---:|---:|---:|
| 5120x1440 | 29,491,200 | 28.125 | 843.750 |
| 1920x1080 | 8,294,400 | 7.910 | 237.305 |
| 1920x540 | 4,147,200 | 3.955 | 118.652 |
| 3 x 1920x1080 | 24,883,200 | 23.730 | 711.914 |

Diagnostics track original, crop, scaled, mapped, CPU-copied, and render-upload
bytes separately. They also expose capture/upload FPS, `AcquireNextFrame`, GPU
submission, `Map` wait, row repack and `UpdateSubresource` timings. The default
`--desktop-bandwidth-warning-mib-s 1000` threshold is informational and
machine-dependent; it never aborts capture.

The acceptance targets for the 1920x1080 XREAL output are stable 60 FPS,
average frame time at most 16.67 ms, p95 at most 18 ms, p99 below 22 ms, no
recurring 33 ms spikes, one Present per scene frame, no steady-state resource
creation, and no `Flush`.

## Measured post-change Release results

Measured on the validated Lenovo topology with one-second warmup and five
measured seconds. The swap chain presented about 122 Hz on this output, so the
8.33 ms frame time is comfortably inside the 60 FPS budget.

| Run | Transfer(s) | Average | p95 | p99 | Max | Readback | Upload | Steady resources | Flush |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Native physical | 5120x1440 | 8.330 ms | 10.141 | 10.263 | 10.468 | 837.18 MiB/s | 680.91 MiB/s | 0 | 0 |
| Physical full | 1920x540 | 8.330 ms | 8.715 | 8.900 | 9.072 | 118.36 MiB/s | 101.12 MiB/s | 0 | 0 |
| Physical center 16:9 | 1920x1080 | 8.330 ms | 8.982 | 9.142 | 9.288 | 235.21 MiB/s | 189.74 MiB/s | 0 | 0 |
| Real + one synthetic | 1920x540 + 1920x1080 | 8.330 ms | 8.946 | 9.355 | 9.503 | 351.95 MiB/s | 211.17 MiB/s | 0 | 0 |
| Real + two synthetic | 1920x540 + 2x1920x1080 | 8.330 ms | 9.115 | 9.341 | 9.834 | 587.67 MiB/s | 317.64 MiB/s | 0 | 0 |

The physical 1920x1080 detailed run measured capture `AcquireNextFrame`
p50/p95/p99 of 0.948/8.209/8.331 ms, GPU submission
0.007/0.020/0.030 ms, staging `Map` wait 0.777/4.554/5.056 ms, and CPU row
repack 1.010/1.129/1.319 ms. `UpdateSubresource` averaged 0.846 ms. These are
short machine-specific samples, not universal latency claims.

Peak private bytes in the same runs were about 487 MB native, 165 MB at
1920x540, 245 MB at 1920x1080, 373 MB with two pipelines, and 525 MB with three
pipelines. No measured run reported a recurring frame above 33 ms.

## Release benchmark commands

Native 5120x1440 comparison:

```powershell
.\build-msvc-x64\Release\xreal-spatial-renderer.exe `
  --render-monitor-index 1 --capture-monitor-index 0 `
  --orientation-demo-static --panel-count 1 --panel-1-content desktop `
  --panel-1-capture-monitor-index 0 --desktop-resolution-policy native `
  --desktop-capture-cross-adapter cpu-fallback `
  --allow-desktop-capture-cpu-fallback --desktop-source-1-capture-fps 30 `
  --multi-panel-benchmark --multi-panel-benchmark-seconds 10 `
  --multi-panel-benchmark-warmup-seconds 2 `
  --multi-panel-benchmark-json capture-native.json
```

Full ultrawide downscaled to 1920x540:

```powershell
.\build-msvc-x64\Release\xreal-spatial-renderer.exe `
  --render-monitor-index 1 --capture-monitor-index 0 `
  --orientation-demo-static --panel-count 1 --panel-1-content desktop `
  --panel-1-capture-monitor-index 0 --desktop-source-1-crop full `
  --desktop-source-1-target-width 1920 --desktop-source-1-target-height 540 `
  --desktop-capture-cross-adapter cpu-fallback `
  --allow-desktop-capture-cpu-fallback --desktop-source-1-capture-fps 30 `
  --desktop-capture-diagnostics --desktop-capture-json-output capture-1920x540.json `
  --multi-panel-benchmark --multi-panel-benchmark-seconds 10 `
  --multi-panel-benchmark-warmup-seconds 2 `
  --multi-panel-benchmark-json render-1920x540.json
```

Centered 16:9 crop downscaled to 1920x1080:

```powershell
.\build-msvc-x64\Release\xreal-spatial-renderer.exe `
  --render-monitor-index 1 --capture-monitor-index 0 `
  --orientation-demo-static --panel-count 1 --panel-1-content desktop `
  --panel-1-capture-monitor-index 0 --desktop-source-1-crop center-16x9 `
  --desktop-source-1-target-width 1920 --desktop-source-1-target-height 1080 `
  --desktop-capture-cross-adapter cpu-fallback `
  --allow-desktop-capture-cpu-fallback --desktop-source-1-capture-fps 30 `
  --desktop-capture-diagnostics --desktop-capture-json-output capture-1920x1080.json `
  --multi-panel-benchmark --multi-panel-benchmark-seconds 10 `
  --multi-panel-benchmark-warmup-seconds 2 `
  --multi-panel-benchmark-json render-1920x1080.json
```

Two distinct physical outputs use panel/source 1 and 2 monitor selectors. A
three-source run adds panel/source 3. Example shape:

```powershell
.\build-msvc-x64\Release\xreal-spatial-renderer.exe `
  --render-monitor-index 1 --orientation-demo-static --panel-count 3 `
  --panel-1-content desktop --panel-1-capture-monitor-index 0 `
  --panel-2-content desktop --panel-2-capture-monitor-index 2 `
  --panel-3-content synthetic --performance-profile balanced `
  --desktop-source-1-target-width 1920 --desktop-source-1-target-height 1080 `
  --desktop-source-2-target-width 1920 --desktop-source-2-target-height 1080 `
  --desktop-capture-cross-adapter cpu-fallback `
  --allow-desktop-capture-cpu-fallback --multi-panel-benchmark `
  --multi-panel-benchmark-seconds 10 --multi-panel-benchmark-warmup-seconds 2 `
  --multi-panel-benchmark-json capture-two-source.json
```

Only one physical capture output exists on the validated Lenovo topology. In
the default `--capture-benchmark-sources 2|3` scene, source 1 is that real
Desktop Duplication output and the additional sources are explicitly labelled
`synthetic_gpu_scale_readback`. Each synthetic worker owns a static 3840x2160
GPU texture and goes through the exact `DesktopCaptureScaler`, staging ring,
CPU bridge and render upload path. It validates independent pipeline scaling,
readback, upload, scheduling and shutdown, but it does not reproduce Desktop
Duplication acquisition/metadata overhead and must never be reported as an
additional physical display.

For the static-timeout test, run the 1920x540 command, stop changing the source
desktop, and observe `wait_timeouts` increasing while `frame_availability`
becomes `active_unchanged` or `stale_but_valid`; the panel must remain visible.

For ten-minute hardware stability, replace demo mode with the validated IMU,
fusion, prediction and calibration arguments, set the benchmark duration to
600 seconds, keep diagnostics at 1 Hz, and verify flat memory/resource counts,
zero Flush, bounded bridges, and clean shutdown.

Three-pipeline mixed real/synthetic benchmark on the current Lenovo topology:

```powershell
.\build-msvc-x64\Release\xreal-spatial-renderer.exe `
  --render-monitor-index 1 --capture-monitor-index 0 `
  --capture-benchmark --capture-benchmark-sources 3 `
  --capture-benchmark-seconds 10 `
  --capture-benchmark-target-width 1920 `
  --capture-benchmark-target-height 1080 `
  --capture-benchmark-fps 30 `
  --capture-benchmark-json capture-three-pipeline.json `
  --desktop-capture-cross-adapter cpu-fallback `
  --allow-desktop-capture-cpu-fallback --no-vsync --target-fps 60
```
