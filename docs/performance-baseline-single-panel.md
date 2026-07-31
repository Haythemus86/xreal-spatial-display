# Single-panel performance baseline

Baseline captured before the multi-panel changes from commit `06afe45` on the
Lenovo hybrid-GPU test machine. The render output was `\\.\DISPLAY5`
(1920x1080, Intel Graphics); the desktop source was `\\.\DISPLAY1`
(5120x1440, NVIDIA RTX 5070 Laptop GPU). The renderer window was 1280x720,
hidden by smoke-test mode, with vsync requested.

These measurements compare builds only on this machine. They are not portable
GPU benchmarks. Hidden-window presentation did not synchronize at 60 Hz, so
the FPS values represent this diagnostic path rather than visible-display
refresh behavior.

| Mode | Build | Frames | Average FPS | Average frame | Maximum frame | Capture/upload |
|---|---:|---:|---:|---:|---:|---|
| Synthetic | Debug | 300 | 129.066 | 7.534 ms | 8.772 ms | none |
| Synthetic | Release | 300 | 128.379 | 7.695 ms | 13.010 ms | none |
| CPU checkerboard 640x360 | Debug | 300 | 129.127 | 7.552 ms | 8.737 ms | 1 upload, 299 repeats |
| CPU checkerboard 640x360 | Release | 300 | 128.842 | 7.648 ms | 9.354 ms | 1 upload, 299 repeats |
| Desktop CPU fallback 5120x1440 | Debug | 116 | 140.006 | 4.277 ms | 11.163 ms | 61 captures, 58 uploads |
| Desktop CPU fallback 5120x1440 | Release | 116 | 140.719 | 5.011 ms | 10.943 ms | 61 captures, 58 uploads |

The old renderer did not instrument p50/p95/p99, Present duration, CPU scene
update duration, draw-submission duration, draw calls, constant-buffer updates,
process memory, or D3D resource counts. Those values are therefore
**unavailable**, not zero. The desktop Release capture reported 108.9 capture
frames/s, 10.38 ms average source age, three repeated reads, two overwritten
bridge publications, and 1,798,963,200 CPU fallback bytes over the short run.

The baseline was collected with these exact command shapes (replace `Debug`
with `Release` for the Release row):

```powershell
.\build-msvc-x64\Debug\xreal-spatial-renderer.exe `
  --render-monitor-index 1 --orientation-demo-static --smoke-test `
  --smoke-test-frames 300 --panel-content synthetic --vsync

.\build-msvc-x64\Debug\xreal-spatial-renderer.exe `
  --render-monitor-index 1 --orientation-demo-static --smoke-test `
  --smoke-test-frames 300 --desktop-debug-checkerboard --vsync

.\build-msvc-x64\Debug\xreal-spatial-renderer.exe `
  --render-monitor-index 1 --capture-monitor-index 0 `
  --desktop-capture-smoke-test --desktop-capture-smoke-test-frames 60 `
  --panel-content desktop --desktop-capture-cross-adapter cpu-fallback `
  --allow-desktop-capture-cpu-fallback --vsync
```

Raw output was intentionally written below the untracked build directory and
was not added to Git. Fields unavailable in the pre-refactor executable cannot
be reconstructed retroactively from these runs.
