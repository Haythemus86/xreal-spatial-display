# Virtual-display performance strategy

## Existing path

Extended targets are expected to appear as ordinary active DXGI outputs:

```text
Windows compositor -> IddCx monitor -> Desktop Duplication
  -> existing GPU crop/downscale -> fixed staging ring
  -> bridge -> per-source upload texture/SRV -> panels -> one Present
```

The prototype must verify Desktop Duplication compatibility on real hardware.
It does not claim zero-copy. In cross-adapter mode the existing explicit CPU
fallback may still perform GPU-to-CPU and CPU-to-GPU transfers.

Three 1920x1080 BGRA sources at 30 FPS represent about 712 MiB/s before other
overheads if all require CPU readback. Existing scheduling remains in force:
the selected panel can request 30 FPS, other visible panels normally 15–20 FPS,
and hidden panels 0 FPS. Three sources are not forced to 60 FPS.

## Efficient mirrored mode

Mirrored count three creates one Windows target. All three panel definitions
carry the same stable source key. `PanelContentRegistry` therefore produces:

- one Desktop Duplication session;
- one GPU scale/readback path if needed;
- one bridge;
- one upload for each new source sequence;
- one SRV sampled by three panel draws.

It does not duplicate Intel uploads, CPU readback or source queues. The source
registry remains a fixed array of three entries and frame retention remains
bounded.

## IddCx surface handling

Windows provides compositor frames through the per-monitor IddCx swap chain.
The prototype creates its D3D11 device on the render-adapter LUID supplied by
IddCx, registers it once and acquires/releases compositor surfaces. The steady
loop performs no resource creation, CPU copy or `Flush`. It currently consumes
and completes those surfaces; it does not yet export them to the application.

`IPanelFrameSource` decouples future direct IddCx delivery from the renderer.
A future `VirtualDisplayFrameSource` can retain a bounded GPU-friendly stream
without creating a second renderer or changing panel upload policy. Actual
cross-process sharing, synchronization and copy stages must be profiled before
choosing that path.

## Benchmark plan

Use Release builds for conclusions. Measure one, two and three Extended
targets and three mirrored panels. Record render FPS, p50/p95/p99/max frame
time, Presents, per-source capture/upload FPS and age, memory/bandwidth,
dropped/repeated frames, resource creation and Flush counts. Also record the
UMDF host memory. Mirrored mode is accepted only if it is materially cheaper
than three Extended sources.

No current result is presented as a real-driver performance measurement; the
driver has not yet been built or installed on the reference machine.
