# Gyroscope record-only analysis

The `--record-only` mode exists because manual rotations can violate the strict
interactive calibration rules before enough evidence has been captured. It
separates acquisition from interpretation: ordinary movement, unexpected
direction, cross-axis activity, and threshold crossings never stop a recording.
Analysis begins only after the requested capture duration has ended.

The existing interactive calibration remains available and unchanged when
`--record-only` is absent.

## Recommended procedure

1. Put the glasses on a stable surface and leave them completely still while
   runtime gyroscope bias is measured.
2. During the countdown, prepare a chair, turntable, or clearly marked flat
   surface.
3. When recording starts, perform one slow, controlled 90- or 360-degree
   rotation around the intended axis.
4. Avoid unnecessary tilt, return to the marked orientation, and leave the
   glasses still for the remainder of the recording.

Fifteen seconds normally leaves room for initial stillness, a smooth rotation,
and final stillness. Repeat the experiment in both directions. A manual
rotation is not laboratory-grade calibration, even with a marked reference.

```powershell
.\build-msvc-x64\Debug\xreal-gyro-scale-calibration.exe `
  --record-only `
  --record-seconds 15 `
  --gyro-calibrate-seconds 2 `
  --gyro-warmup-seconds 1 `
  --countdown-seconds 3 `
  --axis auto `
  --expected-degrees 360 `
  --direction auto `
  --csv-output gyro-recording.csv `
  --analysis-output gyro-recording-analysis.json `
  --print-rate 5
```

Omit `--expected-degrees` to collect and segment motion without calculating a
scale estimate. `--axis auto` examines all three axes and uses the dominant axis
of the selected segment. An explicit axis overrides that choice.

## Offline analysis

The analyzer subtracts the accepted runtime bias and uses trapezoidal
integration over device timestamps. It does not assume a constant packet rate.
Duplicate, decreasing, and unreasonably large timestamp deltas remain visible
in the CSV and are excluded from integration.

A motion segment starts when any corrected axis reaches the configured start
threshold. It ends after every axis remains below the stop threshold for the
configured stillness duration. Multiple segments are preserved. The best
segment is selected deterministically using:

- 50% axis dominance;
- 25% inverse cross-axis ratio;
- 15% stillness before and after motion;
- 10% duration quality.

The dominant axis is the axis containing the largest share of integrated
squared angular-rate energy. The cross-axis ratio compares the largest
off-axis peak with the dominant-axis peak; lower is cleaner.

When a known angle is supplied, the selected signed raw integral produces an
experimental scale. Its confidence combines dominance, cross-axis motion,
timestamp validity, duration, endpoint stillness, and agreement between the
selected segment and the complete recording. This estimate is unverified and
is never written into a production calibration profile automatically.

## Outputs and troubleshooting

The CSV contains every decoded sample, raw and bias-corrected gyro channels,
raw acceleration, device and host timestamps, sequence gaps, elapsed device
time, and validity flags. The JSON contains recording metadata, per-axis
statistics, every detected segment, selection details, warnings, and the
optional experimental scale.

If no segment is found, inspect the peak corrected values and lower
`--analysis-start-threshold`. If one long segment consumes the whole capture,
increase the stop threshold or stillness duration only after inspecting the
CSV. A high cross-axis ratio indicates tilt or rotation around several axes.
Timestamp warnings or queue drops indicate acquisition quality problems rather
than a failed physical rotation.

Quaternion integration, Euler angles, sensor fusion, pose prediction, and
rendering are not part of this milestone.
