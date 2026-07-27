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

## Refined offline analysis

The original threshold-only segmentation could select nearly the complete
recording. Small residual bias, table or chair oscillation, placement motion,
and final settling then accumulated for many seconds and distorted the scale
estimate. The hardware-independent `xreal-gyro-recording-analyzer` reuses the
recorded device timestamps and accepted runtime bias, so an existing CSV can
be analyzed repeatedly without glasses or another capture.

The refined pipeline first finds a strict stationary window before motion and
requires a separate stationary window after it. Both windows are checked using
per-axis RMS and peak limits. Samples before the accepted pre-window and inside
the post-window are excluded. A missing pre-window rejects segmentation; a
missing post-window preserves the candidate with a warning and lower
confidence.

The pre-window also supplies per-axis mean, standard deviation, RMS, peak, and
median absolute deviation. The default dynamic deadband is the larger of 500
raw units and three stationary standard deviations. Soft deadband, the default,
preserves only the magnitude above that threshold; hard and disabled modes are
available. The original bias-corrected samples are never changed.

A 50 ms moving-RMS envelope is calculated over device time, including
irregular sample intervals. Movement must remain above the start threshold for
100 ms, and the stop condition must persist for 300 ms. Isolated spikes do not
start a segment. Low-energy edges are then trimmed conservatively while the
acceleration and deceleration phases remain part of the candidate.

Optional residual correction is applied only during offline analysis. `pre`
uses the pre-stillness mean; `linear` interpolates between the pre- and
post-stillness means. A large disagreement produces a warning. JSON preserves
raw bias-corrected, deadbanded, and residual-corrected integrals.

Candidate selection is deterministic. Its 100-point score weights verified
pre- and post-stillness (12 points each), dominant-axis ratio (18), cross-axis
ratio (12), duration (8), sustained motion (10), timestamp validity (8),
contained rotational energy (10), boundary thresholds (5), and residual
stability (5). Every candidate and score component is retained in JSON, along
with rough/refined boundaries and removed durations.

When an expected angle is supplied, the analyzer reports scale estimates from
raw bias-corrected, deadbanded, and residual-corrected integration. The
configured residual method is recommended when available. Disagreement between
methods reduces confidence. Every scale remains experimental and unverified;
the analyzer never promotes one into a production calibration profile.

Single-file reanalysis:

```powershell
.\build-msvc-x64\Debug\xreal-gyro-recording-analyzer.exe `
  --input gyro-recording-02.csv `
  --axis auto `
  --expected-degrees 360 `
  --deadband-mode soft `
  --residual-bias-mode linear `
  --output gyro-recording-analysis-02-refined.json
```

Repeated `--input` options enable batch analysis. Estimates outside the
configured percentage around the preliminary median are rejected, then the
median, mean, standard deviation, coefficient of variation, extrema, direction
agreement, and acceptance counts are recomputed from retained recordings.

```powershell
.\build-msvc-x64\Debug\xreal-gyro-recording-analyzer.exe `
  --input gyro-recording.csv `
  --input gyro-recording-02.csv `
  --input gyro-recording-03.csv `
  --input gyro-recording-04.csv `
  --axis z `
  --expected-degrees 360 `
  --deadband-mode soft `
  --residual-bias-mode linear `
  --batch-outlier-percent 20 `
  --output gyro-scale-z-batch.json
```

Manual 360-degree turns still have substantial uncertainty: the reference
angle, rotation axis, endpoint settling, surface slip, and operator timing are
not laboratory references. Compare multiple trials in both directions and do
not use these experimental estimates as a production scale without independent
validation.

## Outputs and troubleshooting

The CSV contains every decoded sample, raw and bias-corrected gyro channels,
raw acceleration, device and host timestamps, sequence gaps, elapsed device
time, and validity flags. The JSON contains recording metadata, per-axis
statistics, every detected segment, selection details, warnings, and the
optional experimental scale.

If no segment is found, inspect the stillness rejection reason, noise floor,
and peak corrected values before changing thresholds. A high cross-axis ratio
indicates tilt or rotation around several axes. Timestamp warnings or queue
drops indicate acquisition quality problems rather than a failed physical
rotation.

Quaternion integration, Euler angles, sensor fusion, pose prediction, and
rendering are not part of this milestone.
