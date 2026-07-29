# Gyroscope and fusion comparison diagnostics

This experimental diagnostic runs gyro-only quaternion integration and the
existing complementary filter in parallel. Both paths receive the same
bias-corrected gyroscope sample, experimental scale, axis mapping and device
timestamp. Only the fused path receives accelerometer gravity correction.

## Frames and conventions

All quaternions use `wxyz` order, Hamilton multiplication, right-handed
coordinates and the established body-to-world convention. Four outputs are
kept distinct:

- `gyro_absolute`: gyro-only integration state;
- `gyro_relative`: displayed gyro state after recentering;
- `fused_absolute`: complementary-filter state;
- `fused_relative`: displayed fused state after recentering.

For each source, relative orientation is calculated as:

```text
relative = inverse(recenter_reference) * absolute
```

Recenter stores each source's current absolute quaternion as its reference. It
never overwrites either absolute state. Both references are captured on the
same sample and use the same policy, even though their quaternion values may
differ because gyro-only and fused states may already have diverged.

Earlier console names `fused_wxyz` and `gyro_prediction_wxyz` were ambiguous
because the former was relative and the latter absolute. Console output now
uses explicit source/frame names. Consumers should not compare absolute and
relative values.

## Difference metrics

Total angular distance uses the shortest quaternion distance and treats `q`
and `-q` as identical. Results are clamped to `[0, 180]` degrees. Tilt distance
compares predicted gravity directions, so pure yaw is excluded. ZYX Euler
differences are also emitted for diagnosis; they wrap at 180 degrees and can be
misleading near gimbal lock. Yaw remains uncorrected because no magnetometer or
external heading reference is used.

## Experiment and stationary detection

`--orientation-comparison-experiment` advances through
`stationary_before`, `motion`, `stationary_after`, then `complete`, using device
timestamps after runtime gyro calibration is accepted. Calibration and startup
are announced separately. Phase transitions include their device timestamps.

The diagnostic-only stationary detector requires all of:

- corrected gyro norm below the configured degrees-per-second threshold;
- accelerometer norm close to 1 g;
- high accelerometer confidence;
- the conditions sustained for the configured device-time interval.

It does not alter the filter or update gyro bias. Drift is measured only from
stationary-classified samples. Each source is measured independently in both
absolute and relative frames; unavailable results are explicit when there are
fewer than two stationary samples.

The recovery target is the most recent stationary fused absolute orientation
from `stationary_before`. During `stationary_after`, gyro-only and fused tilt
errors are measured against that same target. Threshold convergence requires a
continuous device-time interval below the threshold. Maximum, mean, RMS and
final tilt errors exclude yaw.

## Controlled support test

Use a rigid support with a repeatable center stop or alignment marks. Avoid an
uncontrolled rest on flexible temples. Follow the phase messages printed by
the program:

1. Keep the glasses fixed during warm-up and the 2-second gyro calibration.
2. Leave them at the center stop for `stationary_before`.
3. At the recenter message, keep the support fixed long enough to verify the
   relative identity while absolute orientation remains unchanged.
4. During `motion`, rotate about +30 and -30 degrees in pitch, returning to the
   center after each, then repeat for roll. Exact angles are not assumed.
5. Leave the glasses at the same center stop for `stationary_after`.

The suggested command is:

```powershell
.\build-msvc-x64\Debug\xreal-imu-diagnostic.exe `
  --gyro-calibrate-seconds 2 `
  --gyro-warmup-seconds 1 `
  --apply-gyro-bias `
  --gyro-scale-raw-per-dps 4090 `
  --accelerometer-profile xreal-air2-ultra-calibration.json `
  --compare-gyro-and-fusion `
  --orientation-comparison-experiment `
  --fusion-mode complementary `
  --fusion-startup gravity `
  --accelerometer-correction-time-constant 2.0 `
  --accelerometer-max-correction-dps 10 `
  --experiment-stationary-before-seconds 6 `
  --experiment-motion-seconds 12 `
  --experiment-stationary-after-seconds 15 `
  --experiment-recenter-seconds 10 `
  --stationary-gyro-threshold-dps 1.0 `
  --stationary-accel-deviation-g 0.05 `
  --stationary-min-duration-seconds 0.5 `
  --convergence-thresholds-degrees 5,2,1 `
  --convergence-sustain-seconds 0.5 `
  --comparison-output both `
  --comparison-print-rate 10 `
  --comparison-json-output orientation-comparison.json `
  --comparison-csv-output orientation-comparison.csv `
  --duration 42
```

## Output files

The CSV contains one row per selected 10 Hz output sample. It includes device
time, phase, sequence, stationary state/reason, corrected gyro, calibrated
acceleration, confidence and correction state, all four quaternions, all four
ZYX Euler diagnostics, and absolute/relative/tilt differences. Numeric output
uses the classic locale and textual fields are CSV-quoted.

The JSON summary records configuration and profile provenance, quaternion
conventions, separate recenter references, phase device-time ranges, drift in
all four frame/source combinations, separate gyro/fused convergence and
recovery metrics, final orientations/differences, and stream/filter counters.
Unavailable convergence times are `null`.

## Limitations

The 4090 raw-units-per-degree-per-second gyroscope scale and accelerometer axis
mapping remain experimental. This diagnostic adds no yaw correction, hidden
calibration fallback, online bias update, pose prediction, new fusion filter,
or position tracking. Physical angles are not inferred automatically.
