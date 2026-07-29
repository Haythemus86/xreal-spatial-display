# Orientation pose prediction

## Purpose

The IMU reports the sensor pose at its device timestamp. A renderer needs an
estimate of the pose at a later presentation time. The experimental predictor
advances the current fused orientation by a short, explicit horizon. It does
not measure motion-to-photon latency and is not connected to rendering yet.

Prediction is disabled by default. It requires complementary fusion, an
accepted same-run gyroscope bias, an explicit gyroscope scale, and an explicit
accelerometer calibration profile.

## Quaternion and frame convention

Quaternions use Hamilton multiplication, component order `wxyz`, a
right-handed coordinate system, and represent body-to-world orientation.
Calibrated gyroscope velocity is mapped into the body frame before prediction.
For a body-frame rotation vector `r`, the predictor computes the quaternion
exponential map `delta = exp(r / 2)` and applies:

```text
predicted_absolute = measured_absolute * delta_body
```

The small-angle branch is normalized, as is every final prediction. Relative
orientation is always derived from the predicted absolute orientation using
the same recenter reference as the measured fused orientation. Recenter never
changes either absolute orientation. Delayed records carry a recenter
generation and are not evaluated across incompatible generations.

## Motion models

Constant velocity is the default:

```text
rotation_vector = angular_velocity * horizon
```

Constant acceleration is opt-in. Acceleration is estimated from filtered
angular-velocity differences and device timestamp deltas:

```text
angular_acceleration = delta(angular_velocity) / delta(device_time)
rotation_vector = angular_velocity * horizon
                + 0.5 * angular_acceleration * horizon^2
```

When acceleration history is unavailable or rejected, this mode explicitly
falls back to constant velocity. It never invents an initial acceleration.

Angular-velocity and angular-acceleration smoothing use time-based exponential
filters. Both are disabled unless their time constants are supplied. Duplicate
timestamps do not advance filter state. Decreasing timestamps reject the
prediction and reset temporal history. Excessive timestamp deltas reset the
history and fall back safely.

## Horizons and safety limits

The default horizon is 10 ms and the default maximum is 50 ms. Common offline
evaluation horizons are 0, 5, 10, 15, 20, and 30 ms. Zero horizon must reproduce
the measured pose.

Limits cover the horizon, angular speed, angular acceleration, acceleration
contribution, and final predicted angle. `reject` preserves the measured pose
and reports a reason. `clamp` must be selected explicitly and reports every
clamping event. A rejected prediction does not stop acquisition or mutate the
fusion filter.

## Delayed evaluation

For a prediction made at device time `T` with horizon `H`, the bounded evaluator
looks for a later fused absolute orientation near `T + H`. Matching uses device
timestamps only and a configurable tolerance. It compares:

- predicted pose at `T + H` against the later measured pose;
- the original measured pose at `T` against the same later pose as the
  no-prediction baseline;
- total quaternion angular error and tilt-only error separately.

Improvement is `baseline_error - predicted_error`. Percentage improvement is
unavailable when the baseline is zero. Unmatched, expired, and recenter-
incompatible records are counted. Evaluation never feeds back into prediction
or fusion.

## Live CSV

`--prediction-csv-output` writes one row per eligible fused IMU sample, not only
one row per console print. This preserves device-time resolution for offline
evaluation. The stable header contains timestamps, sequence, phase, recenter
generation, mode, requested/applied horizons, validity and reasons, clamp
state, raw and filtered angular velocity, optional acceleration, explicit
measured/predicted absolute and relative quaternions, Euler diagnostics,
predicted delta angle, and optional delayed-evaluation metrics. Unavailable
values are empty. Formatting uses the classic locale and never intentionally
emits NaN or infinity.

## Live JSON

`--prediction-json-output` writes schema version 1 with experimental status,
configuration, quaternion convention, profile and mapping provenance,
recenter state, aggregate predictor and delayed-evaluation statistics, and the
final measured and predicted poses. The current 4090 raw-units-per-degree-per-
second scale and sensor mappings remain experimental and are not marked as
verified.

## Offline multi-horizon analyzer

The analyzer consumes the live prediction CSV; raw HID packets are not needed.
It resets and reuses the production `OrientationPredictor` independently for
every requested horizon, matches future fused absolute orientations by device
time, and writes one CSV summary row per horizon plus a JSON report. Results
include mean, RMS, maximum, median, percentile 95, tilt errors, improvement,
and unmatched counts. The JSON calls its selection
`best_horizon_for_this_dataset`; it is not a universal optimum.

```powershell
.\build-msvc-x64\Debug\xreal-orientation-analyzer.exe `
  --evaluate-orientation-prediction `
  --prediction-input-csv orientation-prediction.csv `
  --prediction-horizons-ms 0,5,10,15,20,30 `
  --prediction-mode constant-velocity `
  --prediction-evaluation-tolerance-ms 2 `
  --prediction-evaluation-json orientation-prediction-evaluation.json `
  --prediction-evaluation-csv orientation-prediction-evaluation.csv
```

## First live experiment

Keep the glasses still through warm-up and bias calibration. Then wear them,
look forward for three seconds, perform slow yaw, faster yaw, pitch, and natural
combined movements, and remain still for the final five seconds. The prediction
should be nearly identical to measured orientation while stationary and lead it
during rotation. No discontinuity or non-finite value is expected.

Current limitations:

- the 4090 gyroscope scale is experimental;
- gyroscope and accelerometer axis mappings remain unverified;
- yaw has no absolute correction;
- there is no positional tracking;
- there is no renderer or desktop capture integration;
- delayed pose error is not a motion-to-photon measurement.
