# Gyroscope and accelerometer orientation fusion

The optional complementary filter combines high-frequency gyroscope prediction
with a slow gravity correction for pitch and roll. It is experimental and is
disabled unless `--fuse-gyro-accelerometer` is supplied. The existing
gyroscope-only mode remains available.

## Inputs and calibration

Gyroscope samples pass through the existing runtime bias calibration, explicit
scale profile, and sensor-to-body axis mapping. The current scalar candidate of
4090 raw units per degree per second remains experimental and unverified; it is
never hard-coded in the fusion filter.

Accelerometer conversion requires an explicit JSON calibration profile. Each
sensor axis is converted in double precision as
`g = (raw - offset_raw) / raw_units_per_g`, then mapped into the body frame.
Multiplication by the named standard gravity value 9.80665 produces m/s^2.
Invalid offsets, scales, identifiers, or axis mappings are rejected. There are
no built-in fallback calibration values.

## Frames and correction

The quaternion is a right-handed body-to-world rotation, ordered `wxyz`, using
the Hamilton product. World up is +Z and physical gravity points toward -Z.
An accelerometer at rest measures specific force opposite gravity, so its
normalized observation points toward world up. The expected body-frame
observation is obtained by rotating world +Z through the inverse orientation:

`up_body = inverse(q_body_to_world) * [0, 0, 1] * q_body_to_world`.

Every accepted device-timestamp interval is first integrated by the existing
gyroscope quaternion integrator. The shortest body-frame rotation from the
measured up vector to the predicted up vector is then applied on the right of
the prediction. Its fraction is time based:

`alpha = 1 - exp(-dt / correction_time_constant)`.

Alpha is multiplied by the configured gain and accelerometer confidence, then
bounded by the maximum correction rate. The algorithm never fuses Euler
angles. Gravity provides no heading reference, so yaw remains controlled by
the gyroscope and may drift. Anti-parallel observations are skipped because
their correction axis is ambiguous.

## Confidence and dynamic motion

The default confidence is full within 0.05 g of a 1 g magnitude, decreases
linearly to zero at 0.20 g, and is zero beyond that range. Near-zero,
non-finite, or invalid calibrated samples are rejected. Confidence smoothing
uses device time rather than sample count. The reusable estimator also supports
an optional jerk limit and confidence hysteresis.

Linear acceleration can have a magnitude close to 1 g while pointing in the
wrong direction, so magnitude confidence is not a complete motion detector.
During strong motion correction is reduced or stopped; gyroscope prediction
continues. Correction resumes progressively when confidence recovers. The
correction-rate bound prevents an abrupt tilt jump.

## Startup and recenter

`--fusion-startup identity` starts at identity and converges gradually.
`--fusion-startup gravity` waits for a confident acceleration observation,
initializes pitch and roll from gravity, and fixes initial yaw at zero. Dynamic
or invalid acceleration cannot initialize the gravity mode.

Recenter captures the current fused quaternion. Displayed orientation is
`inverse(reference) * fused`, while the absolute fused state and gravity
correction continue unchanged. Clearing recenter restores the absolute fused
orientation.

## Live hardware procedure

Use the accelerometer profile produced by the calibration analyzer:

```powershell
.\build-msvc-x64\Debug\xreal-imu-diagnostic.exe `
  --gyro-calibrate-seconds 2 `
  --gyro-warmup-seconds 1 `
  --apply-gyro-bias `
  --gyro-scale-raw-per-dps 4090 `
  --fuse-gyro-accelerometer `
  --fusion-mode complementary `
  --accelerometer-profile xreal-air2-ultra-calibration.json `
  --fusion-startup gravity `
  --accelerometer-correction-time-constant 2.0 `
  --accelerometer-max-correction-dps 10 `
  --fusion-output both `
  --print-accelerometer-physical `
  --print-fusion-diagnostics `
  --fusion-print-rate 10 `
  --fusion-json-output orientation-fusion.json `
  --recenter-after-seconds 10 `
  --duration 30
```

Keep the glasses still for seconds 0-4, place them on your head during seconds
4-7, then look straight ahead until recenter at second 10. Remain still until
second 15; look up and return during seconds 15-18, down and return during
18-21, tilt left and return during 21-24, tilt right and return during 24-27,
then remain still.

Pitch and roll should return closer to zero than in gyro-only mode. Confidence
may fall during movement and recover near 1 g. Expect no abrupt jump, NaN,
infinity, or rejection of normal approximately 1 ms samples. Yaw may still
drift.

If both `--integrate-gyro-orientation` and `--fuse-gyro-accelerometer` are
provided, the command is rejected; there is no silent precedence. Use
`--fusion-mode gyro-only` to select gyro-only processing through the fusion CLI
option family without requiring an accelerometer profile.

Magnetometer fusion, camera fusion, absolute yaw correction, pose prediction,
6DoF tracking, rendering, and capture are not implemented.
