# Gyroscope-only orientation

`xreal-imu-diagnostic` can integrate the calibrated gyroscope stream into an
experimental orientation quaternion. Quaternions are used internally because
they compose rotations without the singularities of Euler-angle integration
and can be renormalized cheaply.

## Conventions

- Quaternion components are ordered `w, x, y, z`.
- Coordinates are right-handed; positive angular velocity follows the
  right-hand rule.
- The quaternion represents a body-to-world rotation.
- Angular velocity is expressed in the body frame. Each increment is composed
  as `current_orientation * body_delta` using the Hamilton product.
- Recentered output is
  `inverse(recenter_reference) * current_orientation`.

Raw sensor axes remain distinct from logical orientation axes. The configured
mapping supports permutation and sign inversion and must be a valid
permutation. Its `source`, `notes`, `experimental`, and `verified` metadata are
preserved in diagnostic JSON. The observation that sensor Z dominated a
flat-chair yaw test is provisional evidence, not a permanent low-level axis
name or a verified frame definition.

## Integration

The integrator consumes radians per second from the existing physical-units
layer. It does not contain a fallback scale and does not hard-code the current
4090 raw/(degree/s) candidate. That scale remains experimental, configurable,
unverified, and is not an official XREAL constant.

Delta time is computed only from consecutive device timestamps. The shared
timestamp policy accepts monotonically increasing values and a narrowly
identified unsigned 64-bit wraparound. Duplicate and decreasing timestamps are
not integrated. A configurable maximum rejects long gaps while preserving the
last valid quaternion.

For angular velocity `omega`, the update uses an exponential-map delta
quaternion with rotation magnitude `|omega| * dt`. Very small rotations use
the stable approximation `[1, omega * dt / 2]`. The result is normalized after
every applied sample. The first valid timestamp initializes timing without
applying a rotation.

Reset restores the identity quaternion and clears all timing and counters.
Recenter captures the current normalized orientation without discarding
absolute integration history. Clearing recenter restores absolute output.

## Diagnostic Euler angles

Yaw, pitch, and roll are derived only for display using intrinsic ZYX order:
Z yaw, Y pitch, then X roll. The `asin` input is clamped for numerical safety.
Euler output has the usual gimbal-lock ambiguity near pitch +/-90 degrees and
is never fed back into the quaternion integrator.

## Live diagnostic

Keep the glasses stationary during startup bias calibration, then move them
after calibration completes:

```powershell
.\build-msvc-x64\Debug\xreal-imu-diagnostic.exe `
  --gyro-calibrate-seconds 2 `
  --gyro-warmup-seconds 1 `
  --apply-gyro-bias `
  --gyro-scale-raw-per-dps 4090 `
  --integrate-gyro-orientation `
  --orientation-output both `
  --orientation-print-rate 20 `
  --duration 20
```

Optional controls include `--orientation-max-delta-ms`,
`--recenter-after-seconds`, and `--orientation-profile-output`. The JSON output
records the quaternion convention, mapping and scale provenance, final
orientation, optional diagnostic Euler angles, and sample counters.

This is gyro-only dead reckoning. Bias error accumulates, yaw drift is expected,
and pitch and roll are not gravity-corrected. The separate optional
[gyroscope/accelerometer fusion](orientation-fusion.md) layer can stabilize
tilt; magnetic or visual heading correction, pose prediction, and rendering
remain unimplemented.
