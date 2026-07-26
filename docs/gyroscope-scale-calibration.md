# Gyroscope scale calibration

`xreal-gyro-scale-calibration` experimentally measures one gyroscope axis at a
time. It does not use an undocumented scale factor: it compares the integral of
measured raw angular rate with a physical rotation angle supplied by the user.

Gyroscope bias and scale solve different problems. Bias is the stationary
non-zero offset and is measured again before every trial. Scale converts a
bias-corrected raw rate into degrees or radians per second. Device timestamps
are required for this integration because host receive timing includes USB and
thread-scheduling jitter.

## Physical procedure

For each trial:

1. Place the glasses flat on a stable surface and keep them completely still.
2. Wait for runtime bias calibration to be accepted.
3. Prepare to rotate around the requested sensor axis.
4. After the countdown, make one slow, smooth rotation through the configured
   angle, such as 90, 180, or 360 degrees.
5. Avoid tilting around the other axes.
6. Stop at the reference orientation and keep the glasses still until the
   trial completes.

Six or more trials are recommended. Include clockwise and counter-clockwise
rotations, preferably using a marked turntable or printed angular reference.
A hand-performed 360-degree rotation is useful for experimentation but is not
laboratory-grade calibration.

Example for the Z axis:

```powershell
.\build-msvc-x64\Debug\xreal-gyro-scale-calibration.exe `
  --axis z `
  --expected-degrees 360 `
  --direction auto `
  --trials 6 `
  --gyro-calibrate-seconds 2 `
  --gyro-warmup-seconds 1 `
  --stillness-seconds 1 `
  --output gyro-scale-z.json `
  --csv-prefix gyro-scale-z
```

## Detection and rejection

Rotation begins when the selected bias-corrected axis reaches the start
threshold, 5000 raw units by default. It ends after the selected rate remains
below the stop threshold, 1500 raw units by default, for one second. Integration
uses the trapezoidal rule and device timestamp deltas; it does not assume a
fixed packet rate.

A trial is rejected for a missing or rejected bias, invalid timestamps,
unreasonable timestamp deltas, packet rate outside 800--1200 packets/s, the
wrong dominant axis, excessive cross-axis motion, a duration outside 0.5--20
seconds, a requested direction mismatch, insufficient samples, or acquisition
data loss. Thresholds are configurable because the defaults are conservative
diagnostic starting points based on the observed raw signal, not confirmed
hardware constants.

Accepted per-trial scale estimates are combined with a median. Values more than
20% from the initial median are excluded. The final result requires at least
four retained trials, a coefficient of variation no greater than 10%, and no
more than 10% disagreement between positive- and negative-direction medians.

## Output and interpretation

The JSON profile includes device identity, configuration, every accepted or
rejected trial, summary statistics, and the selected axis scale in:

- raw units per degree per second;
- raw units per radian per second;
- degrees per second per raw unit;
- radians per second per raw unit.

Only a finite, positive, valid scale can be applied. The profile calibrates one
axis only; no assumption is made that all axes have the same scale. Quaternion
integration, Euler angles, sensor fusion, drift correction, and pose estimation
are not implemented yet.
