# IMU calibration capture

`xreal-imu-diagnostic` can record named calibration captures while preserving
the validated low-level IMU acquisition path. Calibration output remains in raw
device units. No gyroscope or accelerometer SI scale is assumed.

## Capture modes

Use `--calibration <name>` with names such as:

- `stationary-flat`
- `left-side`
- `right-side`
- `look-up`
- `look-down`
- `yaw-rotation`
- `pitch-rotation`
- `roll-rotation`

Names may contain letters, digits, hyphens, and underscores. A calibration CSV
is always written. Without an explicit `--csv`, its path is
`calibration-<name>.csv`.

Non-rotation captures use `--stationary-seconds`, which defaults to five
seconds, unless `--duration` is explicitly supplied. Rotation captures use the
regular `--duration`, whose default is ten seconds.

Example:

```powershell
.\build-msvc-x64\Debug\xreal-imu-diagnostic.exe `
  --calibration stationary-flat `
  --stationary-seconds 5 `
  --calibration-output stationary-flat.json
```

The stationary estimate uses a per-axis gyroscope standard-deviation threshold
of 5000 raw units. This is a diagnostic movement-rejection policy, not a sensor
scale. Rejected captures retain their raw CSV and JSON statistics for analysis,
but their bias must not be applied.

## Statistics and JSON

The online analysis uses double-precision Welford accumulation and records:

- mean, minimum, maximum, and population standard deviation for all six raw
  gyroscope and accelerometer channels;
- device and host timestamp delta statistics in nanoseconds;
- packet rate, sequence gaps, malformed packets, and diagnostic queue drops;
- raw gyroscope bias, raw acceleration mean and magnitude, sample count, and
  capture duration.

The optional `--calibration-output <file.json>` file contains raw values and
timing information only. It deliberately omits unconfirmed SI conversions.
Generated CSV and JSON captures are local diagnostic artifacts and should not be
committed.

## Offline accelerometer analysis

`xreal-calibration-analyzer` fits one axis-aligned ellipsoid from the raw
accelerometer means of all accepted stationary captures. It does not use the
capture names to infer sensor axes, so small pose-alignment errors do not turn
into hard-coded positive/negative axis pairs. Rejected stationary captures are
reported and excluded; pass the accepted `right-side-retry.json` explicitly in
place of a rejected `right-side.json`.

```powershell
.\build-msvc-x64\Debug\xreal-calibration-analyzer.exe `
  stationary-flat.json `
  stationary-inverted.json `
  left-side.json `
  right-side-retry.json `
  face-up.json `
  face-down.json `
  --output xreal-air2-ultra-calibration.json
```

The combined profile contains raw offsets, raw units per g for each axis,
per-capture corrected gravity magnitudes and radial residuals, aggregate
residuals, and fit-quality warnings. Six accepted captures exactly determine
the six axis-aligned model coefficients, so their residuals are not an
independent validation set. Additional diverse stationary orientations are
recommended for stronger fit validation.
