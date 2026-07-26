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

## Runtime gyroscope bias calibration

The gyroscope has a stationary raw offset that can vary between devices and
with temperature. `xreal-imu-diagnostic` can measure that offset at startup and
optionally subtract it from later samples. The correction is strictly:

```text
corrected_raw = original_raw - measured_bias_raw
```

The original `ImuSample` remains unchanged. Corrected values stay in raw units:
the gyroscope scale is still unknown, so the application does not report
degrees per second or radians per second.

Keep the glasses completely still while calibration is running. Movement makes
the mean unsuitable as a bias and causes an explicit rejection with a non-zero
exit code.

```powershell
.\build-msvc-x64\Debug\xreal-imu-diagnostic.exe `
  --gyro-calibrate-seconds 2 `
  --gyro-warmup-seconds 1 `
  --apply-gyro-bias `
  --duration 10 `
  --print-rate 10 `
  --gyro-calibration-output gyro-bias.json
```

After acceptance, console samples contain both `gyro_raw` and
`gyro_corrected_raw_units`. `--apply-gyro-bias` requires a successful
calibration in the same execution; loading a previous bias profile is not yet
supported.

### Defaults and rejection policy

The startup calibrator defaults to:

- 2 seconds of calibration measured with device timestamps;
- 100 warm-up samples, excluded from the statistics;
- at least 1,500 calibration samples;
- maximum population standard deviation of 750 raw units on every axis;
- maximum observed range of 5,000 raw units on every axis;
- an acceptable device-timestamp packet rate of 800 to 1,200 packets/s.

The default warm-up is sample based: 100 samples are about 0.1 seconds at the
validated 1,000 Hz rate. `--gyro-warmup-seconds <seconds>` adds an explicit
device-timestamp duration. When both controls are present, collection begins
only after both the sample count and duration have elapsed. The suggested
hardware command uses one second to give the sensor additional startup time.

Stationary hardware captures generally showed standard deviations around 240
to 450 raw units. The 750-unit default provides practical noise margin while
being much stricter than the older 5,000-unit diagnostic capture threshold.
The 5,000-unit range limit tolerates ordinary stationary extrema but rejects
brief movement that a standard deviation alone could dilute. All values are
configurable:

```text
--gyro-max-stddev <raw-units>
--gyro-max-range <raw-units>
--gyro-min-samples <count>
--gyro-warmup-seconds <seconds>
```

An accepted result reports a valid `bias_raw`. A rejected result reports no
valid bias and identifies one of these causes: insufficient samples,
insufficient device-time duration, invalid or non-monotonic timestamps,
unexpected packet rate, excessive standard deviation, or excessive range.

Example summaries:

```text
Gyroscope bias calibration (raw units)
  Status: accepted
  Bias raw units: [-4700.5, 1220.0, -2380.5]

Gyroscope bias calibration (raw units)
  Status: rejected
Gyroscope bias calibration rejected: gyroscope raw range exceeded the configured threshold.
```

The numbers above illustrate the output format only; they are not defaults and
are never used as a device-specific fallback.

The JSON written by `--gyro-calibration-output` contains device metadata,
accepted/rejected status, rejection reason, raw bias when valid, per-axis mean,
standard deviation, minimum, maximum and range, device timestamp statistics,
packet rate, requested and measured calibration durations, and the full
configuration used. It contains no serial number and no unverified SI
conversion.

If calibration is rejected:

- place the glasses on a stable surface and avoid touching the cable;
- retry after allowing the device temperature to settle;
- check whether the rejection concerns movement, timestamps, sample count, or
  packet rate before changing a threshold;
- increase a threshold only when captured statistics justify it, rather than
  hard-coding a bias from another run or another device.

This milestone does not implement quaternion integration, orientation filters,
pose prediction, or any gyroscope scale conversion.
