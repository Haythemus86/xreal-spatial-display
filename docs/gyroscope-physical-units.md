# Experimental gyroscope physical units

Gyroscope bias and gyroscope scale solve different problems. Bias is the
stationary zero-rate offset measured for the current session. Scale converts a
bias-corrected raw rate into an angular rate. Bias must be removed first.

This milestone uses a user-selected candidate of 4090 raw units per degree per
second on X, Y, and Z. The equal-axis assumption, the value itself, and the
resulting units are experimental and unverified. They are not official XREAL
values and are not assumed to apply to every device. Production conversion code
reads the value from a `GyroscopeScaleProfile`; it does not contain a hidden
4090 fallback.

For a profile axis whose scale is 4090:

```text
corrected_raw = raw - runtime_bias
gyro_dps = corrected_raw / 4090.0
gyro_rad_s = gyro_dps * pi / 180.0
raw_per_rad_s = raw_per_dps * 180.0 / pi
```

The radians scale is always derived in double precision. It is never loaded
from a separately rounded constant for conversion.

## Scale profile

Generate the initial profile with:

```powershell
.\build-msvc-x64\Debug\xreal-gyro-scale-profile.exe `
  --raw-per-dps 4090 `
  --experimental `
  --unverified `
  --output xreal-air2-ultra-gyro-scale-experimental.json
```

Schema version 1 records per-axis raw-per-degree and derived raw-per-radian
scales, inverse scales, axis validity, provenance, device identifiers,
experimental/verified flags, notes, and axis mapping metadata. X, Y, and Z may
have different scales later. Every enabled axis must have finite, strictly
positive scale and inverse values. Invalid profiles are rejected before any
conversion; partial conversion is not silently attempted.

The provisional mapping preserves sensor X/Y/Z identity and supports explicit
sign inversion. It is marked experimental and unverified. The observation that
sensor Z dominated a flat chair rotation suggests a possible yaw relationship
in that pose, but low-level sensor Z is not renamed to yaw.

## Live diagnostic

The live diagnostic requires an accepted runtime bias before printing physical
units. A JSON profile can replace the direct scalar option.

```powershell
.\build-msvc-x64\Debug\xreal-imu-diagnostic.exe `
  --gyro-calibrate-seconds 2 `
  --gyro-warmup-seconds 1 `
  --apply-gyro-bias `
  --gyro-scale-raw-per-dps 4090 `
  --print-gyro-degrees `
  --print-gyro-radians `
  --compare-q12-scale `
  --duration 10 `
  --print-rate 10
```

Output labels distinguish raw units, bias-corrected raw units, degrees per
second, and radians per second. Provenance and a visible warning accompany any
experimental or unverified scale.

## Offline comparison

Existing captures can compare the selected candidate against the power-of-two
Q12 hypothesis without hardware:

```powershell
.\build-msvc-x64\Debug\xreal-gyro-recording-analyzer.exe `
  --input gyro-recording-02.csv `
  --axis auto `
  --expected-degrees 360 `
  --fixed-scale-raw-per-dps 4090 `
  --compare-scale-raw-per-dps 4096 `
  --output gyro-recording-analysis-02-physical.json
```

The JSON reports reconstructed signed angles, radians, absolute/percentage
errors, relative result differences, and batch mean, median, RMS, and
direction-separated errors. It may identify which candidate fits the supplied
manual references more closely. Closeness to 4096 is not documentary proof of
Q12, and neither candidate is automatically marked verified or promoted into a
production profile.

Manual angle references, imperfect alignment, residual bias, segmentation,
and human motion remain important error sources. Quaternion integration, Euler
angles, sensor fusion, drift correction, and pose prediction are not
implemented in this milestone.
