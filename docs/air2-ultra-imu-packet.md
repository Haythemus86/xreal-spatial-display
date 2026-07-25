# XREAL Air 2 Ultra IMU packet interpretation

This production acquisition path is restricted to the hardware identity tested
on Windows 11 x64:

- VID `0x3318`
- PID `0x0426`
- HID interface number `2`
- 64-byte input reports

It sends the validated 10-byte activation report once, then reads on the same
HID handle. A first report beginning with `0xAA` is classified as an activation
response. Sensor reports must begin with the observed prefix/type bytes `01 02`.

## Upstream evidence

The interpretation was checked against MSmithDev's `AirAPI_Windows` repository
at commit `9643f5a41bcf0297e56cd321f70f244dd3babc58` on 2026-07-25. Its
`LICENSE.md` is the MIT License. No upstream source code is copied into this
project; the decoder uses independently written, bounds-safe little-endian
helpers.

The supporting upstream definitions are:

- [`AIR_2_ULTRA_PID`, timestamp/scalar declarations, and `air_sample`](https://github.com/MSmithDev/AirAPI_Windows/blob/9643f5a41bcf0297e56cd321f70f244dd3babc58/AirAPI_Windows.cpp#L15-L54)
- [`parse_report`](https://github.com/MSmithDev/AirAPI_Windows/blob/9643f5a41bcf0297e56cd321f70f244dd3babc58/AirAPI_Windows.cpp#L55-L194)
- [`process_ang_vel` and `process_accel`](https://github.com/MSmithDev/AirAPI_Windows/blob/9643f5a41bcf0297e56cd321f70f244dd3babc58/AirAPI_Windows.cpp#L196-L215)
- [Air 2 Ultra interface selection and activation report](https://github.com/MSmithDev/AirAPI_Windows/blob/9643f5a41bcf0297e56cd321f70f244dd3babc58/AirAPI_Windows.cpp#L417-L438)
- [upstream MIT license](https://github.com/MSmithDev/AirAPI_Windows/blob/9643f5a41bcf0297e56cd321f70f244dd3babc58/LICENSE.md)

The upstream parser requires 64 bytes, advances to byte 4 for an eight-byte
timestamp, skips six bytes before three gyroscope values, then skips another six
bytes before three accelerometer values. Each axis is decoded as a signed
24-bit, little-endian integer.

## Confirmed layout

| Bytes | Type | Endianness | Meaning | Evidence |
|---:|---|---|---|---|
| `0` | `uint8` | n/a | Sensor prefix `0x01`, or activation response prefix `0xAA` | Real Air 2 Ultra capture |
| `1` | `uint8` | n/a | Sensor type `0x02` | Real Air 2 Ultra capture |
| `4..11` | `uint64` | little-endian | Device timestamp in nanoseconds | `parse_report` and its clock comment |
| `18..20` | signed 24-bit | little-endian | Gyroscope X raw | `parse_report` |
| `21..23` | signed 24-bit | little-endian | Gyroscope Y raw | `parse_report` |
| `24..26` | signed 24-bit | little-endian | Gyroscope Z raw | `parse_report` |
| `33..35` | signed 24-bit | little-endian | Accelerometer X raw | `parse_report` |
| `36..38` | signed 24-bit | little-endian | Accelerometer Y raw | `parse_report` |
| `39..41` | signed 24-bit | little-endian | Accelerometer Z raw | `parse_report` |
| `63` | `uint8` | n/a | Wrapping packet sequence | Real Air 2 Ultra capture |

All other bytes remain undocumented and are intentionally ignored.

## Scaling status

`AirAPI_Windows` applies factors based on assumed ±2000 degrees/s and ±16 g
ranges, but the source explicitly describes these scale and bias corrections as
rough guesses. Those values are not sufficient evidence for confirmed SI
conversion. `ImuSample` therefore preserves the raw integers and leaves its SI
fields empty. The console diagnostic and CSV output do not invent converted
values.
