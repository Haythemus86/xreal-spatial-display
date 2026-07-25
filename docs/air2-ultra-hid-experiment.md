# Air 2 Ultra HID activation experiment

This experiment is restricted to the hardware identity observed during testing:

- VID `0x3318`
- PID `0x0426`
- control interface `0`
- candidate input interfaces `1`, `2`, and `8`

These values are confirmed only for the tested XREAL Air 2 Ultra. Compatibility
with other models or firmware versions is unverified.

The experimental command uses a 64-byte vendor packet beginning with `0xFD`.
Its CRC is CRC-32/ISO-HDLC over the declared packet range beginning at byte 5.
HIDAPI prepends Report ID `0` when the packet is passed to `hid_write`.

The activation command is `0x0019` with data byte `0x01`. The tested glasses
returned a response containing the matching request ID and ACK status `0x04`.
This confirms command acknowledgement, but does not yet identify an IMU stream.
The command is never sent unless `--enable-imu` is explicitly supplied. No
disable command is assumed or implemented.

The active experiment opens interfaces `1`, `2`, and `8` and starts independent
readers before sending one activation command to interface `0`. It then keeps
all successfully opened handles alive and also reads interface `0`. No command
is sent to interfaces `1`, `2`, or `8`.

The packet layout and checksum behavior are based on the community-documented
XREAL Air protocol and remain experimental for Air 2 Ultra:

- <https://voidcomputing.hu/blog/worse-better-prettier/#mcu-protocol>
- <https://monado.pages.freedesktop.org/monado/xreal__air__hmd_8h.html>

Raw responses and stream packets must be inspected before any packet offsets or
sensor semantics are treated as confirmed.

## Direct interface 2 experiment

The independent `--enable-imu-direct` mode follows the observed Windows AirAPI
sequence. It opens exact interface `2` and passes exactly these 10 bytes to
`hid_write` without MCU framing or application-side padding:

```text
00 AA C5 D1 21 42 04 00 19 01
```

The first byte is HID Report ID `0`. The same handle is then read with a 64-byte
buffer. This mode does not send anything to interfaces `0`, `1`, or `8`, and it
does not decode returned packets.
