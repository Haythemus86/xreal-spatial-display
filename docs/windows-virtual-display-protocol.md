# Virtual-display control protocol

The application and UMDF driver exchange one fixed-size binary request and one
fixed-size response through `\\.\XrealVirtualDisplay`. The shared structures
live in `virtual-display/protocol`. This is not a renderer ABI and contains no
JSON or pointers.

## Envelope

- Magic: `XDVP` (`0x58445650`).
- Version: `1`.
- IOCTL: `0x00222000` for the prototype
  (`CTL_CODE(FILE_DEVICE_UNKNOWN, 0x800, METHOD_BUFFERED, FILE_ANY_ACCESS)`).
- All fields are fixed-width integers or fixed-capacity character arrays.
- Each envelope declares its byte size and requires an exact input size.
- Maximum monitor count is three.

Supported commands are `GetCapabilities`, `GetCurrentConfiguration`,
`ApplyConfiguration`, `DisableVirtualDisplays`, `GetVirtualMonitors` and
`GetStatus`.

An Apply request contains enabled state, count, mode, width, height, refresh
and the client's observed generation. Count accepts only one through three;
Disable is a separate command. Mode accepts only Extended or Mirrored. The
prototype accepts only 1920x1080/60.

## Response

Every response contains acceptance, generation, requested and actual
configuration, actual monitor count, state, restart-required flag, error code,
bounded error text and up to three monitor records. A monitor record contains
logical/connector index, active state, mode, deterministic UUID, friendly name,
Windows device name and Windows stable monitor path when discovered.

Mirrored count three has requested count three but actual Windows count one.
This distinction is intentional: the three consumers exist in `PanelScene`,
not as redundant cloned Windows sources.

## Validation and failure behavior

Both ends validate magic, version, declared size, command, count, mode,
resolution and refresh. Malformed configuration is rejected without changing
the previous valid generation. The driver performs no arbitrary file or
network access and parses no renderer or JSON structures.

Error codes distinguish invalid protocol/count/mode, unsupported mode or
refresh, malformed buffers, unavailable/rejecting drivers and platform
failure. The user-mode transport includes the Win32 error code and readable
context when opening or calling the driver fails.

Changing the layout requires an explicit Apply or Disable command. Merely
starting the renderer or querying status cannot create monitors.
