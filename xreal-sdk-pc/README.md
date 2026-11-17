# XREAL PC SDK 6DoF bridge

This Unity companion component is the first PC SDK integration step for XREAL
Air 2 Ultra. Add it to an XREAL PC SDK Unity project. It reads the head pose
supplied by XREAL's XR loader and publishes it over UDP to the native renderer
on `127.0.0.1:45871`.

## SDK setup

XREAL distributes the PC SDK as Unity package archives. Download the two
packages listed by XREAL's [PC SDK guide](https://github.com/dengxian-xreal/xreal.github.io/blob/main/docs/14_PC%20SDK%20(Beta).md):

- `com.xreal.xr`
- `com.xreal.xr.experimental`

Import both `.tgz` archives in Unity Package Manager, then add UniTask from the
Git URL specified in the guide. Enable **XREAL XR Loader** under **Project
Settings → XR Plug-in Management → Standalone** and leave **Initialize XR on
Startup** disabled. The guide currently targets Unity 2022.3 or Unity 6 and
lists the Air 2 Ultra as a 6DoF-capable PC device.

In the XREAL PC SDK project, use the standalone sample scene so the XREAL XR
display loader and embedding render feature are configured. Copy
`Assets/Scripts/XrealPoseUdpPublisher.cs` into that project and add it to the
sample's XR camera. Build the Windows standalone player and run it while the
glasses are connected. Start the native renderer with `--xreal-sdk-pose` to
consume the pose stream on port 45871.
The script initializes the configured loader, reads the tracked head pose, and
sends the newest valid pose as one line of JSON per UDP packet. Press `R` to
recenter the pose origin. Position is in metres; rotation is a Unity `xyzw`
quaternion; timestamp is a monotonic Unity clock value in seconds.

## Datagram format

```json
1,42,1,1.234,0.01,0.02,0.03,0,0,0,1
```

The fields are version, sequence, valid (`1` or `0`), sender time in seconds,
position X/Y/Z in metres, and rotation X/Y/Z/W. Invalid tracking is reported
with valid `0`; consumers must discard that
sample and should expire the last valid pose after a short timeout. The sender
binds no local port and sends only to loopback. It does not open HID interfaces
or decode camera data.

## Scope

This project validates the vendor tracking source and provides a narrow pose
transport for the native application. The C++ renderer now has an opt-in
`--xreal-sdk-pose` source that consumes these loopback datagrams and applies
both inverse head rotation and inverse head translation to its camera view.
The XREAL PC SDK owns the tracking implementation; the existing native D3D11
renderer and desktop capture pipeline remain in C++.

Unity positions use +Z forward. The native renderer maps translation to its
right-handed +X-right, +Y-up, -Z-forward coordinates and maps quaternion
components accordingly. Recenter is performed by the Unity publisher with `R`.

The XREAL package archives are proprietary SDK dependencies and are not stored
in this repository. Obtain them from XREAL and accept the SDK terms there.
