# XREAL PC SDK 6DoF pose source

The optional `--xreal-sdk-pose` mode uses XREAL's Windows PC SDK Unity loader
for head tracking, then sends the newest pose to the native D3D11 renderer over
UDP loopback. The native app does not read the Air 2 Ultra cameras or run its
own SLAM implementation.

## Requirements and SDK setup

XREAL currently documents PC SDK support for Air 2 Ultra and Unity 2022.3 or
Unity 6. Use the XREAL standalone sample project and import the two SDK package
archives and UniTask dependency named in [XREAL's PC SDK guide](https://github.com/dengxian-xreal/xreal.github.io/blob/main/docs/14_PC%20SDK%20(Beta).md).
Enable **XREAL XR Loader** for Standalone and leave **Initialize XR on Startup**
disabled. Copy `xreal-sdk-pc/Assets/Scripts/XrealPoseUdpPublisher.cs` into the
project and add it to the XR camera in the sample scene, then build a Windows
standalone player.

The PC SDK archives are not included in this repository. Download them from
XREAL and accept its SDK terms. The Unity editor and Air 2 Ultra are required
to validate the vendor runtime and actual tracking.

## Run

Start the native app in pose-source mode, for example:

```powershell
.\build-msvc-x64\Debug\xreal-spatial-renderer.exe `
  --xreal-sdk-pose `
  --render-monitor-index 1 `
  --borderless `
  --world-axes
```

Then run the Unity standalone player. It sends one versioned CSV datagram at up
to 120 Hz to `127.0.0.1:45871`. The native renderer waits for valid samples and
falls back to a neutral view if packets stop for 100 ms. The socket binds only
to the loopback interface.

The packet fields are version, sequence, tracking-valid flag, sender time in
seconds, position X/Y/Z in metres, and quaternion X/Y/Z/W. The Unity publisher
captures a reference pose on its first valid sample; press `R` while the Unity
player has focus to recenter. The renderer converts Unity's +Z-forward frame to
the native right-handed +X-right, +Y-up, -Z-forward frame. It then applies the
inverse rotation and translation in the camera view matrix.

The mode is intended for a first positional-tracking validation. The existing
native renderer is monoscopic and does not implement per-eye distortion or
stereo composition; this integration adds positional camera motion but does
not yet provide a complete stereo XR display pipeline. XREAL's PC SDK runs in
the Unity player, so coexistence between its display output and the native
renderer still needs to be checked on the target PC and glasses.
