# Native stereo tracking on Windows

The optional Windows x64 console program `xreal-native-stereo` measures six
degrees of freedom using the Air 2 Ultra's two grayscale cameras. It sends the
camera pose to the D3D11 renderer over loopback UDP (port 45871). Unity, the
vendor PC SDK, WSL2 and SteamVR are not required.

## Build (PowerShell, repository root)

Use Visual Studio 2022's x64 developer shell with CMake 3.24+. The tracker uses
the DirectShow capture, factory calibration loader and rectifier from a separate
[myXreal checkout](https://github.com/mazeasdamien/myXreal/tree/6844dd1440e5f072e600708d2b35574a1401f9e4).
The optional build compiles those files locally. Default builds do not need the
checkout or computer vision libraries. Obtain your own camera calibration file
for the **serial number of your glasses**. It must be the XREAL factory JSON
with the SLAM camera intrinsics and extrinsics. The repository's accelerometer
calibration and the upstream author's example profile cannot calibrate your pair.

```powershell
git clone https://github.com/mazeasdamien/myXreal.git ..\myXreal
git -C ..\myXreal checkout 6844dd1440e5f072e600708d2b35574a1401f9e4
```

Install the x64 C++ dependencies. For instance, with an existing classic-mode
vcpkg installation whose root is in `VCPKG_ROOT`:

```powershell
& "$env:VCPKG_ROOT\vcpkg.exe" install opencv4:x64-windows eigen3:x64-windows yaml-cpp:x64-windows nlohmann-json:x64-windows
cmake -S . -B out/native -G "Visual Studio 17 2022" -A x64 -DXREAL_BUILD_NATIVE_STEREO=ON -DMYXREAL_SOURCE_DIR=../myXreal "-DCMAKE_TOOLCHAIN_FILE=$env:VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake"
cmake --build out/native --config Release --parallel
ctest --test-dir out/native -C Release --output-on-failure
```

Alternatively point `OpenCV_DIR` and `CMAKE_PREFIX_PATH` at your own packages.
For the standalone OpenCV Windows distribution, add its x64 DLL directory to
PATH before running the tracker or tests.

## Try it with the glasses

1. Connect the Ultra to a USB-C port with data and display output. Close other
   camera consumers. In PowerShell run
   `.\out\native\src\native_stereo\Release\xreal-native-stereo.exe --probe`.
   The probe enumerates DirectShow cameras, requires a unique matching
   `UVC Camera 0` with USB VID/PID `3318:0426`, then reports received and
   accepted stereo pairs over five seconds. Zero accepted pairs means there
   is no usable stream. No calibration is required for this check.
2. Start the renderer in a window on the PC first:

   ```powershell
   .\out\native\Release\xreal-spatial-renderer.exe --native-stereo-pose --world-axes
   ```

3. In a second terminal start the tracker with your camera calibration:

   ```powershell
   .\out\native\src\native_stereo\Release\xreal-native-stereo.exe "C:\path\to\your\calibration.json"
   ```

4. Face a textured, well-lit scene. Once the tracker prints **Tracking**,
   move the glasses about 20 cm right without turning. The panel should shift
   left, approach when leaning forward, and return close to its original
   position when you move back. Rotate separately to check axis signs. Cover
   the cameras to check tracking loss, then uncover facing the original scene.
   Press **R** in the tracker console for a new origin, **Esc** to stop.
   Use `--render-monitor-index N --borderless` for the glasses once you have
   identified their monitor index in the renderer output.

## Pose and limits

Only increasing timestamps, correctly sized pairs and a driver-reported time
difference of at most 5 ms after compensation are accepted. The tracker matches
ORB features across the rectified eyes and triangulates their depth from
disparity and the calibrated baseline. It matches the reference 3D features to
the current left image and estimates camera motion with PnP and RANSAC. Missing
texture, inconsistent matches and zero disparity produce invalid tracking.
Loss retains the last reference for reacquisition. Recenter clears it.

The optical origin is the rectified left camera at initialization. Internally
+X points right, +Y down and +Z forward. The publisher changes basis to the
existing pose protocol (+X right, +Y up, +Z forward). The renderer applies the
inverse rigid transform to its view. With no valid samples it waits; the
existing receiver expires a silent stream after 100 ms.

This is stereo visual odometry. There is no inertial prediction, global map,
loop closure or camera-to-eye offset compensation. Drift and latency during
fast movements require measurement on hardware. The native display is
monoscopic. The synthetic tests cover metric translation, rotations, blank
images, zero disparity, recovery and recenter; they do not measure physical
accuracy or camera synchronization on a particular firmware.
