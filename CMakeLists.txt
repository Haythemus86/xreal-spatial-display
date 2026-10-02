cmake_minimum_required(VERSION 3.24)

project(
    XrealSpatialDisplay
    VERSION 0.1.0
    DESCRIPTION "Low-latency spatial display for XREAL glasses on Windows"
    LANGUAGES C CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT WIN32)
    message(FATAL_ERROR "XrealSpatialDisplay currently supports Windows only.")
endif()

set(HIDAPI_BUILD_HIDTEST OFF CACHE BOOL "" FORCE)
set(HIDAPI_BUILD_TESTS OFF CACHE BOOL "" FORCE)

add_subdirectory(third_party/hidapi)

add_library(
    xreal-virtual-display-protocol
    virtual-display/protocol/VirtualDisplayProtocol.cpp
    virtual-display/protocol/VirtualDisplayTypes.cpp
)

target_include_directories(
    xreal-virtual-display-protocol
    PUBLIC
        virtual-display
)

add_library(
    xreal-sensors
    src/sensors/AccelerometerCalibration.cpp
    src/sensors/AccelerometerPhysicalUnits.cpp
    src/sensors/GyroscopeBiasCalibration.cpp
    src/sensors/GyroscopeOfflineAnalysis.cpp
    src/sensors/GyroscopeOrientation.cpp
    src/sensors/GyroscopePhysicalUnits.cpp
    src/sensors/GyroscopeRecordingAnalysis.cpp
    src/sensors/GyroscopeRecordingCsv.cpp
    src/sensors/GyroscopeScaleCalibration.cpp
    src/sensors/ImuCalibration.cpp
    src/sensors/LiveOrientationPrediction.cpp
    src/sensors/OrientationFusion.cpp
    src/sensors/OrientationComparison.cpp
    src/sensors/OrientationPrediction.cpp
    src/sensors/OrientationPredictionEvaluation.cpp
    src/sensors/OrientationPredictionAnalysis.cpp
    src/sensors/XrealDevice.cpp
    src/sensors/XrealHidConnection.cpp
    src/sensors/XrealImuPacketDecoder.cpp
    src/sensors/XrealImuStream.cpp
)

target_include_directories(
    xreal-sensors
    PUBLIC
        src
)

target_link_libraries(
    xreal-sensors
    PUBLIC
        hidapi::hidapi
)

add_executable(
    xreal-sensor-diagnostic
    src/main.cpp
)

add_library(
    xreal-interface-probe-support
    src/diagnostics/InterfaceProbeSupport.cpp
)

add_library(
    xreal-imu-diagnostic-support
    src/diagnostics/ImuDiagnosticOptions.cpp
)

add_library(
    xreal-gyro-scale-calibration-support
    src/diagnostics/GyroScaleCalibrationOptions.cpp
)

add_library(
    xreal-gyro-recording-analyzer-support
    src/diagnostics/GyroRecordingAnalyzerOptions.cpp
)

target_include_directories(
    xreal-imu-diagnostic-support
    PUBLIC
        src
)

target_link_libraries(
    xreal-imu-diagnostic-support
    PUBLIC
        xreal-sensors
)

target_include_directories(
    xreal-gyro-scale-calibration-support
    PUBLIC
        src
)

target_link_libraries(
    xreal-gyro-scale-calibration-support
    PUBLIC
        xreal-sensors
)

target_include_directories(
    xreal-gyro-recording-analyzer-support
    PUBLIC
        src
)

target_link_libraries(
    xreal-gyro-recording-analyzer-support
    PUBLIC
        xreal-sensors
)

add_library(
    xreal-platform-windows
    src/platform/windows/DisplayTopology.cpp
    src/platform/windows/HidDeviceCapabilities.cpp
    src/platform/windows/ProcessMemory.cpp
    src/platform/windows/RenderWindow.cpp
)

target_include_directories(
    xreal-platform-windows
    PUBLIC
        src
)

target_link_libraries(
    xreal-platform-windows
    PRIVATE
        hid
        dxgi
        user32
        gdi32
        psapi
)

add_library(
    xreal-rendering-core
    src/rendering/Camera.cpp
    src/rendering/DesktopPanelDiagnostics.cpp
    src/rendering/DemoOrientationSource.cpp
    src/rendering/OrientationRenderBridge.cpp
    src/rendering/PanelContentRegistry.cpp
    src/rendering/PanelLayoutSerialization.cpp
    src/rendering/PanelPerformance.cpp
    src/rendering/PanelScene.cpp
    src/rendering/RenderDiagnostics.cpp
    src/rendering/RendererOptions.cpp
    src/rendering/RendererSummary.cpp
    src/rendering/RenderMath.cpp
    src/rendering/SensorOrientationService.cpp
    src/rendering/XrealSdkPoseUdpReceiver.cpp
)

add_library(
    xreal-desktop-capture
    src/capture/DesktopCaptureBridge.cpp
    src/capture/DesktopCaptureDiagnostics.cpp
    src/capture/DesktopCaptureFrame.cpp
    src/capture/DesktopCaptureManager.cpp
    src/capture/DesktopCaptureManagerFrameSource.cpp
    src/capture/DesktopCaptureOptions.cpp
    src/capture/DesktopCaptureScaler.cpp
    src/capture/DesktopCaptureScaling.cpp
    src/capture/DesktopSyntheticCapture.cpp
    src/capture/DesktopCaptureStatistics.cpp
    src/capture/DesktopDuplicationCapture.cpp
    src/capture/DesktopCpuFrame.cpp
    src/capture/DesktopFrameRetention.cpp
    src/capture/DesktopRenderStages.cpp
    src/capture/DesktopTextureLayout.cpp
)

target_include_directories(
    xreal-desktop-capture
    PUBLIC
        src
)

target_link_libraries(
    xreal-desktop-capture
    PUBLIC
        xreal-platform-windows
    PRIVATE
        d3d11
        d3dcompiler
        dxgi
)

target_include_directories(
    xreal-rendering-core
    PUBLIC
        src
)

target_link_libraries(
    xreal-rendering-core
    PUBLIC
        xreal-sensors
        xreal-desktop-capture
    PRIVATE
        ws2_32
)

add_library(
    xreal-virtual-display-service
    virtual-display/service/VirtualDisplayControlOptions.cpp
    virtual-display/service/VirtualDisplayService.cpp
    virtual-display/service/VirtualDisplayTopology.cpp
    virtual-display/service/VirtualDisplayViewModel.cpp
    virtual-display/service/WindowsVirtualDisplayTransport.cpp
)

target_include_directories(
    xreal-virtual-display-service
    PUBLIC
        src
        virtual-display
)

target_link_libraries(
    xreal-virtual-display-service
    PUBLIC
        xreal-virtual-display-protocol
        xreal-rendering-core
)

add_executable(
    xreal-virtual-display-control
    virtual-display/service/VirtualDisplayControlMain.cpp
)

target_link_libraries(
    xreal-virtual-display-control
    PRIVATE
        xreal-virtual-display-service
)

add_library(
    xreal-graphics-d3d11
    src/graphics/D3D11Renderer.cpp
)

target_include_directories(
    xreal-graphics-d3d11
    PUBLIC
        src
)

target_link_libraries(
    xreal-graphics-d3d11
    PRIVATE
        xreal-desktop-capture
        xreal-rendering-core
        d3d11
        dxgi
        d3dcompiler
)

target_include_directories(
    xreal-interface-probe-support
    PUBLIC
        src
)

add_executable(
    xreal-interface-probe
    src/diagnostics/ActiveInterfaceExperiment.cpp
    src/diagnostics/DirectImuExperiment.cpp
    src/diagnostics/InterfaceProbeMain.cpp
)

add_executable(
    xreal-imu-diagnostic
    src/diagnostics/ImuDiagnosticMain.cpp
)

add_executable(
    xreal-calibration-analyzer
    src/diagnostics/CalibrationAnalyzerMain.cpp
)

add_executable(
    xreal-gyro-scale-calibration
    src/diagnostics/GyroRecordOnlyMode.cpp
    src/diagnostics/GyroScaleCalibrationMain.cpp
)

add_executable(
    xreal-gyro-recording-analyzer
    src/diagnostics/GyroRecordingAnalyzerMain.cpp
)

add_executable(
    xreal-gyro-scale-profile
    src/diagnostics/GyroScaleProfileMain.cpp
)

add_executable(
    xreal-orientation-analyzer
    src/diagnostics/OrientationAnalyzerMain.cpp
)

add_executable(
    xreal-spatial-renderer
    src/rendering/RendererApplication.cpp
    src/rendering/RendererMain.cpp
    shaders/DesktopScale.hlsl
    shaders/Panel.hlsl
)

set_source_files_properties(
    shaders/DesktopScale.hlsl
    shaders/Panel.hlsl
    PROPERTIES HEADER_FILE_ONLY TRUE
)

target_link_libraries(
    xreal-sensor-diagnostic
    PRIVATE
        xreal-sensors
)

target_link_libraries(
    xreal-interface-probe
    PRIVATE
        xreal-interface-probe-support
        xreal-platform-windows
        xreal-sensors
)

target_link_libraries(
    xreal-imu-diagnostic
    PRIVATE
        xreal-imu-diagnostic-support
        xreal-sensors
)

target_link_libraries(
    xreal-calibration-analyzer
    PRIVATE
        xreal-sensors
)

target_link_libraries(
    xreal-gyro-scale-calibration
    PRIVATE
        xreal-gyro-scale-calibration-support
        xreal-sensors
)

target_link_libraries(
    xreal-gyro-recording-analyzer
    PRIVATE
        xreal-gyro-recording-analyzer-support
        xreal-sensors
)

target_link_libraries(
    xreal-gyro-scale-profile
    PRIVATE
        xreal-sensors
)

target_link_libraries(
    xreal-orientation-analyzer
    PRIVATE
        xreal-sensors
)

target_link_libraries(
    xreal-spatial-renderer
    PRIVATE
        xreal-graphics-d3d11
        xreal-desktop-capture
        xreal-platform-windows
        xreal-rendering-core
)

foreach(target xreal-sensor-diagnostic xreal-interface-probe xreal-imu-diagnostic xreal-gyro-scale-calibration xreal-gyro-recording-analyzer xreal-gyro-scale-profile xreal-spatial-renderer)
    add_custom_command(
        TARGET ${target}
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:hidapi::hidapi>
            $<TARGET_FILE_DIR:${target}>
    )
endforeach()

add_custom_command(
    TARGET xreal-spatial-renderer
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_CURRENT_SOURCE_DIR}/shaders/Panel.hlsl
        $<TARGET_FILE_DIR:xreal-spatial-renderer>/Panel.hlsl
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_CURRENT_SOURCE_DIR}/shaders/DesktopScale.hlsl
        $<TARGET_FILE_DIR:xreal-spatial-renderer>/DesktopScale.hlsl
)

set(xreal_project_targets
    xreal-virtual-display-protocol
    xreal-virtual-display-service
    xreal-sensors
    xreal-interface-probe-support
    xreal-imu-diagnostic-support
    xreal-gyro-scale-calibration-support
    xreal-gyro-recording-analyzer-support
    xreal-platform-windows
    xreal-desktop-capture
    xreal-rendering-core
    xreal-graphics-d3d11
    xreal-sensor-diagnostic
    xreal-interface-probe
    xreal-imu-diagnostic
    xreal-calibration-analyzer
    xreal-gyro-scale-calibration
    xreal-gyro-recording-analyzer
    xreal-gyro-scale-profile
    xreal-orientation-analyzer
    xreal-spatial-renderer
    xreal-virtual-display-control
)

foreach(target IN LISTS xreal_project_targets)
    target_compile_options(
        ${target}
        PRIVATE
            $<$<CXX_COMPILER_ID:MSVC>:/W4>
            $<$<CXX_COMPILER_ID:MSVC>:/permissive->
    )
endforeach()

include(CTest)

if(BUILD_TESTING)
    add_executable(
        xreal-desktop-capture-tests
        tests/DesktopCaptureTests.cpp
    )
    target_include_directories(
        xreal-desktop-capture-tests
        PRIVATE
            tests
    )
    target_link_libraries(
        xreal-desktop-capture-tests
        PRIVATE
            xreal-desktop-capture
    )
    target_compile_options(
        xreal-desktop-capture-tests
        PRIVATE
            $<$<CXX_COMPILER_ID:MSVC>:/W4>
            $<$<CXX_COMPILER_ID:MSVC>:/permissive->
    )
    add_test(NAME xreal-desktop-capture-tests COMMAND xreal-desktop-capture-tests)

    add_executable(
        xreal-display-topology-tests
        tests/DisplayTopologyTests.cpp
    )

    target_link_libraries(
        xreal-display-topology-tests
        PRIVATE
            xreal-platform-windows
    )

    target_compile_options(
        xreal-display-topology-tests
        PRIVATE
            $<$<CXX_COMPILER_ID:MSVC>:/W4>
            $<$<CXX_COMPILER_ID:MSVC>:/permissive->
    )

    add_test(NAME xreal-display-topology-tests COMMAND xreal-display-topology-tests)

    add_executable(
        xreal-interface-probe-tests
        tests/InterfaceProbeSupportTests.cpp
    )

    target_link_libraries(
        xreal-interface-probe-tests
        PRIVATE
            xreal-interface-probe-support
    )

    target_compile_options(
        xreal-interface-probe-tests
        PRIVATE
            $<$<CXX_COMPILER_ID:MSVC>:/W4>
            $<$<CXX_COMPILER_ID:MSVC>:/permissive->
    )

    add_test(NAME xreal-interface-probe-tests COMMAND xreal-interface-probe-tests)

    add_executable(
        xreal-imu-packet-decoder-tests
        tests/XrealImuPacketDecoderTests.cpp
    )

    target_link_libraries(
        xreal-imu-packet-decoder-tests
        PRIVATE
            xreal-sensors
    )

    target_compile_options(
        xreal-imu-packet-decoder-tests
        PRIVATE
            $<$<CXX_COMPILER_ID:MSVC>:/W4>
            $<$<CXX_COMPILER_ID:MSVC>:/permissive->
    )

    add_test(NAME xreal-imu-packet-decoder-tests COMMAND xreal-imu-packet-decoder-tests)

    add_executable(
        xreal-imu-calibration-tests
        tests/ImuCalibrationTests.cpp
    )

    target_link_libraries(
        xreal-imu-calibration-tests
        PRIVATE
            xreal-sensors
    )

    target_compile_options(
        xreal-imu-calibration-tests
        PRIVATE
            $<$<CXX_COMPILER_ID:MSVC>:/W4>
            $<$<CXX_COMPILER_ID:MSVC>:/permissive->
    )

    add_test(NAME xreal-imu-calibration-tests COMMAND xreal-imu-calibration-tests)

    add_executable(
        xreal-accelerometer-calibration-tests
        tests/AccelerometerCalibrationTests.cpp
    )

    target_link_libraries(
        xreal-accelerometer-calibration-tests
        PRIVATE
            xreal-sensors
    )

    target_compile_options(
        xreal-accelerometer-calibration-tests
        PRIVATE
            $<$<CXX_COMPILER_ID:MSVC>:/W4>
            $<$<CXX_COMPILER_ID:MSVC>:/permissive->
    )

    add_test(
        NAME xreal-accelerometer-calibration-tests
        COMMAND xreal-accelerometer-calibration-tests
    )

    add_executable(
        xreal-gyroscope-bias-calibration-tests
        tests/GyroscopeBiasCalibrationTests.cpp
    )

    target_link_libraries(
        xreal-gyroscope-bias-calibration-tests
        PRIVATE
            xreal-sensors
    )

    target_compile_options(
        xreal-gyroscope-bias-calibration-tests
        PRIVATE
            $<$<CXX_COMPILER_ID:MSVC>:/W4>
            $<$<CXX_COMPILER_ID:MSVC>:/permissive->
    )

    add_test(
        NAME xreal-gyroscope-bias-calibration-tests
        COMMAND xreal-gyroscope-bias-calibration-tests
    )

    add_executable(
        xreal-imu-diagnostic-options-tests
        tests/ImuDiagnosticOptionsTests.cpp
    )

    target_link_libraries(
        xreal-imu-diagnostic-options-tests
        PRIVATE
            xreal-imu-diagnostic-support
    )

    target_compile_options(
        xreal-imu-diagnostic-options-tests
        PRIVATE
            $<$<CXX_COMPILER_ID:MSVC>:/W4>
            $<$<CXX_COMPILER_ID:MSVC>:/permissive->
    )

    add_test(
        NAME xreal-imu-diagnostic-options-tests
        COMMAND xreal-imu-diagnostic-options-tests
    )

    add_executable(
        xreal-gyroscope-scale-calibration-tests
        tests/GyroscopeScaleCalibrationTests.cpp
    )

    target_link_libraries(
        xreal-gyroscope-scale-calibration-tests
        PRIVATE
            xreal-sensors
    )

    target_compile_options(
        xreal-gyroscope-scale-calibration-tests
        PRIVATE
            $<$<CXX_COMPILER_ID:MSVC>:/W4>
            $<$<CXX_COMPILER_ID:MSVC>:/permissive->
    )

    add_test(
        NAME xreal-gyroscope-scale-calibration-tests
        COMMAND xreal-gyroscope-scale-calibration-tests
    )

    add_executable(
        xreal-gyro-scale-calibration-options-tests
        tests/GyroScaleCalibrationOptionsTests.cpp
    )

    target_link_libraries(
        xreal-gyro-scale-calibration-options-tests
        PRIVATE
            xreal-gyro-scale-calibration-support
    )

    target_compile_options(
        xreal-gyro-scale-calibration-options-tests
        PRIVATE
            $<$<CXX_COMPILER_ID:MSVC>:/W4>
            $<$<CXX_COMPILER_ID:MSVC>:/permissive->
    )

    add_test(
        NAME xreal-gyro-scale-calibration-options-tests
        COMMAND xreal-gyro-scale-calibration-options-tests
    )

    add_executable(
        xreal-gyroscope-recording-analysis-tests
        tests/GyroscopeRecordingAnalysisTests.cpp
    )

    target_link_libraries(
        xreal-gyroscope-recording-analysis-tests
        PRIVATE
            xreal-sensors
    )

    target_compile_options(
        xreal-gyroscope-recording-analysis-tests
        PRIVATE
            $<$<CXX_COMPILER_ID:MSVC>:/W4>
            $<$<CXX_COMPILER_ID:MSVC>:/permissive->
    )

    add_test(
        NAME xreal-gyroscope-recording-analysis-tests
        COMMAND xreal-gyroscope-recording-analysis-tests
    )

    add_executable(
        xreal-gyroscope-offline-analysis-tests
        tests/GyroscopeOfflineAnalysisTests.cpp
    )

    target_link_libraries(
        xreal-gyroscope-offline-analysis-tests
        PRIVATE
            xreal-sensors
    )

    target_compile_options(
        xreal-gyroscope-offline-analysis-tests
        PRIVATE
            $<$<CXX_COMPILER_ID:MSVC>:/W4>
            $<$<CXX_COMPILER_ID:MSVC>:/permissive->
    )

    add_test(
        NAME xreal-gyroscope-offline-analysis-tests
        COMMAND xreal-gyroscope-offline-analysis-tests
    )

    add_executable(
        xreal-gyro-recording-analyzer-options-tests
        tests/GyroRecordingAnalyzerOptionsTests.cpp
    )

    target_link_libraries(
        xreal-gyro-recording-analyzer-options-tests
        PRIVATE
            xreal-gyro-recording-analyzer-support
    )

    target_compile_options(
        xreal-gyro-recording-analyzer-options-tests
        PRIVATE
            $<$<CXX_COMPILER_ID:MSVC>:/W4>
            $<$<CXX_COMPILER_ID:MSVC>:/permissive->
    )

    add_test(
        NAME xreal-gyro-recording-analyzer-options-tests
        COMMAND xreal-gyro-recording-analyzer-options-tests
    )

    add_executable(
        xreal-gyroscope-physical-units-tests
        tests/GyroscopePhysicalUnitsTests.cpp
    )

    target_link_libraries(
        xreal-gyroscope-physical-units-tests
        PRIVATE
            xreal-sensors
    )

    target_compile_options(
        xreal-gyroscope-physical-units-tests
        PRIVATE
            $<$<CXX_COMPILER_ID:MSVC>:/W4>
            $<$<CXX_COMPILER_ID:MSVC>:/permissive->
    )

    add_test(
        NAME xreal-gyroscope-physical-units-tests
        COMMAND xreal-gyroscope-physical-units-tests
    )

    add_executable(
        xreal-gyroscope-orientation-tests
        tests/GyroscopeOrientationTests.cpp
    )

    target_link_libraries(
        xreal-gyroscope-orientation-tests
        PRIVATE
            xreal-sensors
    )

    target_compile_options(
        xreal-gyroscope-orientation-tests
        PRIVATE
            $<$<CXX_COMPILER_ID:MSVC>:/W4>
            $<$<CXX_COMPILER_ID:MSVC>:/permissive->
    )

    add_test(
        NAME xreal-gyroscope-orientation-tests
        COMMAND xreal-gyroscope-orientation-tests
    )

    add_executable(
        xreal-orientation-fusion-tests
        tests/OrientationFusionTests.cpp
    )

    target_link_libraries(
        xreal-orientation-fusion-tests
        PRIVATE
            xreal-sensors
    )

    target_compile_options(
        xreal-orientation-fusion-tests
        PRIVATE
            $<$<CXX_COMPILER_ID:MSVC>:/W4>
            $<$<CXX_COMPILER_ID:MSVC>:/permissive->
    )

    add_test(
        NAME xreal-orientation-fusion-tests
        COMMAND xreal-orientation-fusion-tests
    )

    add_executable(
        xreal-orientation-comparison-tests
        tests/OrientationComparisonTests.cpp
    )

    target_link_libraries(
        xreal-orientation-comparison-tests
        PRIVATE
            xreal-sensors
    )

    target_compile_options(
        xreal-orientation-comparison-tests
        PRIVATE
            $<$<CXX_COMPILER_ID:MSVC>:/W4>
            $<$<CXX_COMPILER_ID:MSVC>:/permissive->
    )

    add_test(
        NAME xreal-orientation-comparison-tests
        COMMAND xreal-orientation-comparison-tests
    )

    add_executable(
        xreal-orientation-prediction-tests
        tests/OrientationPredictionTests.cpp
    )

    target_link_libraries(
        xreal-orientation-prediction-tests
        PRIVATE
            xreal-sensors
    )

    target_compile_options(
        xreal-orientation-prediction-tests
        PRIVATE
            $<$<CXX_COMPILER_ID:MSVC>:/W4>
            $<$<CXX_COMPILER_ID:MSVC>:/permissive->
    )

    add_test(
        NAME xreal-orientation-prediction-tests
        COMMAND xreal-orientation-prediction-tests
    )

    add_executable(
        xreal-orientation-prediction-analysis-tests
        tests/OrientationPredictionAnalysisTests.cpp
    )

    target_link_libraries(
        xreal-orientation-prediction-analysis-tests
        PRIVATE
            xreal-sensors
    )

    target_compile_options(
        xreal-orientation-prediction-analysis-tests
        PRIVATE
            $<$<CXX_COMPILER_ID:MSVC>:/W4>
            $<$<CXX_COMPILER_ID:MSVC>:/permissive->
    )

    add_test(
        NAME xreal-orientation-prediction-analysis-tests
        COMMAND xreal-orientation-prediction-analysis-tests
    )

    add_test(
        NAME xreal-orientation-analyzer-missing-file-test
        COMMAND xreal-orientation-analyzer
            --evaluate-orientation-prediction
            --prediction-input-csv ${CMAKE_CURRENT_BINARY_DIR}/orientation-prediction-missing.csv
            --prediction-horizons-ms 0,5,10
            --prediction-mode constant-velocity
            --prediction-evaluation-json ${CMAKE_CURRENT_BINARY_DIR}/unused-prediction.json
            --prediction-evaluation-csv ${CMAKE_CURRENT_BINARY_DIR}/unused-prediction.csv
    )
    set_tests_properties(
        xreal-orientation-analyzer-missing-file-test
        PROPERTIES WILL_FAIL TRUE
    )

    add_executable(
        xreal-rendering-math-tests
        tests/RenderingMathTests.cpp
    )
    target_link_libraries(
        xreal-rendering-math-tests
        PRIVATE
            xreal-rendering-core
    )
    target_compile_options(
        xreal-rendering-math-tests
        PRIVATE
            $<$<CXX_COMPILER_ID:MSVC>:/W4>
            $<$<CXX_COMPILER_ID:MSVC>:/permissive->
    )
    add_test(NAME xreal-rendering-math-tests COMMAND xreal-rendering-math-tests)

    add_executable(
        xreal-renderer-options-tests
        tests/RendererOptionsTests.cpp
    )
    target_link_libraries(
        xreal-renderer-options-tests
        PRIVATE
            xreal-rendering-core
    )
    target_compile_definitions(
        xreal-renderer-options-tests
        PRIVATE
            XREAL_PANEL_SHADER_PATH="${CMAKE_CURRENT_SOURCE_DIR}/shaders/Panel.hlsl"
    )
    target_compile_options(
        xreal-renderer-options-tests
        PRIVATE
            $<$<CXX_COMPILER_ID:MSVC>:/W4>
            $<$<CXX_COMPILER_ID:MSVC>:/permissive->
    )
    add_test(NAME xreal-renderer-options-tests COMMAND xreal-renderer-options-tests)

    add_executable(
        xreal-render-diagnostics-tests
        tests/RenderDiagnosticsTests.cpp
    )
    target_include_directories(
        xreal-render-diagnostics-tests
        PRIVATE
            tests
    )
    target_link_libraries(
        xreal-render-diagnostics-tests
        PRIVATE
            xreal-rendering-core
    )
    target_compile_options(
        xreal-render-diagnostics-tests
        PRIVATE
            $<$<CXX_COMPILER_ID:MSVC>:/W4>
            $<$<CXX_COMPILER_ID:MSVC>:/permissive->
    )
    add_test(NAME xreal-render-diagnostics-tests COMMAND xreal-render-diagnostics-tests)

    add_executable(
        xreal-multi-panel-tests
        tests/MultiPanelTests.cpp
    )
    target_link_libraries(
        xreal-multi-panel-tests
        PRIVATE
            xreal-rendering-core
    )
    target_compile_options(
        xreal-multi-panel-tests
        PRIVATE
            $<$<CXX_COMPILER_ID:MSVC>:/W4>
            $<$<CXX_COMPILER_ID:MSVC>:/permissive->
    )
    add_test(NAME xreal-multi-panel-tests COMMAND xreal-multi-panel-tests)

    add_executable(
        xreal-virtual-display-tests
        tests/VirtualDisplayTests.cpp
    )
    target_link_libraries(
        xreal-virtual-display-tests
        PRIVATE
            xreal-virtual-display-service
    )
    target_compile_options(
        xreal-virtual-display-tests
        PRIVATE
            $<$<CXX_COMPILER_ID:MSVC>:/W4>
            $<$<CXX_COMPILER_ID:MSVC>:/permissive->
    )
    add_test(NAME xreal-virtual-display-tests COMMAND xreal-virtual-display-tests)

    add_executable(
        xreal-virtual-display-driver-model-tests
        tests/VirtualDisplayDriverModelTests.cpp
    )
    target_link_libraries(
        xreal-virtual-display-driver-model-tests
        PRIVATE
            xreal-virtual-display-service
    )
    target_compile_options(
        xreal-virtual-display-driver-model-tests
        PRIVATE
            $<$<CXX_COMPILER_ID:MSVC>:/W4>
            $<$<CXX_COMPILER_ID:MSVC>:/permissive->
    )
    add_test(
        NAME xreal-virtual-display-driver-model-tests
        COMMAND xreal-virtual-display-driver-model-tests
    )
endif()
