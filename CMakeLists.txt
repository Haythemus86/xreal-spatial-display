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
    src/sensors/OrientationFusion.cpp
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
    src/platform/windows/HidDeviceCapabilities.cpp
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

foreach(target xreal-sensor-diagnostic xreal-interface-probe xreal-imu-diagnostic xreal-gyro-scale-calibration xreal-gyro-recording-analyzer xreal-gyro-scale-profile)
    add_custom_command(
        TARGET ${target}
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:hidapi::hidapi>
            $<TARGET_FILE_DIR:${target}>
    )
endforeach()

set(xreal_project_targets
    xreal-sensors
    xreal-interface-probe-support
    xreal-imu-diagnostic-support
    xreal-gyro-scale-calibration-support
    xreal-gyro-recording-analyzer-support
    xreal-platform-windows
    xreal-sensor-diagnostic
    xreal-interface-probe
    xreal-imu-diagnostic
    xreal-calibration-analyzer
    xreal-gyro-scale-calibration
    xreal-gyro-recording-analyzer
    xreal-gyro-scale-profile
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
endif()
