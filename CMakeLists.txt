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
    src/sensors/XrealDevice.cpp
    src/sensors/XrealHidConnection.cpp
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

foreach(target xreal-sensor-diagnostic xreal-interface-probe)
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
    xreal-platform-windows
    xreal-sensor-diagnostic
    xreal-interface-probe
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
endif()
