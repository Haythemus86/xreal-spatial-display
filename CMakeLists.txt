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

add_executable(
    xreal-sensor-diagnostic
    src/main.cpp
    src/sensors/XrealDevice.cpp
)

target_include_directories(
    xreal-sensor-diagnostic
    PRIVATE
        src
)

target_link_libraries(
    xreal-sensor-diagnostic
    PRIVATE
        hidapi::hidapi
)

add_custom_command(
    TARGET xreal-sensor-diagnostic
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:hidapi::hidapi>
        $<TARGET_FILE_DIR:xreal-sensor-diagnostic>
)

target_compile_definitions(
    xreal-sensor-diagnostic
    PRIVATE
        WIN32_LEAN_AND_MEAN
        NOMINMAX
)

if(MSVC)
    target_compile_options(
        xreal-sensor-diagnostic
        PRIVATE
            /W4
            /permissive-
    )
endif()
