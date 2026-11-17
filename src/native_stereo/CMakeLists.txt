set(MYXREAL_SOURCE_DIR "" CACHE PATH "Path to a local myXreal checkout")
if(NOT EXISTS "${MYXREAL_SOURCE_DIR}/include/myxreal/stereo_camera.h")
    message(FATAL_ERROR "Set MYXREAL_SOURCE_DIR to the myXreal checkout (see docs/native-stereo.md)")
endif()
find_package(OpenCV 4 REQUIRED COMPONENTS core calib3d features2d imgproc)
find_package(Eigen3 CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)

# Build only the capture/rectification translation units from the user's checkout.
add_library(xreal-stereo-camera SHARED "${MYXREAL_SOURCE_DIR}/src/driver/stereo_camera.cpp")
target_include_directories(xreal-stereo-camera PUBLIC "${MYXREAL_SOURCE_DIR}/include")
target_link_libraries(xreal-stereo-camera PRIVATE Eigen3::Eigen strmiids ole32 oleaut32)
add_library(xreal-stereo-odometry StereoOdometry.cpp)
target_include_directories(xreal-stereo-odometry PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" ${OpenCV_INCLUDE_DIRS})
target_link_libraries(xreal-stereo-odometry PUBLIC ${OpenCV_LIBS})
add_executable(xreal-native-stereo NativeStereoMain.cpp CameraIdentity.cpp
    "${MYXREAL_SOURCE_DIR}/src/core/calibration_loader.cpp"
    "${MYXREAL_SOURCE_DIR}/src/core/stereo_rectifier.cpp")
target_link_libraries(xreal-native-stereo PRIVATE ws2_32 strmiids ole32 oleaut32 xreal-stereo-camera xreal-stereo-odometry
    Eigen3::Eigen nlohmann_json::nlohmann_json yaml-cpp::yaml-cpp)
target_compile_definitions(xreal-native-stereo PRIVATE NOMINMAX)
if(BUILD_TESTING)
    add_executable(xreal-stereo-tests ../../tests/StereoOdometryTests.cpp)
    target_link_libraries(xreal-stereo-tests PRIVATE xreal-stereo-odometry)
    add_test(NAME xreal-stereo-tests COMMAND xreal-stereo-tests)
endif()
