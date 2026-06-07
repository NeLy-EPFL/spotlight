# Unit tests for the recorder's hardware-independent logic, written with
# GoogleTest. Kept in their own subdirectory, mirroring comm_protocol/tests, and
# built only when RECORDER_BUILD_TESTS is enabled (see the parent CMakeLists.txt;
# it defaults to ON for a standalone recorder build).
#
# Only the translation units that compile and link without the camera/motion
# SDKs (Euresys, PCO, Zaber) are exercised here. Those listed below pull in
# nothing heavier than Qt SerialPort, OpenCV, yaml-cpp, spdlog and POSIX shared
# memory, so the tests stay buildable on a plain desktop checkout.

# Fetch GoogleTest. Reuse a target supplied by a parent project if one already
# exists (mirrors comm_protocol/tests and how the libraries handle ArduinoJson),
# otherwise pull a pinned release. This needs an internet connection on first
# configuration.
if(NOT TARGET GTest::gtest_main)
    include(FetchContent)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.15.2
    )
    # Don't let GoogleTest override the parent project's compiler/linker flags
    # (matters on Windows/MSVC; harmless elsewhere).
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
endif()

# The recorder has no intermediate library to link against (each binary compiles
# the shared sources directly), so the sources under test are compiled into the
# test executable as well.
add_executable(recorder_tests
    test_utils.cc
    test_calibration.cc
    test_recorder_config.cc
    test_muscle_roi.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/common/utils.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/common/calibration.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/common/recorder_config.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/common/muscle_recording.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/peripherals/muscle_camera.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/apps/shared_memory_utils.cc)

# RECORDER_INCLUDE_DIR is set by the parent CMakeLists.txt. The current source
# dir is added so the test files can include "test_helpers.h".
target_include_directories(recorder_tests PRIVATE
    ${RECORDER_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})

# Qt6::SerialPort is needed because utils.cc (pulled in transitively) enumerates
# serial ports; it also provides the Qt include paths used by utils.h.
target_link_libraries(recorder_tests PRIVATE
    Qt6::SerialPort
    ${OpenCV_LIBS}
    spdlog::spdlog
    yaml-cpp
    GTest::gtest_main
    pthread rt dl)

# Register each TEST() with CTest individually so `ctest` reports them by name.
include(GoogleTest)
gtest_discover_tests(recorder_tests)
