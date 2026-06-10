# Tests for the recorder, written with GoogleTest. Two executables:
#
#   recorder_tests_nohardware  (RECORDER_BUILD_TESTS, default ON in standalone)
#     Hardware-independent logic: calibration, config loading, image/metadata
#     utilities, muscle-camera ROI and trigger timing.  Buildable on any
#     desktop checkout; no physical device needed.
#
#   recorder_tests_hardware  (RECORDER_BUILD_HARDWARE_TESTS, default OFF)
#     Exercises the init-configure-destroy cycle of each peripheral (Euresys
#     behavior camera, PCO muscle camera, Zaber motion stages, Arduino trigger
#     controller) and an integrative test that brings all four up together and
#     exercises the shutdown sequence.  Requires the hardware to be powered and
#     connected; set SPOTLIGHT_PROFILE_DIR to a valid profile directory.
#
# Both targets are kept in this subdirectory.  The parent CMakeLists.txt
# enters this directory when either option is ON.

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
    # Don't register GoogleTest's install() rules. The recorder sets
    # CMAKE_INSTALL_PREFIX to its own source dir, so leaving these on makes
    # `make install` scatter gtest/gmock headers + cmake config into the source
    # tree (recorder/include, recorder/lib). We only ever link the targets.
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
endif()

include(GoogleTest)

# =============================================================================
# recorder_tests_nohardware
# =============================================================================
if(RECORDER_BUILD_TESTS)
    # The recorder has no intermediate library to link against (each binary
    # compiles the shared sources directly), so the sources under test are
    # compiled into the test executable as well.
    add_executable(recorder_tests_nohardware
        test_utils.cc
        test_calibration.cc
        test_recorder_config.cc
        test_muscle_roi.cc
        test_muscle_camera.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/common/utils.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/common/calibration.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/common/recorder_config.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/common/muscle_recording.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/common/loop_monitors.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/peripherals/muscle_camera.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/apps/shared_memory_utils.cc)

    # RECORDER_INCLUDE_DIR is set by the parent CMakeLists.txt. The current
    # source dir is added so the test files can include "test_helpers.h".
    target_include_directories(recorder_tests_nohardware PRIVATE
        ${RECORDER_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})

    # Qt6::SerialPort is needed because utils.cc (pulled in transitively)
    # enumerates serial ports; it also provides the Qt include paths used by
    # utils.h.
    target_link_libraries(recorder_tests_nohardware PRIVATE
        Qt6::SerialPort
        ${OpenCV_LIBS}
        spdlog::spdlog
        yaml-cpp
        GTest::gtest_main
        pthread rt dl)

    # Register each TEST() with CTest individually so `ctest` reports them by
    # name.  The "nohardware" label lets callers select or exclude them.
    gtest_discover_tests(recorder_tests_nohardware
        PROPERTIES LABELS nohardware)
endif()

# =============================================================================
# recorder_tests_hardware
# =============================================================================
if(RECORDER_BUILD_HARDWARE_TESTS)
    add_executable(recorder_tests_hardware
        test_hardware_behavior_camera.cc
        test_hardware_motion_control.cc
        test_hardware_arduino.cc
        test_hardware_muscle_camera.cc
        test_hardware_integration.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/common/utils.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/common/calibration.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/common/recorder_config.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/common/loop_monitors.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/common/tracking_control.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/peripherals/behavior_camera.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/peripherals/motion_control.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/peripherals/arduino_communication.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/peripherals/muscle_camera.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/apps/shared_memory_utils.cc)

    target_include_directories(recorder_tests_hardware PRIVATE
        ${RECORDER_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})

    # Qt6::SerialPort: arduino_communication + utils use Qt serial-port classes.
    # ZABER_MOTION_LIB: MotionControl and tracking_control use the Zaber SDK.
    target_link_libraries(recorder_tests_hardware PRIVATE
        Qt6::SerialPort
        ${OpenCV_LIBS}
        ${ZABER_MOTION_LIB}
        spdlog::spdlog
        yaml-cpp
        GTest::gtest_main
        comm_protocol
        pthread rt dl)

    # Tag every hardware test with the "hardware" label so they can be selected
    # or excluded via `ctest --label-regex hardware` /
    # `ctest --label-exclude hardware`.
    gtest_discover_tests(recorder_tests_hardware
        PROPERTIES LABELS hardware)
endif()
