# Unit tests for the recorder's hardware-independent logic, written with
# GoogleTest. Kept in their own subdirectory, mirroring comm_protocol/tests, and
# built only when RECORDER_BUILD_TESTS is enabled (see the parent CMakeLists.txt;
# it defaults to ON for a standalone recorder build).
#
# These tests never open or talk to a physical device -- that is the rule for
# recorder/tests/. They MAY, however, include external SDK headers and link the
# SDK libraries: e.g. test_muscle_camera/test_muscle_roi exercise the
# hardware-independent MuscleTriggerTiming / MuscleCameraROI logic, which lives in
# muscle_camera.cc / muscle_recording.cc (one .cc per header), and those TUs pull
# in the PCO SDK. Linking the PCO SDK here is fine because no camera is ever
# constructed; the tests just need the SDK present, not a live camera.

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

# The recorder has no intermediate library to link against (each binary compiles
# the shared sources directly), so the sources under test are compiled into the
# test executable as well.
add_executable(recorder_tests
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
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/peripherals/muscle_camera.cc)

# muscle_camera.cc needs PCO_LINUX=1. Source-file properties are
# directory-scoped, so set them here (the parent set covers the app targets in
# recorder/CMakeLists.txt; pco_camera_sources uses target_compile_definitions).
set_source_files_properties(
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/peripherals/muscle_camera.cc
    PROPERTIES COMPILE_DEFINITIONS PCO_LINUX=1)

# RECORDER_INCLUDE_DIR is set by the parent CMakeLists.txt. The current source
# dir is added so the test files can include "test_helpers.h"; PCO_INCLUDE_DIRS
# is needed by muscle_camera.cc + the bundled PCO sources.
target_include_directories(recorder_tests PRIVATE
    ${RECORDER_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${PCO_INCLUDE_DIRS})

# Qt6::SerialPort is needed because utils.cc (pulled in transitively) enumerates
# serial ports; it also provides the Qt include paths used by utils.h. The PCO
# libs satisfy muscle_camera.cc's references (no camera is opened); the rpath
# keeps system Qt ahead of the bundled Qt in the PCO lib dir (see parent
# CMakeLists.txt).
target_link_libraries(recorder_tests PRIVATE
    Qt6::SerialPort
    ${OpenCV_LIBS}
    ${PCO_LIBS}
    pco_camera_sources
    spdlog::spdlog
    yaml-cpp
    GTest::gtest_main
    pthread rt dl)
set_target_properties(recorder_tests PROPERTIES
    INSTALL_RPATH "${QT_LIB_DIR};${PCO_LIB_DIR}")

# Register each TEST() with CTest individually so `ctest` reports them by name.
include(GoogleTest)
gtest_discover_tests(recorder_tests)

# --- Hardware tests ----------------------------------------------------------
# These DO drive a physical device (the behavior/JAI camera on the Euresys
# grabber), unlike everything above. They live in their own executable and are
# tagged with the CTest label "hardware" so the default test run can exclude them
# (`ctest -LE hardware`) and the rig can run only them (`ctest -L hardware`). See
# docs/setup/building.md. Building + test discovery do NOT need the camera (only
# running the test does), so this target builds anywhere the recorder builds.
#
# Deliberately does NOT link the PCO SDK: a behavior-camera failure here is then
# isolated from the muscle camera entirely. EGrabber is header-only (it dlopens
# its GenTL producer at runtime), so no Euresys library needs linking.
add_executable(recorder_hardware_tests
    test_behavior_camera_hardware.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/peripherals/behavior_camera.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/common/utils.cc)
target_include_directories(recorder_hardware_tests PRIVATE
    ${RECORDER_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(recorder_hardware_tests PRIVATE
    Qt6::SerialPort
    ${OpenCV_LIBS}
    spdlog::spdlog
    yaml-cpp
    GTest::gtest_main
    pthread rt dl)
set_target_properties(recorder_hardware_tests PROPERTIES
    INSTALL_RPATH "${QT_LIB_DIR}")
gtest_discover_tests(recorder_hardware_tests PROPERTIES LABELS "hardware")

# Hardware tests that need the PCO (muscle) camera, and the combined behavior +
# muscle bring-up test. This target links BOTH the PCO SDK and Euresys (via
# behavior_camera.cc / muscle_camera.cc), unlike recorder_hardware_tests above
# (which stays PCO-free to isolate the behavior camera). Same "hardware" label.
add_executable(recorder_hardware_tests_pco
    test_camera_hardware.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/peripherals/behavior_camera.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/peripherals/muscle_camera.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/common/recorder_config.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/common/utils.cc)
set_source_files_properties(
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/peripherals/muscle_camera.cc
    PROPERTIES COMPILE_DEFINITIONS PCO_LINUX=1)
target_include_directories(recorder_hardware_tests_pco PRIVATE
    ${RECORDER_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${PCO_INCLUDE_DIRS})
# Path to the shipped example profile config the tests read camera ROI/timing from.
target_compile_definitions(recorder_hardware_tests_pco PRIVATE
    "EXAMPLE_CONFIG_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}/../etc/profile_example/recorder_config.yaml\"")
target_link_libraries(recorder_hardware_tests_pco PRIVATE
    Qt6::SerialPort
    ${OpenCV_LIBS}
    ${PCO_LIBS}
    pco_camera_sources
    spdlog::spdlog
    yaml-cpp
    GTest::gtest_main
    pthread rt dl)
set_target_properties(recorder_hardware_tests_pco PROPERTIES
    INSTALL_RPATH "${QT_LIB_DIR};${PCO_LIB_DIR}")
gtest_discover_tests(recorder_hardware_tests_pco PROPERTIES LABELS "hardware")
