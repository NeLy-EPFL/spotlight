# recorder: the main recorder software for the recording computer (desktop,
# C++23). It can be built on its own:
#   cmake -S recorder -B recorder/build
#   cmake --build recorder/build
# or as part of the umbrella build at the repository root.
#
# NOTE: this wires the build against the new file layout (snake_case sources
# under src/, public headers included project-rooted from include/recorder/).
# The program logic is still being ported from the old codebase onto
# comm_protocol, so the targets are not expected to compile cleanly yet; the
# target/source composition mirrors the previous build (see git history's
# CMakeLists_old.txt) so it can be pruned during the refactor.

cmake_minimum_required(VERSION 3.20 FATAL_ERROR)

# True when this CMakeLists is the top of the build tree (standalone), false when
# pulled in via add_subdirectory() by the umbrella CMakeLists.txt at the root.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project("SpotlightRecorder"
        VERSION 0.3.0
        DESCRIPTION "Spotlight recorder software"
        LANGUAGES CXX)

    # Emit compile_commands.json for editor IntelliSense
    # (see .vscode/c_cpp_properties.json).
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
endif()

# The recorder is built against C++23 (see README.md).
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable debugging with frame pointers and add pthread support.
set(CMAKE_CXX_FLAGS_DEBUG
    "${CMAKE_CXX_FLAGS_DEBUG} -g3 -O0 -fno-omit-frame-pointer")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

# Pull in the shared protocol library. Guarded so that the umbrella build, which
# already adds comm_protocol before this directory, does not add it twice.
if(NOT TARGET comm_protocol)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../comm_protocol comm_protocol)
endif()

# Public headers live under include/recorder/, so consumers write e.g.
#   #include "recorder/common/recorder_config.h"
set(RECORDER_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

# --- External dependencies ---------------------------------------------------
find_package(Qt6 REQUIRED COMPONENTS Widgets SerialPort)
find_package(OpenCV REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp QUIET)

# Motion control (Zaber).
find_library(ZABER_MOTION_LIB NAMES zaber-motion REQUIRED)

# Frame grabber (Euresys).
set(EGRABBER_INCLUDE_DIR "/opt/euresys/egrabber/include")
include_directories(${EGRABBER_INCLUDE_DIR})

# PCO camera SDK (only the pco-camera-server target needs it). PCO_LINUX is not
# defined globally on purpose: it is incompatible with the rest of the codebase,
# so pco_camera_server.h defines it locally for the PCO headers only.
set(PCO_ROOT_DIR "/opt/pco/pco.cpp")
set(PCO_INCLUDE_DIR "${PCO_ROOT_DIR}/include")
set(PCO_LIB_DIR "${PCO_ROOT_DIR}/lib")
set(PCO_CAMERA_DIR "${PCO_ROOT_DIR}/pco.camera")
set(SDKLIB pco_sc2cam)
set(RECORDERLIB pco_recorder)
set(CONVERTLIB pco_convert)
set(XCITELIB etc_xcite)
set(PCO_CAMERA_SOURCES
    ${PCO_CAMERA_DIR}/cameraexception.cpp
    ${PCO_CAMERA_DIR}/image.cpp
    ${PCO_CAMERA_DIR}/camera.cpp
    ${PCO_CAMERA_DIR}/xcite.cpp
    ${PCO_CAMERA_DIR}/stdafx.cpp)

# --- Shared source groups ----------------------------------------------------
set(COMMON_SOURCES
    src/common/behavior_recording.cc
    src/common/calibration.cc
    src/common/cli.cc
    src/common/muscle_recording.cc
    src/common/recorder_config.cc
    src/common/tracking_control.cc
    src/common/utils.cc)

set(PERIPHERAL_SOURCES
    src/peripherals/arduino_communication.cc
    src/peripherals/behavior_camera.cc
    src/peripherals/experiment_protocol.cc
    src/peripherals/motion_control.cc
    src/peripherals/muscle_camera.cc)

# --- pco-camera-server -------------------------------------------------------
# Internal server that acquires muscle-camera frames and publishes them over
# shared memory. Built against the PCO SDK and kept separate from the rest.
add_executable(pco-camera-server
    src/apps/pco_camera_server_main.cc
    src/apps/shared_memory_utils.cc
    src/common/recorder_config.cc
    ${PCO_CAMERA_SOURCES})
target_include_directories(pco-camera-server PUBLIC
    ${RECORDER_INCLUDE_DIR} ${PCO_INCLUDE_DIR} ${PCO_CAMERA_DIR})
target_link_directories(pco-camera-server PUBLIC ${PCO_LIB_DIR})
target_link_libraries(pco-camera-server PUBLIC ${SDKLIB} ${RECORDERLIB} ${CONVERTLIB} ${XCITELIB})
target_link_libraries(pco-camera-server PUBLIC pthread rt dl)
target_link_libraries(pco-camera-server PUBLIC ${OpenCV_LIBS})
target_link_libraries(pco-camera-server PUBLIC spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>)
target_link_libraries(pco-camera-server PUBLIC yaml-cpp)
set_target_properties(pco-camera-server PROPERTIES
    INSTALL_RPATH "${PCO_LIB_DIR}"
    BUILD_WITH_INSTALL_RPATH TRUE)

# --- run-spotlight -----------------------------------------------------------
# The main recording application.
add_executable(run-spotlight
    ${PERIPHERAL_SOURCES}
    ${COMMON_SOURCES}
    src/apps/shared_memory_utils.cc
    src/apps/gui.cc
    src/apps/run_spotlight_main.cc)
target_include_directories(run-spotlight PRIVATE ${RECORDER_INCLUDE_DIR})
target_link_libraries(run-spotlight pthread rt dl)
target_link_libraries(run-spotlight Qt6::Widgets Qt6::SerialPort)
target_link_libraries(run-spotlight ${OpenCV_LIBS})
target_link_libraries(run-spotlight ${ZABER_MOTION_LIB})
target_link_libraries(run-spotlight spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>)
target_link_libraries(run-spotlight yaml-cpp)
target_link_libraries(run-spotlight comm_protocol)
set_target_properties(run-spotlight PROPERTIES AUTOMOC ON)

# --- align-cameras -----------------------------------------------------------
# Camera and excitation-beam alignment tool.
add_executable(align-cameras
    src/apps/align_cameras_main.cc
    src/apps/shared_memory_utils.cc
    src/peripherals/behavior_camera.cc
    src/peripherals/muscle_camera.cc
    src/peripherals/arduino_communication.cc
    src/peripherals/experiment_protocol.cc
    src/common/behavior_recording.cc
    src/common/muscle_recording.cc
    src/common/recorder_config.cc
    src/common/utils.cc
    src/common/cli.cc)
target_include_directories(align-cameras PRIVATE ${RECORDER_INCLUDE_DIR})
target_link_libraries(align-cameras PRIVATE pthread rt dl)
target_link_libraries(align-cameras PRIVATE Qt6::Widgets Qt6::SerialPort)
target_link_libraries(align-cameras PRIVATE ${OpenCV_LIBS})
target_link_libraries(align-cameras PRIVATE spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>)
target_link_libraries(align-cameras PRIVATE yaml-cpp)
target_link_libraries(align-cameras PRIVATE comm_protocol)
set_target_properties(align-cameras PROPERTIES AUTOMOC ON)

# --- run-arena-registration-scan ---------------------------------------------
# Automated mapping-board scan over the translation-stage positions.
find_library(DMTX_LIB NAMES dmtx REQUIRED)
add_executable(run-arena-registration-scan
    src/apps/run_arena_registration_scan_main.cc
    src/apps/shared_memory_utils.cc
    src/peripherals/behavior_camera.cc
    src/peripherals/motion_control.cc
    src/peripherals/arduino_communication.cc
    src/peripherals/experiment_protocol.cc
    src/common/cli.cc
    src/common/recorder_config.cc
    src/common/behavior_recording.cc
    src/common/utils.cc)
target_include_directories(run-arena-registration-scan PRIVATE ${RECORDER_INCLUDE_DIR})
target_link_libraries(run-arena-registration-scan Qt6::Widgets Qt6::SerialPort)
target_link_libraries(run-arena-registration-scan ${OpenCV_LIBS})
target_link_libraries(run-arena-registration-scan ${ZABER_MOTION_LIB})
target_link_libraries(run-arena-registration-scan spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>)
target_link_libraries(run-arena-registration-scan yaml-cpp)
target_link_libraries(run-arena-registration-scan ${DMTX_LIB})
target_link_libraries(run-arena-registration-scan comm_protocol)

# Install compiled binaries to the bin folder.
set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR})
install(
    TARGETS
        pco-camera-server
        run-spotlight
        align-cameras
        run-arena-registration-scan
    RUNTIME DESTINATION
        bin
)
