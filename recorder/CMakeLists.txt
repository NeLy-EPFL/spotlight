# recorder: the main recorder software for the recording computer (desktop,
# C++23). It can be built on its own:
#   cmake -S recorder -B recorder/build
#   cmake --build recorder/build
# or as part of the umbrella build at the repository root.
#
# NOTE: this wires the build against the new file layout (snake_case sources
# under src/, public headers included project-rooted from include/recorder/).
# All targets now compile and link, but the program logic is still being ported
# from the old codebase onto comm_protocol, so runtime behaviour is not yet
# validated. The target/source composition mirrors the previous build (see git
# history's CMakeLists_old.txt) so it can be pruned during the refactor.

cmake_minimum_required(VERSION 3.20 FATAL_ERROR)

# True when this CMakeLists is the top of the build tree (standalone), false when
# pulled in via add_subdirectory() by the umbrella CMakeLists.txt at the root.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project("SpotlightRecorder"
        VERSION 0.3.0
        DESCRIPTION "Spotlight recorder software"
        LANGUAGES CXX)

    # Default to a Debug build when the user does not pass
    # -DCMAKE_BUILD_TYPE=... (only meaningful for single-config generators
    # like Makefiles/Ninja).
    if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
        set(CMAKE_BUILD_TYPE Debug CACHE STRING
            "Build type (Debug, Release, RelWithDebInfo, MinSizeRel)" FORCE)
    endif()

    # Emit compile_commands.json for editor IntelliSense
    # (see .vscode/c_cpp_properties.json) and enable CTest for the unit tests.
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
    enable_testing()
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

# PCO camera SDK. The muscle (PCO) camera is driven in-process by muscle_camera.cc
# (no separate server / shared memory anymore), so the SDK is linked into
# run-spotlight and align-cameras. PCO_LINUX selects the PCO headers' Linux code
# paths; it is scoped PER FILE (see set_source_files_properties below) to exactly
# the TUs that include PCO headers -- the bundled SDK .cpp files and
# muscle_camera.cc -- rather than to the whole target. Two notes:
#   - It must expand to a non-empty token: some PCO headers test it with
#     `#elif PCO_LINUX` (e.g. etc_xcite_defs.h, etc_xcite_export.h), so an empty
#     define would expand to bare `#elif` with no expression and fail to compile.
#     The build passes PCO_LINUX=1.
#   - Selecting the Linux path makes pco_linux_defs.h inject Windows-compat
#     typedefs and macros (BOOL/BYTE/WORD/DWORD/HANDLE/..., FALSE/TRUE/MAKEWORD/
#     far/MAX_PATH/...) into the global namespace of every TU that pulls in a PCO
#     header. (This is the header's `#elif defined(PCO_LINUX)` branch; the
#     `_WIN32` `#include <windows.h>` branch is never taken on Linux, and with
#     neither defined the header hits its `#error`. The shims have no include
#     guard either.) These names were empirically verified not to clash with the
#     Qt/OpenCV/Euresys headers (the four stacks compile in one TU), and per-file
#     scoping keeps the shims out of gui.cc and the other Qt TUs.
#
# Runtime/link note (bundled Qt/ICU skew): /opt/pco/pco.cpp/lib also ships a
# bundled Qt 6.5.3 / ICU 56 (for PCO's own GUI tools), but the four camera libs
# linked below depend on NONE of it (verified: no Qt/ICU in their NEEDED). The
# binaries find system Qt 6.8.2 via LD_LIBRARY_PATH and the Qt lib dir placed
# FIRST on the rpath (ahead of PCO_LIB_DIR), so the bundled Qt is never resolved.
set(PCO_ROOT_DIR "/opt/pco/pco.cpp")
set(PCO_INCLUDE_DIR "${PCO_ROOT_DIR}/include")
set(PCO_LIB_DIR "${PCO_ROOT_DIR}/lib")
set(PCO_CAMERA_DIR "${PCO_ROOT_DIR}/pco.camera")
# Link the four camera libs by absolute path (rather than via link_directories),
# so the linker is unambiguous and we do not put PCO_LIB_DIR on the *link* search
# path. Runtime resolution of these libs and their libpco* siblings is handled by
# the rpath set per target below.
set(PCO_LIBS
    ${PCO_LIB_DIR}/libpco_sc2cam.so
    ${PCO_LIB_DIR}/libpco_recorder.so
    ${PCO_LIB_DIR}/libpco_convert.so
    ${PCO_LIB_DIR}/libetc_xcite.so)
set(PCO_CAMERA_SOURCES
    ${PCO_CAMERA_DIR}/cameraexception.cpp
    ${PCO_CAMERA_DIR}/image.cpp
    ${PCO_CAMERA_DIR}/camera.cpp
    ${PCO_CAMERA_DIR}/xcite.cpp
    ${PCO_CAMERA_DIR}/stdafx.cpp)
# Scope PCO_LINUX=1 to muscle_camera.cc (the only non-PCO-SDK TU that includes
# PCO headers). The bundled SDK sources get PCO_LINUX=1 through
# pco_camera_sources's target_compile_definitions (see below).
set_source_files_properties(
    src/peripherals/muscle_camera.cc
    PROPERTIES COMPILE_DEFINITIONS PCO_LINUX=1)

# The PCO headers (stdafx.h, camera.h, ...) live under PCO_CAMERA_DIR and
# PCO_INCLUDE_DIR. muscle_camera.cc and the bundled SDK sources need both.
set(PCO_INCLUDE_DIRS ${PCO_INCLUDE_DIR} ${PCO_CAMERA_DIR})

# PCO camera SDK source files compiled as a shared library to prevent WEAK ELF
# symbol interference with EGenTL's coaxlink.cti. When the PCO SDK sources are
# compiled into an executable (or a shared library without symbol controls),
# GCC emits WEAK inline-function symbols (_ZNSt9exceptionC1Ev and many
# std::string / std::chrono / etc. helpers) into the dynamic symbol table.
# When coaxlink.cti is later dlopen'd by EGenTL, its undefined references are
# resolved from the global symbol table; if the PCO-derived WEAK versions "win"
# over the libstdc++.so implementations, the ioctl sequence coaxlink issues
# diverges from the expected one and EGrabber<> construction fails with
# DriverNotAvailable. The version script (pco_camera_sources.map) restricts the
# .so's dynamic exports to the pco:: and etc:: namespace symbols only; all
# std::* inline helpers become local (hidden), so coaxlink.cti never resolves
# them from this library.
set(PCO_CAMERA_SOURCES_MAP ${CMAKE_CURRENT_SOURCE_DIR}/pco_camera_sources.map)
add_library(pco_camera_sources SHARED ${PCO_CAMERA_SOURCES})
target_compile_definitions(pco_camera_sources PRIVATE PCO_LINUX=1)
target_include_directories(pco_camera_sources PRIVATE ${PCO_INCLUDE_DIRS})
target_link_libraries(pco_camera_sources PRIVATE ${PCO_LIBS})
target_link_options(pco_camera_sources PRIVATE
    -Wl,--version-script,${PCO_CAMERA_SOURCES_MAP})
set_target_properties(pco_camera_sources PROPERTIES
    LINK_DEPENDS ${PCO_CAMERA_SOURCES_MAP}
    INSTALL_RPATH "${PCO_LIB_DIR}")

# Qt lib dir, placed first on the rpath so the bundled Qt in PCO_LIB_DIR is never
# resolved ahead of system Qt. Derived from the imported Qt6::Core target.
get_target_property(_qt6_core_location Qt6::Core LOCATION)
get_filename_component(QT_LIB_DIR "${_qt6_core_location}" DIRECTORY)

# --- Shared source groups ----------------------------------------------------
set(COMMON_SOURCES
    src/common/behavior_recording.cc
    src/common/calibration.cc
    src/common/cli.cc
    src/common/muscle_recording.cc
    src/common/recorder_config.cc
    src/common/loop_monitors.cc
    src/common/tracking_control.cc
    src/common/utils.cc)

set(PERIPHERAL_SOURCES
    src/peripherals/arduino_communication.cc
    src/peripherals/behavior_camera.cc
    src/peripherals/motion_control.cc
    src/peripherals/muscle_camera.cc)

# --- run-spotlight -----------------------------------------------------------
# The main recording application. The PCO muscle camera is driven in-process via
# muscle_camera.cc (part of PERIPHERAL_SOURCES) + the bundled PCO SDK sources.
add_executable(run-spotlight
    ${PERIPHERAL_SOURCES}
    ${COMMON_SOURCES}
    src/apps/gui.cc
    # gui.h declares the Q_OBJECT class MainGUIWindow. It must be listed as a
    # target source so AUTOMOC scans it and emits its meta-object code: the
    # header lives under include/, not next to gui.cc, so AUTOMOC's
    # same-directory header discovery misses it.
    include/recorder/apps/gui.h
    src/apps/run_spotlight_main.cc)
target_include_directories(run-spotlight PRIVATE
    ${RECORDER_INCLUDE_DIR} ${PCO_INCLUDE_DIRS})
target_link_libraries(run-spotlight pthread rt dl)
target_link_libraries(run-spotlight Qt6::Widgets Qt6::SerialPort)
target_link_libraries(run-spotlight ${OpenCV_LIBS})
target_link_libraries(run-spotlight ${ZABER_MOTION_LIB})
target_link_libraries(run-spotlight ${PCO_LIBS} pco_camera_sources)
target_link_libraries(run-spotlight spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>)
target_link_libraries(run-spotlight yaml-cpp)
target_link_libraries(run-spotlight comm_protocol)
# Installed binary: Qt lib dir first, then PCO lib dir (for the camera libs +
# their libpco* siblings); see the bundled-Qt/ICU note above. $ORIGIN/../lib
# locates libpco_camera_sources.so installed alongside (see install() below).
# The build-tree binary keeps CMake's auto rpath (which already includes
# PCO_LIB_DIR, since the PCO libs are linked by absolute path, plus Qt and
# /usr/local/lib).
set_target_properties(run-spotlight PROPERTIES
    AUTOMOC ON
    INSTALL_RPATH "${QT_LIB_DIR};${PCO_LIB_DIR};$ORIGIN/../lib")

# --- align-cameras -----------------------------------------------------------
# Camera and excitation-beam alignment tool.
add_executable(align-cameras
    src/apps/align_cameras_main.cc
    src/peripherals/behavior_camera.cc
    src/peripherals/muscle_camera.cc
    src/peripherals/arduino_communication.cc
    src/common/behavior_recording.cc
    src/common/muscle_recording.cc
    src/common/recorder_config.cc
    src/common/loop_monitors.cc
    src/common/utils.cc
    src/common/cli.cc)
target_include_directories(align-cameras PRIVATE
    ${RECORDER_INCLUDE_DIR} ${PCO_INCLUDE_DIRS})
target_link_libraries(align-cameras PRIVATE pthread rt dl)
target_link_libraries(align-cameras PRIVATE Qt6::Widgets Qt6::SerialPort)
target_link_libraries(align-cameras PRIVATE ${OpenCV_LIBS})
target_link_libraries(align-cameras PRIVATE ${PCO_LIBS} pco_camera_sources)
target_link_libraries(align-cameras PRIVATE spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>)
target_link_libraries(align-cameras PRIVATE yaml-cpp)
target_link_libraries(align-cameras PRIVATE comm_protocol)
set_target_properties(align-cameras PROPERTIES
    AUTOMOC ON
    INSTALL_RPATH "${QT_LIB_DIR};${PCO_LIB_DIR};$ORIGIN/../lib")

# --- run-arena-registration-scan ---------------------------------------------
# Automated mapping-board scan over the translation-stage positions.
find_library(DMTX_LIB NAMES dmtx REQUIRED)
add_executable(run-arena-registration-scan
    src/apps/run_arena_registration_scan_main.cc
    src/peripherals/behavior_camera.cc
    src/peripherals/motion_control.cc
    src/peripherals/arduino_communication.cc
    src/common/cli.cc
    src/common/recorder_config.cc
    src/common/behavior_recording.cc
    src/common/loop_monitors.cc
    src/common/utils.cc)
target_include_directories(run-arena-registration-scan PRIVATE ${RECORDER_INCLUDE_DIR})
target_link_libraries(run-arena-registration-scan Qt6::Widgets Qt6::SerialPort)
target_link_libraries(run-arena-registration-scan ${OpenCV_LIBS})
target_link_libraries(run-arena-registration-scan ${ZABER_MOTION_LIB})
target_link_libraries(run-arena-registration-scan spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>)
target_link_libraries(run-arena-registration-scan yaml-cpp)
target_link_libraries(run-arena-registration-scan ${DMTX_LIB})
target_link_libraries(run-arena-registration-scan comm_protocol)

# --- Unit tests --------------------------------------------------------------
# Desktop GoogleTest unit tests for the recorder's hardware-independent logic
# (see tests/). Like comm_protocol's tests, they build by default only in a
# standalone recorder configuration, so the umbrella build does not pay for them
# unless it opts in via -DRECORDER_BUILD_TESTS=ON.
set(RECORDER_STANDALONE OFF)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(RECORDER_STANDALONE ON)
endif()
option(RECORDER_BUILD_TESTS "Build recorder unit tests" ${RECORDER_STANDALONE})
if(RECORDER_BUILD_TESTS)
    add_subdirectory(tests)
endif()

# Install compiled binaries and the PCO camera sources shared library.
set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR})
install(
    TARGETS
        run-spotlight
        align-cameras
        run-arena-registration-scan
    RUNTIME DESTINATION bin
)
install(
    TARGETS pco_camera_sources
    LIBRARY DESTINATION lib
)
