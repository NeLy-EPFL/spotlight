# recorder: the main recorder software for the recording computer (desktop,
# C++23). It can be built on its own:
#   cmake -S recorder -B recorder/build
#   cmake --build recorder/build
# or as part of the umbrella build at the repository root.
#
# NOTE: the recorder programs are not implemented in this tree yet, so this only
# wires up the build (and the shared comm_protocol dependency). Add the recorder
# targets (executables/libraries) below as the sources land.

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

# Pull in the shared protocol library. Guarded so that the umbrella build, which
# already adds comm_protocol before this directory, does not add it twice.
if(NOT TARGET comm_protocol)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../comm_protocol comm_protocol)
endif()

# Recorder targets go here, e.g.:
#   add_executable(run-spotlight src/run_spotlight.cc ...)
#   target_link_libraries(run-spotlight PRIVATE comm_protocol)
