cmake_minimum_required(VERSION 3.20 FATAL_ERROR)

project("SpotlightController"
    VERSION 0.3.0
    DESCRIPTION "Control software for the Spotlight system"
    LANGUAGES CXX)

# Emit build/compile_commands.json so editors/IntelliSense can resolve includes
# for the desktop targets (e.g. comm_protocol's tests, which are not part of the
# PlatformIO firmware build).
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Enables CTest registration for the whole project (e.g. comm_protocol's tests).
enable_testing()

add_subdirectory(comm_protocol)
add_subdirectory(recorder)
