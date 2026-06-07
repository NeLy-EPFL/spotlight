# comm_protocol: JSON serial protocol shared by the recorder (desktop, C++20+)
# and trigger_firmware (ESP32, C++20). It is normally pulled into the recorder
# build via add_subdirectory(), and the microcontroller build consumes the same
# sources through PlatformIO (see library.json). It can additionally be
# configured on its own to build and run its unit tests:
#   cmake -S comm_protocol -B comm_protocol/build
#   cmake --build comm_protocol/build
#   ctest --test-dir comm_protocol/build

cmake_minimum_required(VERSION 3.20 FATAL_ERROR)

# True when this CMakeLists is the top of the build tree (standalone), false
# when it is pulled in via add_subdirectory() by the recorder or another parent.
set(COMM_PROTOCOL_STANDALONE OFF)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(COMM_PROTOCOL_STANDALONE ON)
endif()

if(COMM_PROTOCOL_STANDALONE)
    project("CommProtocol"
        VERSION 0.4.0
        DESCRIPTION "JSON serial protocol shared by the recorder and firmware"
        LANGUAGES CXX)

    # Default to a Debug build when the user does not pass
    # -DCMAKE_BUILD_TYPE=... (only meaningful for single-config generators
    # like Makefiles/Ninja).
    if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
        set(CMAKE_BUILD_TYPE Debug CACHE STRING
            "Build type (Debug, Release, RelWithDebInfo, MinSizeRel)" FORCE)
    endif()

    # Emit compile_commands.json for editor IntelliSense (see
    # .vscode/c_cpp_properties.json) and enable CTest for the unit tests.
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
    enable_testing()
endif()

add_library(comm_protocol STATIC
    src/protocol.cc
)

# Public header lives at include/comm_protocol/protocol.h, so consumers write
#   #include <comm_protocol/protocol.h>
target_include_directories(comm_protocol PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# C++20 is the minimum; the recorder may compile against a newer standard.
target_compile_features(comm_protocol PUBLIC cxx_std_20)

# ArduinoJson is header-only on the desktop. Reuse a target supplied by the
# parent project if one already exists, otherwise fetch a matching v7 release.
if(NOT TARGET ArduinoJson)
    include(FetchContent)
    # EXCLUDE_FROM_ALL (a FetchContent feature added in CMake 3.28) stops
    # ArduinoJson's own install() rules from being added to a parent project's
    # `make install`. Without it, building under the recorder -- which points
    # CMAKE_INSTALL_PREFIX at its source dir -- scatters ArduinoJson's headers
    # and cmake config into the source tree. We only consume it as a header-only
    # link dependency, so its install rules are never wanted. On older CMake we
    # omit the keyword (it would error) and accept the default behavior.
    set(_arduinojson_exclude "")
    if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.28)
        set(_arduinojson_exclude EXCLUDE_FROM_ALL)
    endif()
    FetchContent_Declare(
        ArduinoJson
        GIT_REPOSITORY https://github.com/bblanchon/ArduinoJson.git
        GIT_TAG v7.2.1
        ${_arduinojson_exclude}
    )
    FetchContent_MakeAvailable(ArduinoJson)
    unset(_arduinojson_exclude)
endif()

target_link_libraries(comm_protocol PUBLIC ArduinoJson)

# Unit tests live in tests/ (kept separate from the recorder's tests). They are
# desktop-only and never pulled into the PlatformIO firmware build, which
# consumes the sources directly rather than through this CMakeLists. They build
# by default only in a standalone configuration, so a parent build (e.g. the
# recorder) does not pay for them unless it opts in.
option(COMM_PROTOCOL_BUILD_TESTS "Build comm_protocol unit tests"
    ${COMM_PROTOCOL_STANDALONE})
if(COMM_PROTOCOL_BUILD_TESTS)
    add_subdirectory(tests)
endif()
