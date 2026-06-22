# Unit tests for comm_protocol, written with GoogleTest. Kept in their own
# subdirectory so they stay separate from the recorder's tests. Built only when
# testing is enabled (see the COMM_PROTOCOL_BUILD_TESTS option in the parent
# CMakeLists.txt).

# Fetch GoogleTest. Reuse a target supplied by a parent project if one already
# exists (mirrors how the library handles ArduinoJson), otherwise pull a pinned
# release. This needs an internet connection on first configuration.
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

add_executable(comm_protocol_tests
    test_protocol.cc
)

# Links the library under test plus GoogleTest's bundled main(). protocol.h and
# ArduinoJson come in transitively through comm_protocol's PUBLIC requirements.
target_link_libraries(comm_protocol_tests PRIVATE comm_protocol GTest::gtest_main)

# Register each TEST() with CTest individually so `ctest` reports them by name.
include(GoogleTest)
gtest_discover_tests(comm_protocol_tests)
