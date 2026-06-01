# Unit tests for comm_protocol. Kept in their own subdirectory so they stay
# separate from the recorder's tests. Built only when testing is enabled (see
# the COMM_PROTOCOL_BUILD_TESTS option in the parent CMakeLists.txt).

add_executable(comm_protocol_tests
    test_protocol.cc
)

# Links the library under test; protocol.h and ArduinoJson come in transitively
# through comm_protocol's PUBLIC usage requirements.
target_link_libraries(comm_protocol_tests PRIVATE comm_protocol)

add_test(NAME comm_protocol_tests COMMAND comm_protocol_tests)
