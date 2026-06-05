cmake_minimum_required(VERSION 3.20)
project(SpotlightRecorderTest CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

enable_testing()

# Find GoogleTest
set(GTEST_ROOT "/usr/local/")
set(GTEST_INCLUDE_DIR "${GTEST_ROOT}/include")
set(GTEST_LIB_DIR "${GTEST_ROOT}/lib")
find_library(GTEST_LIB gtest HINTS ${GTEST_LIB_DIR})
find_library(GTEST_MAIN_LIB gtest_main HINTS ${GTEST_LIB_DIR})

# Find various libraries and packages
find_package(spdlog REQUIRED)
find_package(OpenCV REQUIRED)
find_library(ZABER_MOTION_LIB NAMES zaber-motion REQUIRED)

# Qt
find_package(Qt6 REQUIRED COMPONENTS Widgets SerialPort)

# Set up include directories for eGrabber and required compile options
set(EGRABBER_INCLUDE_DIR "/opt/euresys/egrabber/include")
include_directories(${EGRABBER_INCLUDE_DIR})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g")

# Recorder public headers, included project-rooted as
#   #include "recorder/common/utils.h"
set(RECORDER_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../include)

# Source files shared across the test executables.
set(COMMON_SOURCES
    ../src/common/utils.cc)

# Add executables
# The `add_test(NAME ... COMMAND ...)` line registers the tests with CTest
# Example test
add_executable(test_hello_world "test_hello_world.cc")
target_include_directories(test_hello_world PRIVATE ${GTEST_INCLUDE_DIR})
target_link_libraries(test_hello_world Qt6::Widgets Qt6::SerialPort)
target_link_libraries(test_hello_world ${GTEST_LIB} ${GTEST_MAIN_LIB})
target_link_libraries(test_hello_world ${OpenCV_LIBS})
target_link_libraries(test_hello_world spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>)
add_test(NAME test_hello_world COMMAND test_hello_world)

# Test behavior camera
add_executable(test_behavior_camera
    ${COMMON_SOURCES}
    ../src/peripherals/behavior_camera.cc
    "test_behavior_camera.cc")
target_include_directories(test_behavior_camera PRIVATE ${RECORDER_INCLUDE_DIR} ${GTEST_INCLUDE_DIR})
target_link_libraries(test_behavior_camera Qt6::Widgets Qt6::SerialPort)
target_link_libraries(test_behavior_camera ${GTEST_LIB} ${GTEST_MAIN_LIB})
target_link_libraries(test_behavior_camera ${OpenCV_LIBS})
target_link_libraries(test_behavior_camera spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>)
add_test(NAME test_behavior_camera COMMAND test_behavior_camera)

# Test translation stages
add_executable(test_motion_control
    ${COMMON_SOURCES}
    ../src/peripherals/motion_control.cc
    "test_motion_control.cc")
target_include_directories(test_motion_control PRIVATE ${RECORDER_INCLUDE_DIR} ${GTEST_INCLUDE_DIR})
target_link_libraries(test_motion_control Qt6::Widgets Qt6::SerialPort)
target_link_libraries(test_motion_control ${GTEST_LIB} ${GTEST_MAIN_LIB})
target_link_libraries(test_motion_control ${ZABER_MOTION_LIB})
target_link_libraries(test_motion_control ${OpenCV_LIBS})
target_link_libraries(test_motion_control spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>)
add_test(NAME test_motion_control COMMAND test_motion_control)

# Test the recorder-microcontroller message protocol
add_executable(test_arduino_message_protocol
    ${COMMON_SOURCES}
    ../src/peripherals/arduino_communication.cc
    "test_arduino_message_protocol.cc")
target_include_directories(test_arduino_message_protocol PRIVATE ${RECORDER_INCLUDE_DIR} ${GTEST_INCLUDE_DIR})
target_link_libraries(test_arduino_message_protocol Qt6::Widgets Qt6::SerialPort)
target_link_libraries(test_arduino_message_protocol ${GTEST_LIB} ${GTEST_MAIN_LIB})
target_link_libraries(test_arduino_message_protocol ${OpenCV_LIBS})
target_link_libraries(test_arduino_message_protocol spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>)
add_test(NAME test_arduino_message_protocol COMMAND test_arduino_message_protocol)
