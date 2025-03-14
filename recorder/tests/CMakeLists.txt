cmake_minimum_required(VERSION 3.10)
project(SpotlightControlTest CXX)

set(CMAKE_CXX_STANDARD 17)
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
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g -std=c++17")

# Find common source files
file(GLOB COMMON_SOURCES
    "../src/utils.*"
    "../src/dataTypes.hpp"
    "../src/constants.hpp")
message(DEBUG "Source files added: ${SOURCES}")

# Add executables
# The `add_test(NAME ... COMMAND ...)` line registers the tests with CTest
# Example test
add_executable(testHelloWorld "testHelloWorld.cpp")
target_include_directories(testHelloWorld PRIVATE ${GTEST_INCLUDE_DIR})
target_link_libraries(testHelloWorld Qt6::Widgets Qt6::SerialPort)
target_link_libraries(testHelloWorld ${GTEST_LIB} ${GTEST_MAIN_LIB})
target_link_libraries(testHelloWorld ${OpenCV_LIBS})
target_link_libraries(testHelloWorld spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>)
add_test(NAME testHelloWorld COMMAND testHelloWorld)

# Test bahavior camera
file(GLOB BEHAVIOR_CAMERA_SOURCES "../src/peripherals/behaviorCamera.*")
add_executable(testBehaviorCamera
    ${COMMON_SOURCES}
    ${BEHAVIOR_CAMERA_SOURCES}
    "testBehaviorCamera.cpp")
target_include_directories(testBehaviorCamera PRIVATE ${GTEST_INCLUDE_DIR})
target_link_libraries(testBehaviorCamera Qt6::Widgets Qt6::SerialPort)
target_link_libraries(testBehaviorCamera ${GTEST_LIB} ${GTEST_MAIN_LIB})
target_link_libraries(testBehaviorCamera ${OpenCV_LIBS})
target_link_libraries(testBehaviorCamera spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>)
add_test(NAME testBehaviorCamera COMMAND behaviorCameraTest)

# Test translation stages
file(GLOB MOTION_CONTROL_SOURCES "../src/peripherals/motionControl.*")
add_executable(testMotionControl
    ${COMMON_SOURCES}
    ${MOTION_CONTROL_SOURCES}
    "testMotionControl.cpp")
target_include_directories(testMotionControl PRIVATE ${GTEST_INCLUDE_DIR})
target_link_libraries(testMotionControl Qt6::Widgets Qt6::SerialPort)
target_link_libraries(testMotionControl ${GTEST_LIB} ${GTEST_MAIN_LIB})
target_link_libraries(testMotionControl ${ZABER_MOTION_LIB})
target_link_libraries(testMotionControl ${OpenCV_LIBS})
target_link_libraries(testMotionControl spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>)
add_test(NAME testMotionControl COMMAND testMotionControl)

# Test camera acquisition config
file(GLOB MOTION_CONTROL_SOURCES "../src/arduinoMessageProtocol.*")
add_executable(testArduinoMessageProtocol
    ${COMMON_SOURCES}
    ${MOTION_CONTROL_SOURCES}
    "testArduinoMessageProtocol.cpp")
target_include_directories(testArduinoMessageProtocol PRIVATE ${GTEST_INCLUDE_DIR})
target_link_libraries(testArduinoMessageProtocol Qt6::Widgets Qt6::SerialPort)
target_link_libraries(testArduinoMessageProtocol ${GTEST_LIB} ${GTEST_MAIN_LIB})
target_link_libraries(testArduinoMessageProtocol ${OpenCV_LIBS})
target_link_libraries(testArduinoMessageProtocol spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>)
add_test(NAME testArduinoMessageProtocol COMMAND testArduinoMessageProtocol)

# Test parameter loading
file(GLOB PARAMETERS_SOURCES "../src/parameters.*")
add_executable(testParameters
    ${COMMON_SOURCES}
    ${MOTION_CONTROL_SOURCES}
    "testParameters.cpp")
target_include_directories(testParameters PRIVATE ${GTEST_INCLUDE_DIR})
target_link_libraries(testParameters ${GTEST_LIB} ${GTEST_MAIN_LIB})
target_link_libraries(testParameters spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>)
add_test(NAME testParameters COMMAND testParameters)