cmake_minimum_required(VERSION 3.10)
project(SpotlightControlTest)

enable_testing()

# Find GoogleTest
set(GTEST_ROOT "/usr/local/")
set(GTEST_INCLUDE_DIR "${GTEST_ROOT}/include")
set(GTEST_LIB_DIR "${GTEST_ROOT}/lib")
find_library(GTEST_LIB gtest HINTS ${GTEST_LIB_DIR})
find_library(GTEST_MAIN_LIB gtest_main HINTS ${GTEST_LIB_DIR})

# Find OpenCV
find_package(OpenCV REQUIRED)

# Set up include directories for eGrabber and required compile options
set(EGRABBER_INCLUDE_DIR "/opt/euresys/egrabber/include")
include_directories(${EGRABBER_INCLUDE_DIR})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g -std=c++17")

# Find all source files
file(GLOB_RECURSE SOURCES "../src/*.cpp" "../src/*.h")
message(DEBUG "Source files added: ${SOURCES}")

# Add executables
# The `add_test(NAME ... COMMAND ...)` line registers the tests with CTest
add_executable(helloWorldTest helloWorldTest.cpp)
target_include_directories(helloWorldTest PRIVATE ${GTEST_INCLUDE_DIR})
target_link_libraries(helloWorldTest ${GTEST_LIB} ${GTEST_MAIN_LIB})
target_link_libraries(helloWorldTest ${OpenCV_LIBS})
add_test(NAME helloWorldTest COMMAND helloWorldTest)

add_executable(behaviorCameraTest ${SOURCES} testBehaviorCamera.cpp)
target_include_directories(behaviorCameraTest PRIVATE ${GTEST_INCLUDE_DIR})
target_link_libraries(behaviorCameraTest ${GTEST_LIB} ${GTEST_MAIN_LIB})
target_link_libraries(behaviorCameraTest ${OpenCV_LIBS})
add_test(NAME behaviorCameraTest COMMAND behaviorCameraTest)