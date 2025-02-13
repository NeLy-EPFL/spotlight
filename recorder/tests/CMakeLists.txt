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
add_executable(testHelloWorld testHelloWorld.cpp)
target_include_directories(testHelloWorld PRIVATE ${GTEST_INCLUDE_DIR})
target_link_libraries(testHelloWorld ${GTEST_LIB} ${GTEST_MAIN_LIB})
target_link_libraries(testHelloWorld ${OpenCV_LIBS})
add_test(NAME testHelloWorld COMMAND testHelloWorld)

add_executable(testBehaviorCamera ${SOURCES} testBehaviorCamera.cpp)
target_include_directories(testBehaviorCamera PRIVATE ${GTEST_INCLUDE_DIR})
target_link_libraries(testBehaviorCamera ${GTEST_LIB} ${GTEST_MAIN_LIB})
target_link_libraries(testBehaviorCamera ${OpenCV_LIBS})
add_test(NAME testBehaviorCamera COMMAND behaviorCameraTest)
