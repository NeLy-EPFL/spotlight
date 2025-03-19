cmake_minimum_required(VERSION 3.10)

project(SpotlightController)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Enable debugging with frame pointers
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g3 -O0 -fno-omit-frame-pointer")

# Add pthread support for multithreaded applications
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")


find_package(Qt6 REQUIRED COMPONENTS Widgets SerialPort)
find_package(OpenCV REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp QUIET)

# Motion control (Zaber)
find_library(ZABER_MOTION_LIB NAMES zaber-motion REQUIRED)

# Frame grabber (Euresys)
set(EGRABBER_INCLUDE_DIR "/opt/euresys/egrabber/include")
include_directories(${EGRABBER_INCLUDE_DIR})

file(GLOB SOURCES
    "peripherals/*.cpp"
    "peripherals/*.hpp"
    "*.cpp"
    "*.hpp")
# message(STATUS "SOURCES: ${SOURCES}")

add_executable(SpotlightController ${SOURCES})
target_link_libraries(SpotlightController pthread rt dl)
target_link_libraries(SpotlightController Qt6::Widgets Qt6::SerialPort)
target_link_libraries(SpotlightController ${OpenCV_LIBS})
target_link_libraries(SpotlightController ${ZABER_MOTION_LIB})
target_link_libraries(SpotlightController spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>)
target_link_libraries(SpotlightController yaml-cpp)
set_target_properties(SpotlightController PROPERTIES AUTOMOC ON)

# file(GLOB RUN_CALIBRATION_SOURCES
#     "calibrationProcedure/runCalibration_main.cpp"
#     "calibrationProcedure/runCalibration.hpp"
#     "peripherals/behaviorCamera.cpp"
#     "peripherals/behaviorCamera.hpp")
# add_executable(RunCalibration ${RUN_CALIBRATION_SOURCES})
# # target_link_libraries(RunCalibration pthread rt dl)
# target_link_libraries(RunCalibration Qt6::Widgets Qt6::SerialPort)
# target_link_libraries(RunCalibration ${OpenCV_LIBS})
# target_link_libraries(RunCalibration ${ZABER_MOTION_LIB})
# target_link_libraries(RunCalibration spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>)
# target_link_libraries(RunCalibration yaml-cpp)
# set_target_properties(RunCalibration PROPERTIES AUTOMOC ON)
