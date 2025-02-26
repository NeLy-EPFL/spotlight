cmake_minimum_required(VERSION 3.10)

project(SpotlightController)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(Qt6 REQUIRED COMPONENTS Widgets SerialPort)
find_package(OpenCV REQUIRED)
find_package(spdlog REQUIRED)

# Motion control (Zaber)
find_library(ZABER_MOTION_LIB NAMES zaber-motion REQUIRED)

# Frame grabber (Euresys)
set(EGRABBER_INCLUDE_DIR "/opt/euresys/egrabber/include")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g -std=c++17")
include_directories(${EGRABBER_INCLUDE_DIR})

file(GLOB SOURCES
    "peripherals/*.cpp"
    "peripherals/*.hpp"
    "recordingController.cpp"
    "recordingController.hpp"
    "constants.hpp"
    "dataTypes.hpp"
    "utils.cpp"
    "utils.hpp"
    "gui.cpp"
    "gui.hpp"
    "main.cpp"
    "arduinoMessageProtocol.cpp"
    "arduinoMessageProtocol.hpp"
)

add_executable(SpotlightController ${SOURCES})
target_link_libraries(SpotlightController pthread rt dl)
target_link_libraries(SpotlightController Qt6::Widgets Qt6::SerialPort)
target_link_libraries(SpotlightController ${OpenCV_LIBS})
target_link_libraries(SpotlightController ${ZABER_MOTION_LIB})
target_link_libraries(SpotlightController spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>)
set_target_properties(SpotlightController PROPERTIES AUTOMOC ON)
