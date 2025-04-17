cmake_minimum_required(VERSION 3.20 FATAL_ERROR)

project("SpotlightController"
    VERSION 1.0.0
    DESCRIPTION "Control software for the Spotlight setup"
    LANGUAGES CXX)

# Use C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# This is used by the PCO stuff
add_compile_definitions(PCO_LINUX)

# Enable debugging with frame pointers
set(CMAKE_CXX_FLAGS_DEBUG
    "${CMAKE_CXX_FLAGS_DEBUG} -g3 -O0 -fno-omit-frame-pointer")

# Add pthread support for multithreaded applications
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

# Find required packages
find_package(Qt6 REQUIRED COMPONENTS Widgets SerialPort)
find_package(OpenCV REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp QUIET)

# Motion control (Zaber)
find_library(ZABER_MOTION_LIB NAMES zaber-motion REQUIRED)

# Frame grabber (Euresys)
set(EGRABBER_INCLUDE_DIR "/opt/euresys/egrabber/include")
include_directories(${EGRABBER_INCLUDE_DIR})

# Set paths for PCO library
set(PCO_ROOT_DIR "/opt/pco/pco.cpp")
set(PCO_INCLUDE_DIR "${PCO_ROOT_DIR}/include")
set(PCO_LIB_DIR "${PCO_ROOT_DIR}/lib")
set(PCO_CAMERA_DIR "${PCO_ROOT_DIR}/pco.camera")

# Define PCO libraries for Linux
set(SDKLIB pco_sc2cam)
set(RECORDERLIB pco_recorder)
set(CONVERTLIB pco_convert)
set(XCITELIB etc_xcite)

# Define PCO source files
set(PCOCAMERA_SOURCES
    ${PCO_CAMERA_DIR}/cameraexception.cpp
    ${PCO_CAMERA_DIR}/image.cpp
    ${PCO_CAMERA_DIR}/camera.cpp
    ${PCO_CAMERA_DIR}/xcite.cpp
    ${PCO_CAMERA_DIR}/stdafx.cpp)


# PCO camera server
file(GLOB PCO_CAMERA_SERVER_SOURCES
    "pcoCameraServer/*.cpp"
    "pcoCameraServer/*.hpp"
    "recorderConfig.cpp"
    "recorderConfig.hpp")
add_executable(pco-camera-server ${PCO_CAMERA_SERVER_SOURCES} ${PCOCAMERA_SOURCES})
target_include_directories(pco-camera-server PUBLIC ${PCO_INCLUDE_DIR} ${PCO_CAMERA_DIR})
target_link_directories(pco-camera-server PUBLIC ${PCO_LIB_DIR})
target_link_libraries(pco-camera-server PUBLIC ${SDKLIB} ${RECORDERLIB} ${CONVERTLIB} ${XCITELIB})
target_link_libraries(pco-camera-server PUBLIC pthread rt dl)
target_link_libraries(pco-camera-server PUBLIC ${OpenCV_LIBS})
target_link_libraries(pco-camera-server PUBLIC spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>)
target_link_libraries(pco-camera-server PUBLIC yaml-cpp)
set_target_properties(pco-camera-server PROPERTIES
    INSTALL_RPATH "${PCO_LIB_DIR}"
    BUILD_WITH_INSTALL_RPATH TRUE)

file(GLOB MAIN_APPLICATION_SOURCES
    "peripherals/*.cpp"
    "peripherals/*.hpp"
    "pcoCameraServer/sharedMemoryUtils.*"
    "*.cpp"
    "*.hpp")
add_executable(SpotlightController ${MAIN_APPLICATION_SOURCES})
target_link_libraries(SpotlightController pthread rt dl)
target_link_libraries(SpotlightController Qt6::Widgets Qt6::SerialPort)
target_link_libraries(SpotlightController ${OpenCV_LIBS})
target_link_libraries(SpotlightController ${ZABER_MOTION_LIB})
target_link_libraries(SpotlightController spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>)
target_link_libraries(SpotlightController yaml-cpp)
set_target_properties(SpotlightController PROPERTIES AUTOMOC ON)

# Compile calibration tool
file(GLOB RUN_CALIBRATION_SOURCES
    "calibrationProcedure/runCalibration_main.cpp"
    "calibrationProcedure/runCalibration.hpp"
    "peripherals/behaviorCamera.*"
    "peripherals/muscleCamera.*"
    "peripherals/motionControl.*"
    "pcoCameraServer/sharedMemoryUtils.*"
    "cli.*"
    "recorderConfig.*"
    "behaviorRecording.*"
    "muscleRecording.*"
    "utils.*")
add_executable(run-calibration-scan ${RUN_CALIBRATION_SOURCES})
# target_link_libraries(run-calibration-scan pthread rt dl)
target_link_libraries(run-calibration-scan Qt6::Widgets Qt6::SerialPort)
target_link_libraries(run-calibration-scan ${OpenCV_LIBS})
target_link_libraries(run-calibration-scan ${ZABER_MOTION_LIB})
target_link_libraries(run-calibration-scan spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>)
target_link_libraries(run-calibration-scan yaml-cpp)

# Compile camera alignment tool
file(GLOB ALIGN_CAMERAS_SOURCES
    "cameraAlignment/*.cpp"
    "cameraAlignment/*.hpp"
    "peripherals/behaviorCamera.*"
    "peripherals/muscleCamera.*"
    "pcoCameraServer/sharedMemoryUtils.*"
    "behaviorRecording.*"
    "muscleRecording.*"
    "recorderConfig.*"
    "utils.*"
    "cli.*"
    "dataTypes.hpp")
add_executable(align-cameras ${ALIGN_CAMERAS_SOURCES})
target_link_libraries(align-cameras PRIVATE pthread rt dl)
target_link_libraries(align-cameras PRIVATE Qt6::Widgets Qt6::SerialPort)
target_link_libraries(align-cameras PRIVATE ${OpenCV_LIBS})
target_link_libraries(align-cameras PRIVATE spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>)
target_link_libraries(align-cameras PRIVATE yaml-cpp)
set_target_properties(align-cameras PROPERTIES AUTOMOC ON)