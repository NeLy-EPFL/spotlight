cmake_minimum_required(VERSION 3.20 FATAL_ERROR)

project("SpotlightController"
    VERSION 0.2.0
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
    "src/pcoCameraServer/*.cpp"
    "src/pcoCameraServer/*.hpp"
    "src/common/recorderConfig.*"
    "src/common/fileFormatVersions.*")
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

# Compile main recording application
file(GLOB MAIN_APPLICATION_SOURCES
    "src/peripherals/*.cpp"
    "src/peripherals/*.hpp"
    "src/pcoCameraServer/sharedMemoryUtils.*"
    "src/common/*.cpp"
    "src/common/*.hpp"
    "src/runSpotlight/*.cpp"
    "src/runSpotlight/*.hpp")
add_executable(run-spotlight ${MAIN_APPLICATION_SOURCES})
target_link_libraries(run-spotlight pthread rt dl)
target_link_libraries(run-spotlight Qt6::Widgets Qt6::SerialPort)
target_link_libraries(run-spotlight ${OpenCV_LIBS})
target_link_libraries(run-spotlight ${ZABER_MOTION_LIB})
target_link_libraries(run-spotlight spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>)
target_link_libraries(run-spotlight yaml-cpp)
set_target_properties(run-spotlight PROPERTIES AUTOMOC ON)

# Compile calibration tool
file(GLOB RUN_CALIBRATION_SOURCES
    "src/runCalibrationScan/runCalibration_main.cpp"
    "src/runCalibrationScan/runCalibration.hpp"
    "src/peripherals/behaviorCamera.*"
    "src/peripherals/muscleCamera.*"
    "src/peripherals/motionControl.*"
    "src/peripherals/arduinoCommunication.*"
    "src/peripherals/experimentProtocol.*"
    "src/pcoCameraServer/sharedMemoryUtils.*"
    "src/common/cli.*"
    "src/common/recorderConfig.*"
    "src/common/behaviorRecording.*"
    "src/common/muscleRecording.*"
    "src/common/utils.*"
    "src/common/fileFormatVersions.*")
add_executable(run-calibration-scan ${RUN_CALIBRATION_SOURCES})
# target_link_libraries(run-calibration-scan pthread rt dl)
target_link_libraries(run-calibration-scan Qt6::Widgets Qt6::SerialPort)
target_link_libraries(run-calibration-scan ${OpenCV_LIBS})
target_link_libraries(run-calibration-scan ${ZABER_MOTION_LIB})
target_link_libraries(run-calibration-scan spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>)
target_link_libraries(run-calibration-scan yaml-cpp)

# Compile camera alignment tool
file(GLOB ALIGN_CAMERAS_SOURCES
    "src/alignCameras/*.cpp"
    "src/alignCameras/*.hpp"
    "src/peripherals/behaviorCamera.*"
    "src/peripherals/muscleCamera.*"
    "src/peripherals/arduinoCommunication.*"
    "src/peripherals/experimentProtocol.*"
    "src/pcoCameraServer/sharedMemoryUtils.*"
    "src/common/behaviorRecording.*"
    "src/common/muscleRecording.*"
    "src/common/recorderConfig.*"
    "src/common/utils.*"
    "src/common/cli.*"
    "src/common/dataTypes.hpp"
    "src/common/fileFormatVersions.*")
add_executable(align-cameras ${ALIGN_CAMERAS_SOURCES})
target_link_libraries(align-cameras PRIVATE pthread rt dl)
target_link_libraries(align-cameras PRIVATE Qt6::Widgets Qt6::SerialPort)
target_link_libraries(align-cameras PRIVATE ${OpenCV_LIBS})
target_link_libraries(align-cameras PRIVATE spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>)
target_link_libraries(align-cameras PRIVATE yaml-cpp)
set_target_properties(align-cameras PROPERTIES AUTOMOC ON)

# Compile arena registration scan tool
find_library(DMTX_LIB NAMES dmtx REQUIRED)
file(GLOB RUN_ARENA_REGISTRATION_SCAN_SOURCES
    "src/runArenaRegistrationScan/runArenaRegistrationScan_main.cpp"
    "src/runArenaRegistrationScan/runArenaRegistrationScan.hpp"
    "src/peripherals/behaviorCamera.*"
    "src/peripherals/motionControl.*"
    "src/peripherals/arduinoCommunication.*"
    "src/peripherals/experimentProtocol.*"
    "src/pcoCameraServer/sharedMemoryUtils.*"
    "src/common/cli.*"
    "src/common/recorderConfig.*"
    "src/common/behaviorRecording.*"
    "src/common/utils.*"
    "src/common/fileFormatVersions.*")
add_executable(run-arena-registration-scan ${RUN_ARENA_REGISTRATION_SCAN_SOURCES})
target_link_libraries(run-arena-registration-scan Qt6::Widgets Qt6::SerialPort)
target_link_libraries(run-arena-registration-scan ${OpenCV_LIBS})
target_link_libraries(run-arena-registration-scan ${ZABER_MOTION_LIB})
target_link_libraries(run-arena-registration-scan spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>)
target_link_libraries(run-arena-registration-scan yaml-cpp)
target_link_libraries(run-arena-registration-scan ${DMTX_LIB})

# Install compiled binaries to the bin folder
set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR})
install(
    TARGETS
        pco-camera-server
        run-spotlight
        run-calibration-scan
        align-cameras
        run-arena-registration-scan
    RUNTIME DESTINATION
        bin
)