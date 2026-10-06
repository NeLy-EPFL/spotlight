cmake_minimum_required(VERSION 3.20 FATAL_ERROR)

project(pco_camera_clock_drift_profiling LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# PCO SDK (same layout as opt/examples/pco/pco_standalone; recent SDK versions
# no longer ship pco.camera/stdafx.cpp)
set(PCO_ROOT_DIR "/opt/pco/pco.cpp")
set(PCO_INCLUDE_DIR "${PCO_ROOT_DIR}/include")
set(PCO_LIB_DIR "${PCO_ROOT_DIR}/lib")
set(PCO_CAMERA_DIR "${PCO_ROOT_DIR}/pco.camera")

add_compile_definitions(PCO_LINUX)

add_executable(pco_camera_clock_drift_profiling
    src/main.cc
    ${PCO_CAMERA_DIR}/cameraexception.cpp
    ${PCO_CAMERA_DIR}/image.cpp
    ${PCO_CAMERA_DIR}/camera.cpp
    ${PCO_CAMERA_DIR}/xcite.cpp
)

target_include_directories(pco_camera_clock_drift_profiling
    PRIVATE ${PCO_INCLUDE_DIR} ${PCO_CAMERA_DIR}
)

target_link_directories(pco_camera_clock_drift_profiling PRIVATE ${PCO_LIB_DIR})
target_link_libraries(pco_camera_clock_drift_profiling
    PRIVATE pco_sc2cam pco_recorder pco_convert etc_xcite
)

set_target_properties(pco_camera_clock_drift_profiling PROPERTIES
    INSTALL_RPATH "${PCO_LIB_DIR}"
    BUILD_WITH_INSTALL_RPATH TRUE
)
