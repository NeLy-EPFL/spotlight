cmake_minimum_required(VERSION 3.20 FATAL_ERROR)

project("my_pco_project" VERSION 1.0.0 DESCRIPTION "PCO Camera HelloWorld Project" LANGUAGES CXX)

# Use C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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

# Define PCO_LINUX for Linux builds
add_compile_definitions(PCO_LINUX)

# Define PCO source files
set(PCOCAMERA_SOURCES
    ${PCO_CAMERA_DIR}/cameraexception.cpp
    ${PCO_CAMERA_DIR}/image.cpp
    ${PCO_CAMERA_DIR}/camera.cpp
    ${PCO_CAMERA_DIR}/xcite.cpp
    ${PCO_CAMERA_DIR}/stdafx.cpp
)

# Create the HelloWorld executable including PCO source files
add_executable(HelloWorld 
    src/HelloWorld.cpp
    ${PCOCAMERA_SOURCES}
)

# Add PCO include directories
target_include_directories(HelloWorld 
    PRIVATE 
    ${PCO_INCLUDE_DIR}
    ${PCO_CAMERA_DIR}
)

# Link with PCO libraries
target_link_directories(HelloWorld 
    PRIVATE 
    ${PCO_LIB_DIR}
)

target_link_libraries(HelloWorld 
    PRIVATE 
    ${SDKLIB}
    ${RECORDERLIB}
    ${CONVERTLIB}
    ${XCITELIB}
)

# Set RPATH to find shared libraries at runtime
set_target_properties(HelloWorld PROPERTIES
    INSTALL_RPATH "${PCO_LIB_DIR}"
    BUILD_WITH_INSTALL_RPATH TRUE
)

# Install target
install(TARGETS HelloWorld
    RUNTIME DESTINATION bin
)