cmake_minimum_required(VERSION 3.10)

# Project name
project(ZaberExample CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find the Zaber Motion library (assumes it's installed and in the system path)
find_library(ZABER_MOTION_LIB NAMES zaber-motion REQUIRED)

# Add executable
add_executable(zaber_example zaber_example.cpp)

# Link the Zaber Motion library
target_link_libraries(zaber_example PRIVATE ${ZABER_MOTION_LIB})