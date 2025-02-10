cmake_minimum_required(VERSION 3.10)

# Define the project
project(eGrabberExample)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Set up include directories for eGrabber
set(EGRABBER_INCLUDE_DIR "/opt/euresys/egrabber/include")

# Add compile options
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g -std=c++17")

# Set include directories
include_directories(${EGRABBER_INCLUDE_DIR})

# Find all sample source files
# file(GLOB C_SAMPLES "*.cpp")
# add_executable(main ${C_SAMPLES})
add_executable(main main.cpp)

# Link libraries (pthread, dl, rt)
target_link_libraries(main pthread rt dl)

# Link OpenCV libraries
target_link_libraries(main ${OpenCV_LIBS})

# Optionally, you can add other libraries here
# target_link_libraries(main <other-libraries>)

cmake_minimum_required(VERSION 3.10)
project(OpenCVExample)
