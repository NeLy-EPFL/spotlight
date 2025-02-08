cmake_minimum_required(VERSION 3.10)
project(OpenCVExample)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Add executable
add_executable(main main.cpp)

# Link OpenCV libraries
target_link_libraries(main ${OpenCV_LIBS})

