cmake_minimum_required(VERSION 3.10)
project(ArduinoHelloWorld VERSION 1.0)

# Specify C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find Boost package
find_package(Boost REQUIRED COMPONENTS system)

# Add executable
add_executable(arduino_hello arduino_hello.cpp)

# Link Boost libraries
target_link_libraries(arduino_hello 
    PRIVATE 
    Boost::system 
    pthread
)

# Print information during configuration
message(STATUS "Using C++17")
message(STATUS "Boost libraries: ${Boost_LIBRARIES}")
message(STATUS "Boost include dirs: ${Boost_INCLUDE_DIRS}")

# Installation rules
install(TARGETS arduino_hello DESTINATION bin)