cmake_minimum_required(VERSION 3.10)
project(QtHelloWorld)

find_package(Qt6 REQUIRED COMPONENTS Widgets)

add_executable(hello hello.cpp)
target_link_libraries(hello Qt6::Widgets)