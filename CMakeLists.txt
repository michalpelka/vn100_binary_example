
cmake_minimum_required(VERSION 3.0.2)
project(livox_vn100_multisensor)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)


## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread chrono)

include_directories(include ${CMAKE_SOURCE_DIR})

include_directories(3rd/Boost-Serial-Port/ 3rd/hexdump)
add_library(client_vn100  STATIC vn100/vn100.cpp
        vn100/vn100_parsing.cpp
        3rd/Boost-Serial-Port/BoostSerial.cpp)

add_executable (print_vn100 print_vn100.cpp)
target_link_libraries(print_vn100 client_vn100)
