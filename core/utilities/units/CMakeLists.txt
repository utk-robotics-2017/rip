cmake_minimum_required(VERSION 3.0)
message("Building units")
project(units)

# create and configure the library target
add_library(${PROJECT_NAME} STATIC include/units/units.hpp src/units.cpp)
SET(CMAKE_CXX_STANDARD 11)
target_link_libraries(${PROJECT_NAME} json)
target_include_directories(${PROJECT_NAME} PUBLIC include)
