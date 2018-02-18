cmake_minimum_required(VERSION 3.0)
message("Building property")
project(property)

# create and configure the library target
add_library(${PROJECT_NAME} include/property/property.hpp src/property.cpp)
SET(CMAKE_CXX_STANDARD 11)
target_include_directories(${PROJECT_NAME} PUBLIC include)