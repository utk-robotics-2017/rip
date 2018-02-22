cmake_minimum_required(VERSION 3.0)
message("Building factory")
project(factory)

# create and configure the library target
add_library(${PROJECT_NAME} include/factory/factory.hpp src/factory.cpp)
SET(CMAKE_CXX_STANDARD 11)
target_include_directories(${PROJECT_NAME} PUBLIC include)