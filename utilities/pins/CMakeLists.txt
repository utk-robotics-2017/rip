cmake_minimum_requirements(VERSION 3.1)
message("Building pins")
project(pins)

file(GLOB_RECURSE ${PROJECT_NAME}_SOURCES "src/*.cpp")

add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES})
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
target_link_libraries(${PROJECT_NAME} fmt spdlog)
target_include_directories(${PROJECT_NAME} PUBLIC include)