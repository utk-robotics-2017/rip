
cmake_minimum_required(VERSION 3.1)
message("Building clipper")
# define the project
project(clipper)

# create and configure the library target
add_library(${PROJECT_NAME} src/clipper.cpp include/clipper.hpp)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
target_include_directories(${PROJECT_NAME} PUBLIC include)