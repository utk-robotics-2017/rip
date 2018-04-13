cmake_minimum_required(VERSION 3.0)
message("Building stp")
# define the project
project(stp)

# create and configure the library target
file(GLOB_RECURSE ${PROJECT_NAME}_SOURCES src/*.cc)
add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_SOURCES})
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
target_include_directories(${PROJECT_NAME} PUBLIC include)
