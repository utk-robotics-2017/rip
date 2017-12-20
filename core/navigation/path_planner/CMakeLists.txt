cmake_minimum_required(VERSION 3.1)
message("Building path planner")
# define the project
project(path_planner)

# create and configure the library target
file(GLOB_RECURSE ${PROJECT_NAME}_SOURCES src/*.cpp)
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES})
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
target_link_libraries(${PROJECT_NAME} fmt json units spdlog stp geometry)
target_include_directories(${PROJECT_NAME} PUBLIC include)
