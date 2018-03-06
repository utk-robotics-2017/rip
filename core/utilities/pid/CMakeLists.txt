cmake_minimum_required(VERSION 3.1)
message("Building PidController")
# define the project
project(pid)

# create and configure the library target
file(GLOB_RECURSE ${PROJECT_NAME}_SOURCES src/*.cpp)
file(GLOB_RECURSE ${PROJECT_NAME}_HEADERS include/pid/*.hpp)
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
target_link_libraries(${PROJECT_NAME} units misc)
target_include_directories(${PROJECT_NAME} PUBLIC include)
