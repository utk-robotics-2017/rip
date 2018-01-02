cmake_minimum_required(VERSION 3.1)

# define the project
project(exceptions_base)

# create and configure the library target
add_library(${PROJECT_NAME} INTERFACE)
SET(CMAKE_CXX_STANDARD 11)
target_include_directories(${PROJECT_NAME} INTERFACE include)
