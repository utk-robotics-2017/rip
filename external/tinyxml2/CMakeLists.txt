cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
message("Building tinyxml2")
project(tinyxml2)

add_library(tinyxml2 src/tinyxml2.cpp)
target_include_directories(tinyxml2 PUBLIC include)
