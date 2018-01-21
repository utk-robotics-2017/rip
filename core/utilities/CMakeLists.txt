cmake_minimum_required(VERSION 3.1)
message("Building utilities")
project(utilities)

add_subdirectory(cmd_messenger)
add_subdirectory(exceptions_base)
add_subdirectory(units)
add_subdirectory(misc)
add_subdirectory(geometry)
