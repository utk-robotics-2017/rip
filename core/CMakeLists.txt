cmake_minimum_required(VERSION 3.1)
message("Building core")
project(core)

add_subdirectory(utilities)
#add_subdirectory(framework)

add_subdirectory(navigation)

