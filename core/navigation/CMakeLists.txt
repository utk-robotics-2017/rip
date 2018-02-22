cmake_minimum_required(VERSION 3.1)
message("Building navigation")
project(navigation)

add_subdirectory(path_planner)
add_subdirectory(actions)
add_subdirectory(drivetrains)
add_subdirectory(navx)
