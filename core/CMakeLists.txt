cmake_minimum_required(VERSION 3.1)
message("Building core")
project(core)

add_subdirectory(utilities)
add_subdirectory(framework)

if(NAVIGATION OR NAVX OR PATH_PLANNER)
    add_subdirectory(navigation)
else()
    # Not worth building right now.
    # add_subdirectory(communication)
    add_subdirectory(navigation)
endif()
