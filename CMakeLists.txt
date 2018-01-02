cmake_minimum_required(VERSION 3.1)
project(rip)
message(building)
# Get the current working branch
execute_process(
    COMMAND git rev-parse --abbrev-ref HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_BRANCH
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(ALL "Build everything" OFF)
message("Git branch: ${GIT_BRANCH}")

add_subdirectory(external)

if( GIT_BRANCH MATCHES "arduino_gen/.*")
    add_subdirectory(arduino_gen)
    add_subdirectory(gui)
elseif(GIT_BRANCH MATCHES "navigation/.*")
    add_subdirectory(core)
    add_subdirectory(gui)
elseif(GIT_BRANCH MATCHES "cmdmessenger/.*")
    add_subdirectory(core)
    add_subdirectory(gui)
elseif(GIT_BRANCH MATCHES "path_planner/.*")
    add_subdirectory(core)
    add_subdirectory(gui)
elseif(GIT_BRANCH MATCHES "navx/.*")
    add_subdirectory(core)
    add_subdirectory(gui)
else()
    add_subdirectory(appendages)
    add_subdirectory(core)
    add_subdirectory(gui)
endif()
