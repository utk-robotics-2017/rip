cmake_minimum_required(VERSION 3.1)
project(rip)
message(building)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Can remove this if found to be not needed - was required on some machines
# to correctly compile the codebase despite this line being in the
# downstream CMake files.
set (CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}-fdiagnostics-color=always")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeModules)

option(ENABLE_TESTING "Enable module testing" OFF)

add_subdirectory(external)
add_subdirectory(core)
add_subdirectory(appendages)
add_subdirectory(arduino_gen)

option(GUI "Build the GUI" OFF)
if(GUI)
    message("Building the GUI")
    add_subdirectory(gui)
endif()
