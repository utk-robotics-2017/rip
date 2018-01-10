cmake_minimum_required(VERSION 3.1)
message("Building utilities")
project(utils)

# 3rd party
message("Building 3rd party utilities")
add_subdirectory(googletest)
add_subdirectory(fmt)
add_subdirectory(json)
add_subdirectory(spdlog)
add_subdirectory(tinyxml2)
add_subdirectory(periphery)
add_subdirectory(cppfs)
add_subdirectory(clipper)
add_subdirectory(stp)
add_subdirectory(serial)

# custom
message("Building custom utilities")
add_subdirectory(googletest_rip_macros)
