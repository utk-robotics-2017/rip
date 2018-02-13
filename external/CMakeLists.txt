cmake_minimum_required(VERSION 3.1)

message("Building third party libraries")

# 3rd party
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
add_subdirectory(optional-lite)
add_subdirectory(g2o)

# custom
message("Building custom addons")
add_subdirectory(googletest_rip_macros)
