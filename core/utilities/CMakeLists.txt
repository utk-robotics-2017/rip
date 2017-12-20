cmake_minimum_required(VERSION 3.1)
message("Building utilities")
project(utilities)

add_subdirectory(units)
if(GIT_BRANCH MATCHES "navigation/.*" OR GIT_BRANCH MATCHES "path_planner/.*")
	add_subdirectory(geometry)
endif()