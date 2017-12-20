cmake_minimum_required(VERSION 3.1)
message("Building core")
project(core)

add_subdirectory(utilities)

if(GIT_BRANCH MATCHES "cmdmessenger/.*" OR GIT_BRANCH MATCHES "communication/.*")
	add_subdirectory(communication)
elseif(GIT_BRANCH MATCHES "navigation/.*" OR GIT_BRANCH MATCHES "navx/.*" OR GIT_BRANCH MATCHES "path_planner/.*")
	add_subdirectory(navigation)
else()
	add_subdirectory(communication)
	add_subdirectory(navigation)
endif()