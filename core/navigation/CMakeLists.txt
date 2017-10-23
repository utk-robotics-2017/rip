cmake_minimum_required(VERSION 3.1)

if(ALL OR RIP OR NAVIGATION OR NAVX)
	add_subdirectory(navx)
endif()

if(ALL OR RIP OR NAVIGATION OR PATHFINDER)
	add_subdirectory(pathfinder)
endif()