cmake_minimum_required(VERSION 3.1)

if(ALL OR RIP OR COMMUNICATION)
	add_subdirectory(communication)
endif()

if(ALL OR RIP OR NAVIGATION OR NAVX OR PATHFINDER)
	add_subdirectory(navigation)
endif()

if(ALL OR RIP OR VISION)
	add_subdirectory(vision)
endif()

if(ALL OR RIP OR FSM)
	add_subdirectory(fsm)
endif()