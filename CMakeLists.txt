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

# Can remove this if found to be not needed - was required on some machines
# to correctly compile the codebase despite this line being in the
# downstream CMake files.
set (CMAKE_CXX_STANDARD 11)

message("Git branch: ${GIT_BRANCH}")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeModules)

#if(GIT_BRANCH MATCHES "arduino_gen/.*")
#    add_subdirectory(core/utilities/exceptions_base)
#    add_subdirectory(arduino_gen)
#    add_subdirectory(gui)
#else()
#    add_subdirectory(core)
#    add_subdirectory(appendages)
#    add_subdirectory(arduino_gen)
#    add_subdirectory(gui)
#endif()

option(ALL "Build everything" OFF)

# RIP Modules
option(RIP "Build all of RIP" OFF)
option(COMMUNICATION "Build RIP's communication module" OFF)
option(NAVIGATION "Build RIP's navigation module" OFF)
option(NAVX "Build RIP's navx submodule" OFF)
option(PATHFINDER "Build RIP's pathfinder submodule" OFF)
option(PATH_PLANNER "Build RIP's path planner submodule" OFF)
option(VISION "Build RIP's vision module" OFF)
option(FSM "Build RIP's fsm and action scheduler modules" OFF)

# Other Modules
option(APPENDAGES "Build appendages" OFF)
option(RIPCOM "Build RIPCOM" OFF)
option(ARDUINO_GEN "Build ArduinoGen" OFF)

# Utilities
option(ROBOCLAW "Build Roboclaw" OFF)
option(PATHMAN "Build Pathman" OFF)
option(CMD_MESENGER "Build CmdMessenger" OFF)

# Testing
option(ENABLE_TESTING "Enable testing of the modules" OFF)
option(DEBUG_RIP_CMAKE "Prints more debug cmake info" OFF)

if (NOT (ALL OR RIP OR COMMUNICATION OR NAVIGATION OR NAVX OR
         PATH_PLANNER OR PATHFINDER OR VISION OR FSM OR RIPCOM OR
         ARDUINO_GEN OR ROBOCLAW OR PATHMAN OR CMD_MESENGER))

    if(GIT_BRANCH MATCHES "appendages/.*")
        set(APPENDAGES ON)
    elseif(GIT_BRANCH MATCHES "arduino_gen/.*")
        set(ARDUINO_GEN ON)
    elseif(GIT_BRANCH MATCHES "cmd_messenger/.*")
        set(CMD_MESSENGER ON)
    elseif(GIT_BRANCH MATCHES "navigation/.*")
        set(NAVIGATION ON)
    elseif(GIT_BRANCH MATCHES "navx/.*")
        set(NAVX ON)
    elseif(GIT_BRANCH MATCHES "roboclaw/.*")
        set(ROBOCLAW ON)
    elseif(GIT_BRANCH MATCHES "path_planner/.*")
        set(PATH_PLANNER ON)
    endif()
endif()

if (DEBUG_RIP_CMAKE)
    message("ALL: " ${ALL})
    message("RIP: " ${RIP})
    message("COMMUNICATION: " ${COMMUNICATION})
    message("NAVIGATION: " ${NAVIGATION})
    message("NAVX: " ${NAVX})
    message("PATHFINDER: " ${PATHFINDER})
    message("PATH_PLANNER: " ${PATH_PLANNER})
    message("VISION: " ${VISION})
    message("FSM: " ${FSM})
    message("RIPCOM: " ${RIPCOM})
    message("ARDUINO_GEN: " ${ARDUINO_GEN})
    message("ROBOCLAW: " ${ROBOCLAW})
    message("PATHMAN: " ${PATHMAN})
    message("CMD_MESENGER: " ${CMD_MESENGER})
endif()

add_subdirectory(external)

if(ALL OR RIP OR CORE OR RIPCOM OR APPENDAGES OR ARDUINO_GEN)
    message("Adding appendages")
    add_subdirectory(appendages)
endif()

if(ALL OR ARDUINO_GEN)
    message("Adding ArduinoGen")
    add_subdirectory(arduino_gen)
endif()

if(ALL OR RIP OR COMMUNICATION OR NAVIGATION OR NAVX OR PATHFINDER OR VISION OR FSM OR ARDUINO_GEN)
    message("Adding RIP Core")
    add_subdirectory(core)
endif()

if(ALL OR RIPCOM)
    message("Adding RIPCOM")
    add_subdirectory(ripcom)
endif()
