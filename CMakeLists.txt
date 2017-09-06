cmake_minimum_required(VERSION 3.1)
project(rip)

option(ALL "Build everything" OFF)

# RIP Modules
option(RIP "Build all of RIP" OFF)
option(COMMUNICATION "Build RIP's communication module" OFF)
option(NAVIGATION "Build RIP's navigation module" OFF)
option(VISION "Build RIP's vision module" OFF)
option(FSM "Build RIP's fms module" OFF)

# Other Modules
option(RIPCOM "Build RIPCOM" OFF)
option(ARDUINO_GEN "Build ArduinoGen" OFF)

# Utilities
option(ROBOCLAW "Build Roboclaw" OFF)
option(PATHMAN "Build Pathman" OFF)
option(CMD_MESENGER "Build CmdMessenger" OFF)

# Testing
option(ENABLE_TESTING "Enable testing of the modules" OFF)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeModules)

add_subdirectory(utilities)

if(ALL OR RIP OR CORE OR RIPCOM)
    message("Adding appendages")
    add_subdirectory(appendages)
endif()

if(ALL OR ARDUINO_GEN)
    message("Adding ArduinoGen")
    add_subdirectory(arduino_gen)
endif()

if(ALL OR RIP OR CORE)
    message("Adding RIP Core")
    add_subdirectory(core)
endif()

if(ALL OR RIPCOM)
    message("Adding RIPCOM")
    add_subdirectory(ripcom)
endif()
