cmake_minimum_required(VERSION 2.8.12)
message("Building utilities")
project(utils)

#3rd party
message("Building 3rd party utilities")
add_subdirectory(fmt)
add_subdirectory(googletest)
add_subdirectory(json)
add_subdirectory(spdlog)
add_subdirectory(tinyxml2)
add_subdirectory(serial)


# custom
message("Building custom utilities")
add_subdirectory(exceptions_base)
add_subdirectory(pathman)
add_subdirectory(units)
add_subdirectory(googletest_rip_macros)

if(RIP OR RIPCOM)
    add_subdirectory(cmd_messenger)
    add_subdirectory(roboclaw)
else()
    if(CMD_MESSENGER)
        add_subdirectory(cmd_messenger)
    endif()

    if(ROBOCLAW)
        add_subdirectory(roboclaw)
    endif()
endif()
