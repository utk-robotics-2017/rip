cmake_minimum_required(VERSION 3.1)
project(navx)

# Get all .cpp files except for main.cpp
file(GLOB_RECURSE ${PROJECT_NAME}_SOURCES "src/*.cpp")

# Create the navx library, set to c++11, link external libraries, and set include dir
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES})
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
if(ENABLE_TESTING)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -fprofile-arcs -ftest-coverage")
endif()
target_link_libraries(${PROJECT_NAME} units spdlog periphery)
target_include_directories(${PROJECT_NAME} PUBLIC include)

if(ENABLE_DIAG)
	file(GLOB_RECURSE ${PROJECT_NAME}_DIAG_SOURCES "diag/*.cpp")
	add_executable(${PROJECT_NAME}_diag ${${PROJECT_NAME}_DIAG_SOURCES})
	set_property(TARGET ${PROJECT_NAME}_diag PROPERTY CXX_STANDARD 11)
	target_link_libraries(${PROJECT_NAME}_diag ${PROJECT_NAME})
	target_link_libraries(${PROJECT_NAME}_diag periphery units)
endif()

if(ENABLE_TESTING)
    # Get test source files and test data files
    file(GLOB_RECURSE ${PROJECT_NAME}_TEST_SOURCES "test/*.cpp")
    file(GLOB_RECURSE ${PROJECT_NAME}_TEST_DATA "test/data/*")

    # Copy test data to outdir/test
    set(copiedTestData "")
    foreach(TestDataFile ${${PROJECT_NAME}_TEST_DATA})
        file(RELATIVE_PATH name ${CMAKE_CURRENT_SOURCE_DIR} ${TestDataFile})
        add_custom_command(
            OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${name}
            COMMAND ${CMAKE_COMMAND} -E copy_if_different ${TestDataFile} ${CMAKE_CURRENT_BINARY_DIR}/${name}
            DEPENDS ${TestDataFile}
            COMMENT "Copying ${name}"
            VERBATIM
            )
        list(APPEND copiedTestData ${CMAKE_CURRENT_BINARY_DIR}/${name})
    endforeach()

    # Create arduino_gen_test executable, set to c++11, link navx library and google test libraries
    add_executable(${PROJECT_NAME}_test ${${PROJECT_NAME}_TEST_SOURCES} ${copiedTestData})
    set_property(TARGET ${PROJECT_NAME}_test PROPERTY CXX_STANDARD 11)
    target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
    target_link_libraries(${PROJECT_NAME}_test gmock gtest googletest_rip_macros)


    # In CMakeModules I include code coverage for c++
    set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeModules)

    if(CMAKE_COMPILER_IS_GNUCXX)
        include(CodeCoverage)
        setup_target_for_coverage(${PROJECT_NAME}_coverage ${PROJECT_NAME}_test coverage)
    endif()
endif()
