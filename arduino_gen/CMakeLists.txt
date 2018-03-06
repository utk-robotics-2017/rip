cmake_minimum_required(VERSION 3.1)
project(arduino_gen)

# Get all .cpp files except for main.cpp
file(GLOB_RECURSE ${PROJECT_NAME}_SOURCES "src/*.cpp")
file(GLOB_RECURSE ${PROJECT_NAME}_HEADERS "include/*.hpp")
list(REMOVE_ITEM ${PROJECT_NAME}_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp")

# Create the arduino_gen library, set to c++11, link external libraries, and set include dir
add_library(_${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})
set_property(TARGET _${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
if(ENABLE_TESTING)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -fprofile-arcs -ftest-coverage -DTESTING")
endif()
target_link_libraries(_${PROJECT_NAME} fmt json spdlog tinyxml2 cppfs misc)
target_link_libraries(_${PROJECT_NAME} gtest)
target_include_directories(_${PROJECT_NAME} PUBLIC include)

include(FileOutputs)

make_outputs(${CMAKE_CURRENT_SOURCE_DIR} "code_template.txt" ${CMAKE_CURRENT_BINARY_DIR} code_template)
make_appendage_xml_outputs(${CMAKE_CURRENT_BINARY_DIR}/appendages appendage_xml_outputs)
make_appendage_json_outputs(${CMAKE_CURRENT_BINARY_DIR}/appendages appendage_json_outputs)

# Create arduino_gen executable, set to c++11, and link arduino_gen library
add_executable(${PROJECT_NAME} "src/main.cpp" ${code_template} ${appendage_xml_outputs} ${appendage_json_outputs})
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
target_link_libraries(${PROJECT_NAME} _${PROJECT_NAME} args fmt)


if(ENABLE_TESTING)
    # Get test source files and test data files
    file(GLOB_RECURSE ${PROJECT_NAME}_TEST_SOURCES "test/*.cpp")
    file(GLOB_RECURSE ${PROJECT_NAME}_TEST_DATA "test/data/*")

    # Copy test data to outdir/test
    make_outputs(${CMAKE_CURRENT_SOURCE_DIR} "${${PROJECT_NAME}_TEST_DATA}" ${CMAKE_CURRENT_BINARY_DIR} testDataOutputs)

    # Create arduino_gen_test executable, set to c++11, link arduino_gen library and google test libraries
    add_executable(${PROJECT_NAME}_test ${${PROJECT_NAME}_TEST_SOURCES} ${testDataOutputs})
    set_property(TARGET ${PROJECT_NAME}_test PROPERTY CXX_STANDARD 11)
    target_link_libraries(${PROJECT_NAME}_test _${PROJECT_NAME})
    target_link_libraries(${PROJECT_NAME}_test gmock gtest googletest_rip_macros)
endif()
