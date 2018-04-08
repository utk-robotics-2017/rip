cmake_minimum_required(VERSION 3.1)
message("Building appendages")
project(appendages)

file(GLOB_RECURSE ${PROJECT_NAME}_SOURCES "src/*.cpp")
file(GLOB_RECURSE ${PROJECT_NAME}_HEADERS "include/*.hpp")

add_library(${PROJECT_NAME} STATIC EXCLUDE_FROM_ALL ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
target_link_libraries(${PROJECT_NAME}
    fmt json spdlog cppfs
    cmd_messenger units misc motor_controllers)
target_include_directories(${PROJECT_NAME} PUBLIC include)

set_property(GLOBAL PROPERTY AppendageRoot ${CMAKE_CURRENT_SOURCE_DIR})

file(GLOB _AppendageXmls "xml/*.xml")
set_property(GLOBAL PROPERTY AppendageXmls ${_AppendageXmls})
file(GLOB _AppendageJsons "json/*.json")
set_property(GLOBAL PROPERTY AppendageJsons ${_AppendageJsons})

include(FileOutputs)

function(make_appendage_xml_outputs outdir xml_outputs)
    get_property(_AppendageRoot GLOBAL PROPERTY AppendageRoot)
    get_property(_AppendageXmls GLOBAL PROPERTY AppendageXmls)

    make_outputs(${_AppendageRoot} "${_AppendageXmls}" ${outdir} _outputs)
    set(${xml_outputs} ${_outputs} PARENT_SCOPE)
endfunction()

function(make_appendage_json_outputs outdir json_outputs)
    get_property(_AppendageRoot GLOBAL PROPERTY AppendageRoot)
    get_property(_AppendageJsons GLOBAL PROPERTY AppendageJsons)

    make_outputs(${_AppendageRoot} "${_AppendageJsons}" ${outdir} _outputs)
    set(${json_outputs} ${_outputs} PARENT_SCOPE)
endfunction()
