cmake_minimum_required(VERSION 3.1)
message("Building appendages")
project(appendages)

file(GLOB_RECURSE ${PROJECT_NAME}_SOURCES "src/*.cpp")
file(GLOB_RECURSE ${PROJECT_NAME}_HEADERS "include/*.hpp")

add_library(${PROJECT_NAME} EXCLUDE_FROM_ALL ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
target_link_libraries(${PROJECT_NAME}
    fmt json spdlog cppfs
    cmd_messenger misc)
target_include_directories(${PROJECT_NAME} PUBLIC include)

# TODO(Anthony): Refactor to a function or macro
file(GLOB_RECURSE ${PROJECT_NAME}_XMLS "xml/*.xml")
set(copiedXmls "")
foreach(template ${${PROJECT_NAME}_XMLS})
    file(RELATIVE_PATH name ${CMAKE_CURRENT_SOURCE_DIR} ${template})
    add_custom_command(
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${name}
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${template} ${CMAKE_CURRENT_BINARY_DIR}/${name}
        DEPENDS ${template}
        COMMENT "Copying ${name}"
        VERBATIM)
    list(APPEND copiedXmls ${CMAKE_CURRENT_BINARY_DIR}/${name})
endforeach()
add_custom_target(AppendageXmls SOURCES ${copiedXmls})

file(GLOB_RECURSE ${PROJECT_NAME}_JSONS "json/*.json")
set(copiedJsons "")
foreach(template ${${PROJECT_NAME}_JSONS})
    file(RELATIVE_PATH name ${CMAKE_CURRENT_SOURCE_DIR} ${template})
    add_custom_command(
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${name}
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${template} ${CMAKE_CURRENT_BINARY_DIR}/${name}
        DEPENDS ${template}
        COMMENT "Copying ${name}"
        VERBATIM)
    list(APPEND copiedJsons ${CMAKE_CURRENT_BINARY_DIR}/${name})
endforeach()
add_custom_target(AppendageJsons SOURCES ${copiedJsons})

add_custom_target(AppendageTemplates)
add_dependencies(AppendageTemplates AppendageXmls AppendageJsons)

