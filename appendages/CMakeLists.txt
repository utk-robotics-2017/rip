cmake_minimum_required(VERSION 3.1)
message("Building appendages")
project(appendages)

file(GLOB_RECURSE ${PROJECT_NAME}_SOURCES "src/*.cpp")
file(GLOB_RECURSE ${PROJECT_NAME}_HEADERS "include/*.hpp")

add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
target_link_libraries(${PROJECT_NAME}
    fmt json spdlog cppfs
    cmd_messenger misc)
target_include_directories(${PROJECT_NAME} PUBLIC include)


# if(GIT_BRANCH MATCHES "arduino_gen/.*" OR GIT_BRANCH MATCHES "dev" OR GIT_BRANCH MATCHES "master")
#     file(GLOB_RECURSE ${PROJECT_NAME}_TEMPLATES "xml/*.xml")
#
#     set(copiedAppendageTemplates "")
#     foreach(AppendageTemplate ${${PROJECT_NAME}_TEMPLATES})
#         file(RELATIVE_PATH name ${CMAKE_CURRENT_SOURCE_DIR}
#             ${AppendageTemplate})
#         add_custom_command(
#             OUTPUT ${CMAKE_CURRENT_SOURCE_DIR} ${AppendageTemplate}
#             COMMAND ${CMAKE_COMMAND} -E copy_if_different ${AppendageTemplate}
#             ${CMAKE_CURRENT_BINARY_DIR}/${name}
#             DEPENDS ${AppendageTemplate}
#             COMMENT "Copying ${name}"
#             VERBATIM)
#         list(APPEND copiedAppendageTemplates ${CMAKE_CURRENT_BINARY_DIR}/${name})
#     endforeach()
# endif()
