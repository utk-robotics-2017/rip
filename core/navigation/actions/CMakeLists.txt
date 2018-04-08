cmake_minimum_required(VERSION 3.1.0)
message("Building navigation actions")
project(navigation_actions)

file(GLOB_RECURSE ${PROJECT_NAME}_HEADERS "include/navigation_actions/*.hpp")
file(GLOB_RECURSE ${PROJECT_NAME}_SOURCES "src/*.cpp")

add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
target_link_libraries(${PROJECT_NAME} framework drivetrains motor_controllers navx pid)
target_include_directories(${PROJECT_NAME} PUBLIC include)
