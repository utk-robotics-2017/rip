message("Adding cppfs")

cmake_minimum_required(VERSION 3.1)
project(cppfs)

file(GLOB ${PROJECT_NAME}_SOURCES "source/cppfs/source/*.cpp")

if (WIN32)
    file(GLOB winSources "source/cppfs/source/windows/*.cpp")
    list(APPEND ${PROJECT_NAME}_SOURCES ${winSources})
else (WIN32)
    file(GLOB posixSources "source/cppfs/source/posix/*.cpp")
    list(APPEND ${PROJECT_NAME}_SOURCES ${posixSources})
endif (WIN32)

add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES})
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
target_include_directories(${PROJECT_NAME} PUBLIC "source/cppfs/include")
target_include_directories(${PROJECT_NAME} PRIVATE "source/cppfs/3rdparty")
