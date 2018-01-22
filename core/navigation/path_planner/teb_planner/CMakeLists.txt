cmake_minimum_required(VERSION 3.1)
message("Building teb path planner")
# define the project
project(teb_planner)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)
find_package(Eigen3 3.2 REQUIRED)
find_package(G2O REQUIRED)

# create and configure the library target
file(GLOB_RECURSE ${PROJECT_NAME}_SOURCES src/*.cpp)
file(GLOB_RECURSE ${PROJECT_NAME}_HEADERS include/teb_planner/*.hpp)
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
target_link_libraries(${PROJECT_NAME} fmt json units spdlog geometry ${Eigen3_LIBRARY} ${G2O_LIBRARY})
target_include_directories(${PROJECT_NAME} PUBLIC include)
