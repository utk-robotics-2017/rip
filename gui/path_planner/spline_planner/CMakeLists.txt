cmake_minimum_required(VERSION 3.1)
project(spline_planner_gui)

# Instruct CMake to run moc automatically when needed.
set(AUTOMOC ON)
set(AUTORCC ON)
set(AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Find the Qt libraries
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

file(GLOB_RECURSE ${PROJECT_NAME}_SOURCES "src/*.cpp")
list(REMOVE_ITEM ${PROJECT_NAME}_SOURCES "src/main.cpp")

file(GLOB_RECURSE ${PROJECT_NAME}_HEADERS "include/spline_planner_gui/*.hpp")
QT5_WRAP_CPP(${PROJECT_NAME}_HEADER_MOCS ${${PROJECT_NAME}_HEADERS})


file(GLOB_RECURSE ${PROJECT_NAME}_GUI_UI "ui/*.ui")
QT5_WRAP_UI( ${PROJECT_NAME}_GUI_UI_HEADERS ${${PROJECT_NAME}_GUI_UI} )

file(GLOB_RECURSE ${PROJECT_NAME}_GUI_RESOURCES "resources/*.qrc")
QT5_ADD_RESOURCES( ${PROJECT_NAME}_GUI_UI_RESOURCES_RCC ${${PROJECT_NAME}_GUI_RESOURCES})

add_library(_${PROJECT_NAME} STATIC
    ${${PROJECT_NAME}_SOURCES}
    ${${PROJECT_NAME}_HEADERS}
    ${${PROJECT_NAME}_HEADER_MOCS}
    ${${PROJECT_NAME}_GUI_UI_HEADERS}
    ${${PROJECT_NAME}_GUI_UI_RESOURCES_RCC}
)
set_property(TARGET _${PROJECT_NAME} PROPERTY CXX_STANDARD 11)

target_include_directories(_${PROJECT_NAME} PUBLIC "include" ${CMAKE_BINARY_DIR})
target_link_libraries(_${PROJECT_NAME}
    fmt units spdlog stp geometry cppfs misc
    spline_planner
    Qt5::Core Qt5::Gui Qt5::Widgets
)

# Main ELF :linux:
add_executable(${PROJECT_NAME} src/main.cpp)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
target_link_libraries(${PROJECT_NAME} _${PROJECT_NAME} Qt5::Core Qt5::Gui)
