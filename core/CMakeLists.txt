cmake_minimum_required(VERSION 3.1)
message("Building core")

add_subdirectory(utilities)

add_subdirectory(framework)
add_subdirectory(motor_controllers)
add_subdirectory(navigation)
