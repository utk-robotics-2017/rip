cmake_minimum_required(VERSION 3.1)
project(appendages)

if(ALL OR RIP OR RIPCOM)
	file(GLOB_RECURSE ${PROJECT_NAME}_SOURCES "src/*.cpp")

	add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES})
	set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
	target_link_libraries(${PROJECT_NAME} fmt json spdlog pathman)
	target_link_libraries(${PROJECT_NAME} cmd_messenger)
	target_include_directories(${PROJECT_NAME} PUBLIC include)
endif()

if(ALL OR ARDUINO_GEN)
	file(GLOB_RECURSE AppendageTemplateFiles "xml/*.xml" "json/*.json")
	add_custom_target(AppendageTemplates SOURCES ${AppendageTemplateFiles})
	foreach(AppendageTemplate ${AppendageTemplateFiles})
		file(RELATIVE_PATH name ${CMAKE_CURRENT_SOURCE_DIR} ${AppendageTemplate})
		add_custom_command(TARGET AppendageTemplates
			COMMAND ${CMAKE_COMMAND} -E copy_if_different ${AppendageTemplate} $<TARGET_FILE_DIR:_arduino_gen>/appendages/${name}
			DEPENDS ${AppendageTemplate} COMMENT "Copying appendages/${name}" VERBATIM POST_BUILD)
	endforeach()
endif()
