# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/sebastian/Programming/rip

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/sebastian/Programming/rip

# Include any dependencies generated for this target.
include core/utilities/geometry/CMakeFiles/geometry.dir/depend.make

# Include the progress variables for this target.
include core/utilities/geometry/CMakeFiles/geometry.dir/progress.make

# Include the compile flags for this target's objects.
include core/utilities/geometry/CMakeFiles/geometry.dir/flags.make

core/utilities/geometry/CMakeFiles/geometry.dir/src/rectangle.cpp.o: core/utilities/geometry/CMakeFiles/geometry.dir/flags.make
core/utilities/geometry/CMakeFiles/geometry.dir/src/rectangle.cpp.o: core/utilities/geometry/src/rectangle.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sebastian/Programming/rip/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object core/utilities/geometry/CMakeFiles/geometry.dir/src/rectangle.cpp.o"
	cd /home/sebastian/Programming/rip/core/utilities/geometry && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/geometry.dir/src/rectangle.cpp.o -c /home/sebastian/Programming/rip/core/utilities/geometry/src/rectangle.cpp

core/utilities/geometry/CMakeFiles/geometry.dir/src/rectangle.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/geometry.dir/src/rectangle.cpp.i"
	cd /home/sebastian/Programming/rip/core/utilities/geometry && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sebastian/Programming/rip/core/utilities/geometry/src/rectangle.cpp > CMakeFiles/geometry.dir/src/rectangle.cpp.i

core/utilities/geometry/CMakeFiles/geometry.dir/src/rectangle.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/geometry.dir/src/rectangle.cpp.s"
	cd /home/sebastian/Programming/rip/core/utilities/geometry && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sebastian/Programming/rip/core/utilities/geometry/src/rectangle.cpp -o CMakeFiles/geometry.dir/src/rectangle.cpp.s

core/utilities/geometry/CMakeFiles/geometry.dir/src/rectangle.cpp.o.requires:

.PHONY : core/utilities/geometry/CMakeFiles/geometry.dir/src/rectangle.cpp.o.requires

core/utilities/geometry/CMakeFiles/geometry.dir/src/rectangle.cpp.o.provides: core/utilities/geometry/CMakeFiles/geometry.dir/src/rectangle.cpp.o.requires
	$(MAKE) -f core/utilities/geometry/CMakeFiles/geometry.dir/build.make core/utilities/geometry/CMakeFiles/geometry.dir/src/rectangle.cpp.o.provides.build
.PHONY : core/utilities/geometry/CMakeFiles/geometry.dir/src/rectangle.cpp.o.provides

core/utilities/geometry/CMakeFiles/geometry.dir/src/rectangle.cpp.o.provides.build: core/utilities/geometry/CMakeFiles/geometry.dir/src/rectangle.cpp.o


core/utilities/geometry/CMakeFiles/geometry.dir/src/geometry_utils.cpp.o: core/utilities/geometry/CMakeFiles/geometry.dir/flags.make
core/utilities/geometry/CMakeFiles/geometry.dir/src/geometry_utils.cpp.o: core/utilities/geometry/src/geometry_utils.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sebastian/Programming/rip/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object core/utilities/geometry/CMakeFiles/geometry.dir/src/geometry_utils.cpp.o"
	cd /home/sebastian/Programming/rip/core/utilities/geometry && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/geometry.dir/src/geometry_utils.cpp.o -c /home/sebastian/Programming/rip/core/utilities/geometry/src/geometry_utils.cpp

core/utilities/geometry/CMakeFiles/geometry.dir/src/geometry_utils.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/geometry.dir/src/geometry_utils.cpp.i"
	cd /home/sebastian/Programming/rip/core/utilities/geometry && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sebastian/Programming/rip/core/utilities/geometry/src/geometry_utils.cpp > CMakeFiles/geometry.dir/src/geometry_utils.cpp.i

core/utilities/geometry/CMakeFiles/geometry.dir/src/geometry_utils.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/geometry.dir/src/geometry_utils.cpp.s"
	cd /home/sebastian/Programming/rip/core/utilities/geometry && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sebastian/Programming/rip/core/utilities/geometry/src/geometry_utils.cpp -o CMakeFiles/geometry.dir/src/geometry_utils.cpp.s

core/utilities/geometry/CMakeFiles/geometry.dir/src/geometry_utils.cpp.o.requires:

.PHONY : core/utilities/geometry/CMakeFiles/geometry.dir/src/geometry_utils.cpp.o.requires

core/utilities/geometry/CMakeFiles/geometry.dir/src/geometry_utils.cpp.o.provides: core/utilities/geometry/CMakeFiles/geometry.dir/src/geometry_utils.cpp.o.requires
	$(MAKE) -f core/utilities/geometry/CMakeFiles/geometry.dir/build.make core/utilities/geometry/CMakeFiles/geometry.dir/src/geometry_utils.cpp.o.provides.build
.PHONY : core/utilities/geometry/CMakeFiles/geometry.dir/src/geometry_utils.cpp.o.provides

core/utilities/geometry/CMakeFiles/geometry.dir/src/geometry_utils.cpp.o.provides.build: core/utilities/geometry/CMakeFiles/geometry.dir/src/geometry_utils.cpp.o


core/utilities/geometry/CMakeFiles/geometry.dir/src/polygon_list.cpp.o: core/utilities/geometry/CMakeFiles/geometry.dir/flags.make
core/utilities/geometry/CMakeFiles/geometry.dir/src/polygon_list.cpp.o: core/utilities/geometry/src/polygon_list.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sebastian/Programming/rip/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object core/utilities/geometry/CMakeFiles/geometry.dir/src/polygon_list.cpp.o"
	cd /home/sebastian/Programming/rip/core/utilities/geometry && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/geometry.dir/src/polygon_list.cpp.o -c /home/sebastian/Programming/rip/core/utilities/geometry/src/polygon_list.cpp

core/utilities/geometry/CMakeFiles/geometry.dir/src/polygon_list.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/geometry.dir/src/polygon_list.cpp.i"
	cd /home/sebastian/Programming/rip/core/utilities/geometry && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sebastian/Programming/rip/core/utilities/geometry/src/polygon_list.cpp > CMakeFiles/geometry.dir/src/polygon_list.cpp.i

core/utilities/geometry/CMakeFiles/geometry.dir/src/polygon_list.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/geometry.dir/src/polygon_list.cpp.s"
	cd /home/sebastian/Programming/rip/core/utilities/geometry && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sebastian/Programming/rip/core/utilities/geometry/src/polygon_list.cpp -o CMakeFiles/geometry.dir/src/polygon_list.cpp.s

core/utilities/geometry/CMakeFiles/geometry.dir/src/polygon_list.cpp.o.requires:

.PHONY : core/utilities/geometry/CMakeFiles/geometry.dir/src/polygon_list.cpp.o.requires

core/utilities/geometry/CMakeFiles/geometry.dir/src/polygon_list.cpp.o.provides: core/utilities/geometry/CMakeFiles/geometry.dir/src/polygon_list.cpp.o.requires
	$(MAKE) -f core/utilities/geometry/CMakeFiles/geometry.dir/build.make core/utilities/geometry/CMakeFiles/geometry.dir/src/polygon_list.cpp.o.provides.build
.PHONY : core/utilities/geometry/CMakeFiles/geometry.dir/src/polygon_list.cpp.o.provides

core/utilities/geometry/CMakeFiles/geometry.dir/src/polygon_list.cpp.o.provides.build: core/utilities/geometry/CMakeFiles/geometry.dir/src/polygon_list.cpp.o


core/utilities/geometry/CMakeFiles/geometry.dir/src/point.cpp.o: core/utilities/geometry/CMakeFiles/geometry.dir/flags.make
core/utilities/geometry/CMakeFiles/geometry.dir/src/point.cpp.o: core/utilities/geometry/src/point.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sebastian/Programming/rip/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object core/utilities/geometry/CMakeFiles/geometry.dir/src/point.cpp.o"
	cd /home/sebastian/Programming/rip/core/utilities/geometry && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/geometry.dir/src/point.cpp.o -c /home/sebastian/Programming/rip/core/utilities/geometry/src/point.cpp

core/utilities/geometry/CMakeFiles/geometry.dir/src/point.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/geometry.dir/src/point.cpp.i"
	cd /home/sebastian/Programming/rip/core/utilities/geometry && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sebastian/Programming/rip/core/utilities/geometry/src/point.cpp > CMakeFiles/geometry.dir/src/point.cpp.i

core/utilities/geometry/CMakeFiles/geometry.dir/src/point.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/geometry.dir/src/point.cpp.s"
	cd /home/sebastian/Programming/rip/core/utilities/geometry && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sebastian/Programming/rip/core/utilities/geometry/src/point.cpp -o CMakeFiles/geometry.dir/src/point.cpp.s

core/utilities/geometry/CMakeFiles/geometry.dir/src/point.cpp.o.requires:

.PHONY : core/utilities/geometry/CMakeFiles/geometry.dir/src/point.cpp.o.requires

core/utilities/geometry/CMakeFiles/geometry.dir/src/point.cpp.o.provides: core/utilities/geometry/CMakeFiles/geometry.dir/src/point.cpp.o.requires
	$(MAKE) -f core/utilities/geometry/CMakeFiles/geometry.dir/build.make core/utilities/geometry/CMakeFiles/geometry.dir/src/point.cpp.o.provides.build
.PHONY : core/utilities/geometry/CMakeFiles/geometry.dir/src/point.cpp.o.provides

core/utilities/geometry/CMakeFiles/geometry.dir/src/point.cpp.o.provides.build: core/utilities/geometry/CMakeFiles/geometry.dir/src/point.cpp.o


core/utilities/geometry/CMakeFiles/geometry.dir/src/polygon.cpp.o: core/utilities/geometry/CMakeFiles/geometry.dir/flags.make
core/utilities/geometry/CMakeFiles/geometry.dir/src/polygon.cpp.o: core/utilities/geometry/src/polygon.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sebastian/Programming/rip/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object core/utilities/geometry/CMakeFiles/geometry.dir/src/polygon.cpp.o"
	cd /home/sebastian/Programming/rip/core/utilities/geometry && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/geometry.dir/src/polygon.cpp.o -c /home/sebastian/Programming/rip/core/utilities/geometry/src/polygon.cpp

core/utilities/geometry/CMakeFiles/geometry.dir/src/polygon.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/geometry.dir/src/polygon.cpp.i"
	cd /home/sebastian/Programming/rip/core/utilities/geometry && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sebastian/Programming/rip/core/utilities/geometry/src/polygon.cpp > CMakeFiles/geometry.dir/src/polygon.cpp.i

core/utilities/geometry/CMakeFiles/geometry.dir/src/polygon.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/geometry.dir/src/polygon.cpp.s"
	cd /home/sebastian/Programming/rip/core/utilities/geometry && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sebastian/Programming/rip/core/utilities/geometry/src/polygon.cpp -o CMakeFiles/geometry.dir/src/polygon.cpp.s

core/utilities/geometry/CMakeFiles/geometry.dir/src/polygon.cpp.o.requires:

.PHONY : core/utilities/geometry/CMakeFiles/geometry.dir/src/polygon.cpp.o.requires

core/utilities/geometry/CMakeFiles/geometry.dir/src/polygon.cpp.o.provides: core/utilities/geometry/CMakeFiles/geometry.dir/src/polygon.cpp.o.requires
	$(MAKE) -f core/utilities/geometry/CMakeFiles/geometry.dir/build.make core/utilities/geometry/CMakeFiles/geometry.dir/src/polygon.cpp.o.provides.build
.PHONY : core/utilities/geometry/CMakeFiles/geometry.dir/src/polygon.cpp.o.provides

core/utilities/geometry/CMakeFiles/geometry.dir/src/polygon.cpp.o.provides.build: core/utilities/geometry/CMakeFiles/geometry.dir/src/polygon.cpp.o


core/utilities/geometry/CMakeFiles/geometry.dir/src/circle.cpp.o: core/utilities/geometry/CMakeFiles/geometry.dir/flags.make
core/utilities/geometry/CMakeFiles/geometry.dir/src/circle.cpp.o: core/utilities/geometry/src/circle.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sebastian/Programming/rip/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object core/utilities/geometry/CMakeFiles/geometry.dir/src/circle.cpp.o"
	cd /home/sebastian/Programming/rip/core/utilities/geometry && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/geometry.dir/src/circle.cpp.o -c /home/sebastian/Programming/rip/core/utilities/geometry/src/circle.cpp

core/utilities/geometry/CMakeFiles/geometry.dir/src/circle.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/geometry.dir/src/circle.cpp.i"
	cd /home/sebastian/Programming/rip/core/utilities/geometry && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sebastian/Programming/rip/core/utilities/geometry/src/circle.cpp > CMakeFiles/geometry.dir/src/circle.cpp.i

core/utilities/geometry/CMakeFiles/geometry.dir/src/circle.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/geometry.dir/src/circle.cpp.s"
	cd /home/sebastian/Programming/rip/core/utilities/geometry && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sebastian/Programming/rip/core/utilities/geometry/src/circle.cpp -o CMakeFiles/geometry.dir/src/circle.cpp.s

core/utilities/geometry/CMakeFiles/geometry.dir/src/circle.cpp.o.requires:

.PHONY : core/utilities/geometry/CMakeFiles/geometry.dir/src/circle.cpp.o.requires

core/utilities/geometry/CMakeFiles/geometry.dir/src/circle.cpp.o.provides: core/utilities/geometry/CMakeFiles/geometry.dir/src/circle.cpp.o.requires
	$(MAKE) -f core/utilities/geometry/CMakeFiles/geometry.dir/build.make core/utilities/geometry/CMakeFiles/geometry.dir/src/circle.cpp.o.provides.build
.PHONY : core/utilities/geometry/CMakeFiles/geometry.dir/src/circle.cpp.o.provides

core/utilities/geometry/CMakeFiles/geometry.dir/src/circle.cpp.o.provides.build: core/utilities/geometry/CMakeFiles/geometry.dir/src/circle.cpp.o


# Object files for target geometry
geometry_OBJECTS = \
"CMakeFiles/geometry.dir/src/rectangle.cpp.o" \
"CMakeFiles/geometry.dir/src/geometry_utils.cpp.o" \
"CMakeFiles/geometry.dir/src/polygon_list.cpp.o" \
"CMakeFiles/geometry.dir/src/point.cpp.o" \
"CMakeFiles/geometry.dir/src/polygon.cpp.o" \
"CMakeFiles/geometry.dir/src/circle.cpp.o"

# External object files for target geometry
geometry_EXTERNAL_OBJECTS =

core/utilities/geometry/libgeometry.a: core/utilities/geometry/CMakeFiles/geometry.dir/src/rectangle.cpp.o
core/utilities/geometry/libgeometry.a: core/utilities/geometry/CMakeFiles/geometry.dir/src/geometry_utils.cpp.o
core/utilities/geometry/libgeometry.a: core/utilities/geometry/CMakeFiles/geometry.dir/src/polygon_list.cpp.o
core/utilities/geometry/libgeometry.a: core/utilities/geometry/CMakeFiles/geometry.dir/src/point.cpp.o
core/utilities/geometry/libgeometry.a: core/utilities/geometry/CMakeFiles/geometry.dir/src/polygon.cpp.o
core/utilities/geometry/libgeometry.a: core/utilities/geometry/CMakeFiles/geometry.dir/src/circle.cpp.o
core/utilities/geometry/libgeometry.a: core/utilities/geometry/CMakeFiles/geometry.dir/build.make
core/utilities/geometry/libgeometry.a: core/utilities/geometry/CMakeFiles/geometry.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/sebastian/Programming/rip/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX static library libgeometry.a"
	cd /home/sebastian/Programming/rip/core/utilities/geometry && $(CMAKE_COMMAND) -P CMakeFiles/geometry.dir/cmake_clean_target.cmake
	cd /home/sebastian/Programming/rip/core/utilities/geometry && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/geometry.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
core/utilities/geometry/CMakeFiles/geometry.dir/build: core/utilities/geometry/libgeometry.a

.PHONY : core/utilities/geometry/CMakeFiles/geometry.dir/build

core/utilities/geometry/CMakeFiles/geometry.dir/requires: core/utilities/geometry/CMakeFiles/geometry.dir/src/rectangle.cpp.o.requires
core/utilities/geometry/CMakeFiles/geometry.dir/requires: core/utilities/geometry/CMakeFiles/geometry.dir/src/geometry_utils.cpp.o.requires
core/utilities/geometry/CMakeFiles/geometry.dir/requires: core/utilities/geometry/CMakeFiles/geometry.dir/src/polygon_list.cpp.o.requires
core/utilities/geometry/CMakeFiles/geometry.dir/requires: core/utilities/geometry/CMakeFiles/geometry.dir/src/point.cpp.o.requires
core/utilities/geometry/CMakeFiles/geometry.dir/requires: core/utilities/geometry/CMakeFiles/geometry.dir/src/polygon.cpp.o.requires
core/utilities/geometry/CMakeFiles/geometry.dir/requires: core/utilities/geometry/CMakeFiles/geometry.dir/src/circle.cpp.o.requires

.PHONY : core/utilities/geometry/CMakeFiles/geometry.dir/requires

core/utilities/geometry/CMakeFiles/geometry.dir/clean:
	cd /home/sebastian/Programming/rip/core/utilities/geometry && $(CMAKE_COMMAND) -P CMakeFiles/geometry.dir/cmake_clean.cmake
.PHONY : core/utilities/geometry/CMakeFiles/geometry.dir/clean

core/utilities/geometry/CMakeFiles/geometry.dir/depend:
	cd /home/sebastian/Programming/rip && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sebastian/Programming/rip /home/sebastian/Programming/rip/core/utilities/geometry /home/sebastian/Programming/rip /home/sebastian/Programming/rip/core/utilities/geometry /home/sebastian/Programming/rip/core/utilities/geometry/CMakeFiles/geometry.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : core/utilities/geometry/CMakeFiles/geometry.dir/depend

