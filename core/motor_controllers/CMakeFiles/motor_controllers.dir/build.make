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
include core/motor_controllers/CMakeFiles/motor_controllers.dir/depend.make

# Include the progress variables for this target.
include core/motor_controllers/CMakeFiles/motor_controllers.dir/progress.make

# Include the compile flags for this target's objects.
include core/motor_controllers/CMakeFiles/motor_controllers.dir/flags.make

core/motor_controllers/CMakeFiles/motor_controllers.dir/src/motor_dynamics.cpp.o: core/motor_controllers/CMakeFiles/motor_controllers.dir/flags.make
core/motor_controllers/CMakeFiles/motor_controllers.dir/src/motor_dynamics.cpp.o: core/motor_controllers/src/motor_dynamics.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sebastian/Programming/rip/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object core/motor_controllers/CMakeFiles/motor_controllers.dir/src/motor_dynamics.cpp.o"
	cd /home/sebastian/Programming/rip/core/motor_controllers && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/motor_controllers.dir/src/motor_dynamics.cpp.o -c /home/sebastian/Programming/rip/core/motor_controllers/src/motor_dynamics.cpp

core/motor_controllers/CMakeFiles/motor_controllers.dir/src/motor_dynamics.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/motor_controllers.dir/src/motor_dynamics.cpp.i"
	cd /home/sebastian/Programming/rip/core/motor_controllers && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sebastian/Programming/rip/core/motor_controllers/src/motor_dynamics.cpp > CMakeFiles/motor_controllers.dir/src/motor_dynamics.cpp.i

core/motor_controllers/CMakeFiles/motor_controllers.dir/src/motor_dynamics.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/motor_controllers.dir/src/motor_dynamics.cpp.s"
	cd /home/sebastian/Programming/rip/core/motor_controllers && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sebastian/Programming/rip/core/motor_controllers/src/motor_dynamics.cpp -o CMakeFiles/motor_controllers.dir/src/motor_dynamics.cpp.s

core/motor_controllers/CMakeFiles/motor_controllers.dir/src/motor_dynamics.cpp.o.requires:

.PHONY : core/motor_controllers/CMakeFiles/motor_controllers.dir/src/motor_dynamics.cpp.o.requires

core/motor_controllers/CMakeFiles/motor_controllers.dir/src/motor_dynamics.cpp.o.provides: core/motor_controllers/CMakeFiles/motor_controllers.dir/src/motor_dynamics.cpp.o.requires
	$(MAKE) -f core/motor_controllers/CMakeFiles/motor_controllers.dir/build.make core/motor_controllers/CMakeFiles/motor_controllers.dir/src/motor_dynamics.cpp.o.provides.build
.PHONY : core/motor_controllers/CMakeFiles/motor_controllers.dir/src/motor_dynamics.cpp.o.provides

core/motor_controllers/CMakeFiles/motor_controllers.dir/src/motor_dynamics.cpp.o.provides.build: core/motor_controllers/CMakeFiles/motor_controllers.dir/src/motor_dynamics.cpp.o


core/motor_controllers/CMakeFiles/motor_controllers.dir/src/roboclaw.cpp.o: core/motor_controllers/CMakeFiles/motor_controllers.dir/flags.make
core/motor_controllers/CMakeFiles/motor_controllers.dir/src/roboclaw.cpp.o: core/motor_controllers/src/roboclaw.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sebastian/Programming/rip/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object core/motor_controllers/CMakeFiles/motor_controllers.dir/src/roboclaw.cpp.o"
	cd /home/sebastian/Programming/rip/core/motor_controllers && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/motor_controllers.dir/src/roboclaw.cpp.o -c /home/sebastian/Programming/rip/core/motor_controllers/src/roboclaw.cpp

core/motor_controllers/CMakeFiles/motor_controllers.dir/src/roboclaw.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/motor_controllers.dir/src/roboclaw.cpp.i"
	cd /home/sebastian/Programming/rip/core/motor_controllers && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sebastian/Programming/rip/core/motor_controllers/src/roboclaw.cpp > CMakeFiles/motor_controllers.dir/src/roboclaw.cpp.i

core/motor_controllers/CMakeFiles/motor_controllers.dir/src/roboclaw.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/motor_controllers.dir/src/roboclaw.cpp.s"
	cd /home/sebastian/Programming/rip/core/motor_controllers && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sebastian/Programming/rip/core/motor_controllers/src/roboclaw.cpp -o CMakeFiles/motor_controllers.dir/src/roboclaw.cpp.s

core/motor_controllers/CMakeFiles/motor_controllers.dir/src/roboclaw.cpp.o.requires:

.PHONY : core/motor_controllers/CMakeFiles/motor_controllers.dir/src/roboclaw.cpp.o.requires

core/motor_controllers/CMakeFiles/motor_controllers.dir/src/roboclaw.cpp.o.provides: core/motor_controllers/CMakeFiles/motor_controllers.dir/src/roboclaw.cpp.o.requires
	$(MAKE) -f core/motor_controllers/CMakeFiles/motor_controllers.dir/build.make core/motor_controllers/CMakeFiles/motor_controllers.dir/src/roboclaw.cpp.o.provides.build
.PHONY : core/motor_controllers/CMakeFiles/motor_controllers.dir/src/roboclaw.cpp.o.provides

core/motor_controllers/CMakeFiles/motor_controllers.dir/src/roboclaw.cpp.o.provides.build: core/motor_controllers/CMakeFiles/motor_controllers.dir/src/roboclaw.cpp.o


core/motor_controllers/CMakeFiles/motor_controllers.dir/src/config.cpp.o: core/motor_controllers/CMakeFiles/motor_controllers.dir/flags.make
core/motor_controllers/CMakeFiles/motor_controllers.dir/src/config.cpp.o: core/motor_controllers/src/config.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sebastian/Programming/rip/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object core/motor_controllers/CMakeFiles/motor_controllers.dir/src/config.cpp.o"
	cd /home/sebastian/Programming/rip/core/motor_controllers && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/motor_controllers.dir/src/config.cpp.o -c /home/sebastian/Programming/rip/core/motor_controllers/src/config.cpp

core/motor_controllers/CMakeFiles/motor_controllers.dir/src/config.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/motor_controllers.dir/src/config.cpp.i"
	cd /home/sebastian/Programming/rip/core/motor_controllers && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sebastian/Programming/rip/core/motor_controllers/src/config.cpp > CMakeFiles/motor_controllers.dir/src/config.cpp.i

core/motor_controllers/CMakeFiles/motor_controllers.dir/src/config.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/motor_controllers.dir/src/config.cpp.s"
	cd /home/sebastian/Programming/rip/core/motor_controllers && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sebastian/Programming/rip/core/motor_controllers/src/config.cpp -o CMakeFiles/motor_controllers.dir/src/config.cpp.s

core/motor_controllers/CMakeFiles/motor_controllers.dir/src/config.cpp.o.requires:

.PHONY : core/motor_controllers/CMakeFiles/motor_controllers.dir/src/config.cpp.o.requires

core/motor_controllers/CMakeFiles/motor_controllers.dir/src/config.cpp.o.provides: core/motor_controllers/CMakeFiles/motor_controllers.dir/src/config.cpp.o.requires
	$(MAKE) -f core/motor_controllers/CMakeFiles/motor_controllers.dir/build.make core/motor_controllers/CMakeFiles/motor_controllers.dir/src/config.cpp.o.provides.build
.PHONY : core/motor_controllers/CMakeFiles/motor_controllers.dir/src/config.cpp.o.provides

core/motor_controllers/CMakeFiles/motor_controllers.dir/src/config.cpp.o.provides.build: core/motor_controllers/CMakeFiles/motor_controllers.dir/src/config.cpp.o


# Object files for target motor_controllers
motor_controllers_OBJECTS = \
"CMakeFiles/motor_controllers.dir/src/motor_dynamics.cpp.o" \
"CMakeFiles/motor_controllers.dir/src/roboclaw.cpp.o" \
"CMakeFiles/motor_controllers.dir/src/config.cpp.o"

# External object files for target motor_controllers
motor_controllers_EXTERNAL_OBJECTS =

core/motor_controllers/libmotor_controllers.a: core/motor_controllers/CMakeFiles/motor_controllers.dir/src/motor_dynamics.cpp.o
core/motor_controllers/libmotor_controllers.a: core/motor_controllers/CMakeFiles/motor_controllers.dir/src/roboclaw.cpp.o
core/motor_controllers/libmotor_controllers.a: core/motor_controllers/CMakeFiles/motor_controllers.dir/src/config.cpp.o
core/motor_controllers/libmotor_controllers.a: core/motor_controllers/CMakeFiles/motor_controllers.dir/build.make
core/motor_controllers/libmotor_controllers.a: core/motor_controllers/CMakeFiles/motor_controllers.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/sebastian/Programming/rip/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX static library libmotor_controllers.a"
	cd /home/sebastian/Programming/rip/core/motor_controllers && $(CMAKE_COMMAND) -P CMakeFiles/motor_controllers.dir/cmake_clean_target.cmake
	cd /home/sebastian/Programming/rip/core/motor_controllers && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/motor_controllers.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
core/motor_controllers/CMakeFiles/motor_controllers.dir/build: core/motor_controllers/libmotor_controllers.a

.PHONY : core/motor_controllers/CMakeFiles/motor_controllers.dir/build

core/motor_controllers/CMakeFiles/motor_controllers.dir/requires: core/motor_controllers/CMakeFiles/motor_controllers.dir/src/motor_dynamics.cpp.o.requires
core/motor_controllers/CMakeFiles/motor_controllers.dir/requires: core/motor_controllers/CMakeFiles/motor_controllers.dir/src/roboclaw.cpp.o.requires
core/motor_controllers/CMakeFiles/motor_controllers.dir/requires: core/motor_controllers/CMakeFiles/motor_controllers.dir/src/config.cpp.o.requires

.PHONY : core/motor_controllers/CMakeFiles/motor_controllers.dir/requires

core/motor_controllers/CMakeFiles/motor_controllers.dir/clean:
	cd /home/sebastian/Programming/rip/core/motor_controllers && $(CMAKE_COMMAND) -P CMakeFiles/motor_controllers.dir/cmake_clean.cmake
.PHONY : core/motor_controllers/CMakeFiles/motor_controllers.dir/clean

core/motor_controllers/CMakeFiles/motor_controllers.dir/depend:
	cd /home/sebastian/Programming/rip && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sebastian/Programming/rip /home/sebastian/Programming/rip/core/motor_controllers /home/sebastian/Programming/rip /home/sebastian/Programming/rip/core/motor_controllers /home/sebastian/Programming/rip/core/motor_controllers/CMakeFiles/motor_controllers.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : core/motor_controllers/CMakeFiles/motor_controllers.dir/depend
