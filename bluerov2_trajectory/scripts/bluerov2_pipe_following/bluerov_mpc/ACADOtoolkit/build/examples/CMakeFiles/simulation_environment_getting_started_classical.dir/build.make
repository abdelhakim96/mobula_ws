# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/hakim/Desktop/ACADOtoolkit

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hakim/Desktop/ACADOtoolkit/build

# Include any dependencies generated for this target.
include examples/CMakeFiles/simulation_environment_getting_started_classical.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/simulation_environment_getting_started_classical.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/simulation_environment_getting_started_classical.dir/flags.make

examples/CMakeFiles/simulation_environment_getting_started_classical.dir/simulation_environment/getting_started_classical.cpp.o: examples/CMakeFiles/simulation_environment_getting_started_classical.dir/flags.make
examples/CMakeFiles/simulation_environment_getting_started_classical.dir/simulation_environment/getting_started_classical.cpp.o: ../examples/simulation_environment/getting_started_classical.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hakim/Desktop/ACADOtoolkit/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/simulation_environment_getting_started_classical.dir/simulation_environment/getting_started_classical.cpp.o"
	cd /home/hakim/Desktop/ACADOtoolkit/build/examples && /usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/simulation_environment_getting_started_classical.dir/simulation_environment/getting_started_classical.cpp.o -c /home/hakim/Desktop/ACADOtoolkit/examples/simulation_environment/getting_started_classical.cpp

examples/CMakeFiles/simulation_environment_getting_started_classical.dir/simulation_environment/getting_started_classical.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/simulation_environment_getting_started_classical.dir/simulation_environment/getting_started_classical.cpp.i"
	cd /home/hakim/Desktop/ACADOtoolkit/build/examples && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hakim/Desktop/ACADOtoolkit/examples/simulation_environment/getting_started_classical.cpp > CMakeFiles/simulation_environment_getting_started_classical.dir/simulation_environment/getting_started_classical.cpp.i

examples/CMakeFiles/simulation_environment_getting_started_classical.dir/simulation_environment/getting_started_classical.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/simulation_environment_getting_started_classical.dir/simulation_environment/getting_started_classical.cpp.s"
	cd /home/hakim/Desktop/ACADOtoolkit/build/examples && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hakim/Desktop/ACADOtoolkit/examples/simulation_environment/getting_started_classical.cpp -o CMakeFiles/simulation_environment_getting_started_classical.dir/simulation_environment/getting_started_classical.cpp.s

examples/CMakeFiles/simulation_environment_getting_started_classical.dir/simulation_environment/getting_started_classical.cpp.o.requires:

.PHONY : examples/CMakeFiles/simulation_environment_getting_started_classical.dir/simulation_environment/getting_started_classical.cpp.o.requires

examples/CMakeFiles/simulation_environment_getting_started_classical.dir/simulation_environment/getting_started_classical.cpp.o.provides: examples/CMakeFiles/simulation_environment_getting_started_classical.dir/simulation_environment/getting_started_classical.cpp.o.requires
	$(MAKE) -f examples/CMakeFiles/simulation_environment_getting_started_classical.dir/build.make examples/CMakeFiles/simulation_environment_getting_started_classical.dir/simulation_environment/getting_started_classical.cpp.o.provides.build
.PHONY : examples/CMakeFiles/simulation_environment_getting_started_classical.dir/simulation_environment/getting_started_classical.cpp.o.provides

examples/CMakeFiles/simulation_environment_getting_started_classical.dir/simulation_environment/getting_started_classical.cpp.o.provides.build: examples/CMakeFiles/simulation_environment_getting_started_classical.dir/simulation_environment/getting_started_classical.cpp.o


# Object files for target simulation_environment_getting_started_classical
simulation_environment_getting_started_classical_OBJECTS = \
"CMakeFiles/simulation_environment_getting_started_classical.dir/simulation_environment/getting_started_classical.cpp.o"

# External object files for target simulation_environment_getting_started_classical
simulation_environment_getting_started_classical_EXTERNAL_OBJECTS =

../examples/simulation_environment/getting_started_classical: examples/CMakeFiles/simulation_environment_getting_started_classical.dir/simulation_environment/getting_started_classical.cpp.o
../examples/simulation_environment/getting_started_classical: examples/CMakeFiles/simulation_environment_getting_started_classical.dir/build.make
../examples/simulation_environment/getting_started_classical: lib/libacado_toolkit_s.so.1.2.2beta
../examples/simulation_environment/getting_started_classical: lib/libacado_casadi.a
../examples/simulation_environment/getting_started_classical: lib/libacado_qpoases.a
../examples/simulation_environment/getting_started_classical: lib/libacado_csparse.a
../examples/simulation_environment/getting_started_classical: examples/CMakeFiles/simulation_environment_getting_started_classical.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hakim/Desktop/ACADOtoolkit/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../examples/simulation_environment/getting_started_classical"
	cd /home/hakim/Desktop/ACADOtoolkit/build/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/simulation_environment_getting_started_classical.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/simulation_environment_getting_started_classical.dir/build: ../examples/simulation_environment/getting_started_classical

.PHONY : examples/CMakeFiles/simulation_environment_getting_started_classical.dir/build

examples/CMakeFiles/simulation_environment_getting_started_classical.dir/requires: examples/CMakeFiles/simulation_environment_getting_started_classical.dir/simulation_environment/getting_started_classical.cpp.o.requires

.PHONY : examples/CMakeFiles/simulation_environment_getting_started_classical.dir/requires

examples/CMakeFiles/simulation_environment_getting_started_classical.dir/clean:
	cd /home/hakim/Desktop/ACADOtoolkit/build/examples && $(CMAKE_COMMAND) -P CMakeFiles/simulation_environment_getting_started_classical.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/simulation_environment_getting_started_classical.dir/clean

examples/CMakeFiles/simulation_environment_getting_started_classical.dir/depend:
	cd /home/hakim/Desktop/ACADOtoolkit/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hakim/Desktop/ACADOtoolkit /home/hakim/Desktop/ACADOtoolkit/examples /home/hakim/Desktop/ACADOtoolkit/build /home/hakim/Desktop/ACADOtoolkit/build/examples /home/hakim/Desktop/ACADOtoolkit/build/examples/CMakeFiles/simulation_environment_getting_started_classical.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/simulation_environment_getting_started_classical.dir/depend

