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
include examples/CMakeFiles/simulation_environment_fourtankNMPC.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/simulation_environment_fourtankNMPC.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/simulation_environment_fourtankNMPC.dir/flags.make

examples/CMakeFiles/simulation_environment_fourtankNMPC.dir/simulation_environment/fourtankNMPC.cpp.o: examples/CMakeFiles/simulation_environment_fourtankNMPC.dir/flags.make
examples/CMakeFiles/simulation_environment_fourtankNMPC.dir/simulation_environment/fourtankNMPC.cpp.o: ../examples/simulation_environment/fourtankNMPC.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hakim/Desktop/ACADOtoolkit/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/simulation_environment_fourtankNMPC.dir/simulation_environment/fourtankNMPC.cpp.o"
	cd /home/hakim/Desktop/ACADOtoolkit/build/examples && /usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/simulation_environment_fourtankNMPC.dir/simulation_environment/fourtankNMPC.cpp.o -c /home/hakim/Desktop/ACADOtoolkit/examples/simulation_environment/fourtankNMPC.cpp

examples/CMakeFiles/simulation_environment_fourtankNMPC.dir/simulation_environment/fourtankNMPC.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/simulation_environment_fourtankNMPC.dir/simulation_environment/fourtankNMPC.cpp.i"
	cd /home/hakim/Desktop/ACADOtoolkit/build/examples && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hakim/Desktop/ACADOtoolkit/examples/simulation_environment/fourtankNMPC.cpp > CMakeFiles/simulation_environment_fourtankNMPC.dir/simulation_environment/fourtankNMPC.cpp.i

examples/CMakeFiles/simulation_environment_fourtankNMPC.dir/simulation_environment/fourtankNMPC.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/simulation_environment_fourtankNMPC.dir/simulation_environment/fourtankNMPC.cpp.s"
	cd /home/hakim/Desktop/ACADOtoolkit/build/examples && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hakim/Desktop/ACADOtoolkit/examples/simulation_environment/fourtankNMPC.cpp -o CMakeFiles/simulation_environment_fourtankNMPC.dir/simulation_environment/fourtankNMPC.cpp.s

examples/CMakeFiles/simulation_environment_fourtankNMPC.dir/simulation_environment/fourtankNMPC.cpp.o.requires:

.PHONY : examples/CMakeFiles/simulation_environment_fourtankNMPC.dir/simulation_environment/fourtankNMPC.cpp.o.requires

examples/CMakeFiles/simulation_environment_fourtankNMPC.dir/simulation_environment/fourtankNMPC.cpp.o.provides: examples/CMakeFiles/simulation_environment_fourtankNMPC.dir/simulation_environment/fourtankNMPC.cpp.o.requires
	$(MAKE) -f examples/CMakeFiles/simulation_environment_fourtankNMPC.dir/build.make examples/CMakeFiles/simulation_environment_fourtankNMPC.dir/simulation_environment/fourtankNMPC.cpp.o.provides.build
.PHONY : examples/CMakeFiles/simulation_environment_fourtankNMPC.dir/simulation_environment/fourtankNMPC.cpp.o.provides

examples/CMakeFiles/simulation_environment_fourtankNMPC.dir/simulation_environment/fourtankNMPC.cpp.o.provides.build: examples/CMakeFiles/simulation_environment_fourtankNMPC.dir/simulation_environment/fourtankNMPC.cpp.o


# Object files for target simulation_environment_fourtankNMPC
simulation_environment_fourtankNMPC_OBJECTS = \
"CMakeFiles/simulation_environment_fourtankNMPC.dir/simulation_environment/fourtankNMPC.cpp.o"

# External object files for target simulation_environment_fourtankNMPC
simulation_environment_fourtankNMPC_EXTERNAL_OBJECTS =

../examples/simulation_environment/fourtankNMPC: examples/CMakeFiles/simulation_environment_fourtankNMPC.dir/simulation_environment/fourtankNMPC.cpp.o
../examples/simulation_environment/fourtankNMPC: examples/CMakeFiles/simulation_environment_fourtankNMPC.dir/build.make
../examples/simulation_environment/fourtankNMPC: lib/libacado_toolkit_s.so.1.2.2beta
../examples/simulation_environment/fourtankNMPC: lib/libacado_casadi.a
../examples/simulation_environment/fourtankNMPC: lib/libacado_qpoases.a
../examples/simulation_environment/fourtankNMPC: lib/libacado_csparse.a
../examples/simulation_environment/fourtankNMPC: examples/CMakeFiles/simulation_environment_fourtankNMPC.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hakim/Desktop/ACADOtoolkit/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../examples/simulation_environment/fourtankNMPC"
	cd /home/hakim/Desktop/ACADOtoolkit/build/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/simulation_environment_fourtankNMPC.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/simulation_environment_fourtankNMPC.dir/build: ../examples/simulation_environment/fourtankNMPC

.PHONY : examples/CMakeFiles/simulation_environment_fourtankNMPC.dir/build

examples/CMakeFiles/simulation_environment_fourtankNMPC.dir/requires: examples/CMakeFiles/simulation_environment_fourtankNMPC.dir/simulation_environment/fourtankNMPC.cpp.o.requires

.PHONY : examples/CMakeFiles/simulation_environment_fourtankNMPC.dir/requires

examples/CMakeFiles/simulation_environment_fourtankNMPC.dir/clean:
	cd /home/hakim/Desktop/ACADOtoolkit/build/examples && $(CMAKE_COMMAND) -P CMakeFiles/simulation_environment_fourtankNMPC.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/simulation_environment_fourtankNMPC.dir/clean

examples/CMakeFiles/simulation_environment_fourtankNMPC.dir/depend:
	cd /home/hakim/Desktop/ACADOtoolkit/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hakim/Desktop/ACADOtoolkit /home/hakim/Desktop/ACADOtoolkit/examples /home/hakim/Desktop/ACADOtoolkit/build /home/hakim/Desktop/ACADOtoolkit/build/examples /home/hakim/Desktop/ACADOtoolkit/build/examples/CMakeFiles/simulation_environment_fourtankNMPC.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/simulation_environment_fourtankNMPC.dir/depend

