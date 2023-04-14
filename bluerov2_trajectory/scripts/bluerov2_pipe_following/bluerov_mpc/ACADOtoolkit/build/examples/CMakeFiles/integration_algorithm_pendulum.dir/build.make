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
include examples/CMakeFiles/integration_algorithm_pendulum.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/integration_algorithm_pendulum.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/integration_algorithm_pendulum.dir/flags.make

examples/CMakeFiles/integration_algorithm_pendulum.dir/integration_algorithm/pendulum.cpp.o: examples/CMakeFiles/integration_algorithm_pendulum.dir/flags.make
examples/CMakeFiles/integration_algorithm_pendulum.dir/integration_algorithm/pendulum.cpp.o: ../examples/integration_algorithm/pendulum.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hakim/Desktop/ACADOtoolkit/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/integration_algorithm_pendulum.dir/integration_algorithm/pendulum.cpp.o"
	cd /home/hakim/Desktop/ACADOtoolkit/build/examples && /usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/integration_algorithm_pendulum.dir/integration_algorithm/pendulum.cpp.o -c /home/hakim/Desktop/ACADOtoolkit/examples/integration_algorithm/pendulum.cpp

examples/CMakeFiles/integration_algorithm_pendulum.dir/integration_algorithm/pendulum.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/integration_algorithm_pendulum.dir/integration_algorithm/pendulum.cpp.i"
	cd /home/hakim/Desktop/ACADOtoolkit/build/examples && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hakim/Desktop/ACADOtoolkit/examples/integration_algorithm/pendulum.cpp > CMakeFiles/integration_algorithm_pendulum.dir/integration_algorithm/pendulum.cpp.i

examples/CMakeFiles/integration_algorithm_pendulum.dir/integration_algorithm/pendulum.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/integration_algorithm_pendulum.dir/integration_algorithm/pendulum.cpp.s"
	cd /home/hakim/Desktop/ACADOtoolkit/build/examples && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hakim/Desktop/ACADOtoolkit/examples/integration_algorithm/pendulum.cpp -o CMakeFiles/integration_algorithm_pendulum.dir/integration_algorithm/pendulum.cpp.s

examples/CMakeFiles/integration_algorithm_pendulum.dir/integration_algorithm/pendulum.cpp.o.requires:

.PHONY : examples/CMakeFiles/integration_algorithm_pendulum.dir/integration_algorithm/pendulum.cpp.o.requires

examples/CMakeFiles/integration_algorithm_pendulum.dir/integration_algorithm/pendulum.cpp.o.provides: examples/CMakeFiles/integration_algorithm_pendulum.dir/integration_algorithm/pendulum.cpp.o.requires
	$(MAKE) -f examples/CMakeFiles/integration_algorithm_pendulum.dir/build.make examples/CMakeFiles/integration_algorithm_pendulum.dir/integration_algorithm/pendulum.cpp.o.provides.build
.PHONY : examples/CMakeFiles/integration_algorithm_pendulum.dir/integration_algorithm/pendulum.cpp.o.provides

examples/CMakeFiles/integration_algorithm_pendulum.dir/integration_algorithm/pendulum.cpp.o.provides.build: examples/CMakeFiles/integration_algorithm_pendulum.dir/integration_algorithm/pendulum.cpp.o


# Object files for target integration_algorithm_pendulum
integration_algorithm_pendulum_OBJECTS = \
"CMakeFiles/integration_algorithm_pendulum.dir/integration_algorithm/pendulum.cpp.o"

# External object files for target integration_algorithm_pendulum
integration_algorithm_pendulum_EXTERNAL_OBJECTS =

../examples/integration_algorithm/pendulum: examples/CMakeFiles/integration_algorithm_pendulum.dir/integration_algorithm/pendulum.cpp.o
../examples/integration_algorithm/pendulum: examples/CMakeFiles/integration_algorithm_pendulum.dir/build.make
../examples/integration_algorithm/pendulum: lib/libacado_toolkit_s.so.1.2.2beta
../examples/integration_algorithm/pendulum: lib/libacado_casadi.a
../examples/integration_algorithm/pendulum: lib/libacado_qpoases.a
../examples/integration_algorithm/pendulum: lib/libacado_csparse.a
../examples/integration_algorithm/pendulum: examples/CMakeFiles/integration_algorithm_pendulum.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hakim/Desktop/ACADOtoolkit/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../examples/integration_algorithm/pendulum"
	cd /home/hakim/Desktop/ACADOtoolkit/build/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/integration_algorithm_pendulum.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/integration_algorithm_pendulum.dir/build: ../examples/integration_algorithm/pendulum

.PHONY : examples/CMakeFiles/integration_algorithm_pendulum.dir/build

examples/CMakeFiles/integration_algorithm_pendulum.dir/requires: examples/CMakeFiles/integration_algorithm_pendulum.dir/integration_algorithm/pendulum.cpp.o.requires

.PHONY : examples/CMakeFiles/integration_algorithm_pendulum.dir/requires

examples/CMakeFiles/integration_algorithm_pendulum.dir/clean:
	cd /home/hakim/Desktop/ACADOtoolkit/build/examples && $(CMAKE_COMMAND) -P CMakeFiles/integration_algorithm_pendulum.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/integration_algorithm_pendulum.dir/clean

examples/CMakeFiles/integration_algorithm_pendulum.dir/depend:
	cd /home/hakim/Desktop/ACADOtoolkit/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hakim/Desktop/ACADOtoolkit /home/hakim/Desktop/ACADOtoolkit/examples /home/hakim/Desktop/ACADOtoolkit/build /home/hakim/Desktop/ACADOtoolkit/build/examples /home/hakim/Desktop/ACADOtoolkit/build/examples/CMakeFiles/integration_algorithm_pendulum.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/integration_algorithm_pendulum.dir/depend

