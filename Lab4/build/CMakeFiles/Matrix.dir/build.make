# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/matthew/CSC232/Lab4

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/matthew/CSC232/Lab4/build

# Utility rule file for Matrix.

# Include the progress variables for this target.
include CMakeFiles/Matrix.dir/progress.make

CMakeFiles/Matrix:

Matrix: CMakeFiles/Matrix
Matrix: CMakeFiles/Matrix.dir/build.make
.PHONY : Matrix

# Rule to build all files generated by this target.
CMakeFiles/Matrix.dir/build: Matrix
.PHONY : CMakeFiles/Matrix.dir/build

CMakeFiles/Matrix.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Matrix.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Matrix.dir/clean

CMakeFiles/Matrix.dir/depend:
	cd /home/matthew/CSC232/Lab4/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/matthew/CSC232/Lab4 /home/matthew/CSC232/Lab4 /home/matthew/CSC232/Lab4/build /home/matthew/CSC232/Lab4/build /home/matthew/CSC232/Lab4/build/CMakeFiles/Matrix.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Matrix.dir/depend
