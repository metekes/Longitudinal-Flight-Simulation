# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /mnt/c/Users/Lenovo/Desktop/JEDSY

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /mnt/c/Users/Lenovo/Desktop/JEDSY/build

# Include any dependencies generated for this target.
include CMakeFiles/controller.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/controller.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/controller.dir/flags.make

CMakeFiles/controller.dir/PD_controller.cpp.o: CMakeFiles/controller.dir/flags.make
CMakeFiles/controller.dir/PD_controller.cpp.o: ../PD_controller.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/c/Users/Lenovo/Desktop/JEDSY/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/controller.dir/PD_controller.cpp.o"
	/usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/controller.dir/PD_controller.cpp.o -c /mnt/c/Users/Lenovo/Desktop/JEDSY/PD_controller.cpp

CMakeFiles/controller.dir/PD_controller.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/controller.dir/PD_controller.cpp.i"
	/usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/c/Users/Lenovo/Desktop/JEDSY/PD_controller.cpp > CMakeFiles/controller.dir/PD_controller.cpp.i

CMakeFiles/controller.dir/PD_controller.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/controller.dir/PD_controller.cpp.s"
	/usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/c/Users/Lenovo/Desktop/JEDSY/PD_controller.cpp -o CMakeFiles/controller.dir/PD_controller.cpp.s

# Object files for target controller
controller_OBJECTS = \
"CMakeFiles/controller.dir/PD_controller.cpp.o"

# External object files for target controller
controller_EXTERNAL_OBJECTS =

libcontroller.a: CMakeFiles/controller.dir/PD_controller.cpp.o
libcontroller.a: CMakeFiles/controller.dir/build.make
libcontroller.a: CMakeFiles/controller.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/mnt/c/Users/Lenovo/Desktop/JEDSY/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libcontroller.a"
	$(CMAKE_COMMAND) -P CMakeFiles/controller.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/controller.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/controller.dir/build: libcontroller.a

.PHONY : CMakeFiles/controller.dir/build

CMakeFiles/controller.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/controller.dir/cmake_clean.cmake
.PHONY : CMakeFiles/controller.dir/clean

CMakeFiles/controller.dir/depend:
	cd /mnt/c/Users/Lenovo/Desktop/JEDSY/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /mnt/c/Users/Lenovo/Desktop/JEDSY /mnt/c/Users/Lenovo/Desktop/JEDSY /mnt/c/Users/Lenovo/Desktop/JEDSY/build /mnt/c/Users/Lenovo/Desktop/JEDSY/build /mnt/c/Users/Lenovo/Desktop/JEDSY/build/CMakeFiles/controller.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/controller.dir/depend

