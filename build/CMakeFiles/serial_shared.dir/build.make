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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/robocon/Desktop/Infer_color_lidar

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/robocon/Desktop/Infer_color_lidar/build

# Include any dependencies generated for this target.
include CMakeFiles/serial_shared.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/serial_shared.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/serial_shared.dir/flags.make

CMakeFiles/serial_shared.dir/SerialInterface.cpp.o: CMakeFiles/serial_shared.dir/flags.make
CMakeFiles/serial_shared.dir/SerialInterface.cpp.o: ../SerialInterface.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robocon/Desktop/Infer_color_lidar/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/serial_shared.dir/SerialInterface.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/serial_shared.dir/SerialInterface.cpp.o -c /home/robocon/Desktop/Infer_color_lidar/SerialInterface.cpp

CMakeFiles/serial_shared.dir/SerialInterface.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/serial_shared.dir/SerialInterface.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robocon/Desktop/Infer_color_lidar/SerialInterface.cpp > CMakeFiles/serial_shared.dir/SerialInterface.cpp.i

CMakeFiles/serial_shared.dir/SerialInterface.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/serial_shared.dir/SerialInterface.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robocon/Desktop/Infer_color_lidar/SerialInterface.cpp -o CMakeFiles/serial_shared.dir/SerialInterface.cpp.s

# Object files for target serial_shared
serial_shared_OBJECTS = \
"CMakeFiles/serial_shared.dir/SerialInterface.cpp.o"

# External object files for target serial_shared
serial_shared_EXTERNAL_OBJECTS =

libserial_shared.a: CMakeFiles/serial_shared.dir/SerialInterface.cpp.o
libserial_shared.a: CMakeFiles/serial_shared.dir/build.make
libserial_shared.a: CMakeFiles/serial_shared.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/robocon/Desktop/Infer_color_lidar/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libserial_shared.a"
	$(CMAKE_COMMAND) -P CMakeFiles/serial_shared.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/serial_shared.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/serial_shared.dir/build: libserial_shared.a

.PHONY : CMakeFiles/serial_shared.dir/build

CMakeFiles/serial_shared.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/serial_shared.dir/cmake_clean.cmake
.PHONY : CMakeFiles/serial_shared.dir/clean

CMakeFiles/serial_shared.dir/depend:
	cd /home/robocon/Desktop/Infer_color_lidar/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robocon/Desktop/Infer_color_lidar /home/robocon/Desktop/Infer_color_lidar /home/robocon/Desktop/Infer_color_lidar/build /home/robocon/Desktop/Infer_color_lidar/build /home/robocon/Desktop/Infer_color_lidar/build/CMakeFiles/serial_shared.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/serial_shared.dir/depend
