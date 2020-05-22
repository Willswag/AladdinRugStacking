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
CMAKE_SOURCE_DIR = /home/spencer/src/rugg-stacking-linux

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/spencer/src/rugg-stacking-linux

# Include any dependencies generated for this target.
include CMakeFiles/vision-server.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/vision-server.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/vision-server.dir/flags.make

CMakeFiles/vision-server.dir/vision-server.cpp.o: CMakeFiles/vision-server.dir/flags.make
CMakeFiles/vision-server.dir/vision-server.cpp.o: vision-server.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/spencer/src/rugg-stacking-linux/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/vision-server.dir/vision-server.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/vision-server.dir/vision-server.cpp.o -c /home/spencer/src/rugg-stacking-linux/vision-server.cpp

CMakeFiles/vision-server.dir/vision-server.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/vision-server.dir/vision-server.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/spencer/src/rugg-stacking-linux/vision-server.cpp > CMakeFiles/vision-server.dir/vision-server.cpp.i

CMakeFiles/vision-server.dir/vision-server.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/vision-server.dir/vision-server.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/spencer/src/rugg-stacking-linux/vision-server.cpp -o CMakeFiles/vision-server.dir/vision-server.cpp.s

CMakeFiles/vision-server.dir/vision-server.cpp.o.requires:

.PHONY : CMakeFiles/vision-server.dir/vision-server.cpp.o.requires

CMakeFiles/vision-server.dir/vision-server.cpp.o.provides: CMakeFiles/vision-server.dir/vision-server.cpp.o.requires
	$(MAKE) -f CMakeFiles/vision-server.dir/build.make CMakeFiles/vision-server.dir/vision-server.cpp.o.provides.build
.PHONY : CMakeFiles/vision-server.dir/vision-server.cpp.o.provides

CMakeFiles/vision-server.dir/vision-server.cpp.o.provides.build: CMakeFiles/vision-server.dir/vision-server.cpp.o


# Object files for target vision-server
vision__server_OBJECTS = \
"CMakeFiles/vision-server.dir/vision-server.cpp.o"

# External object files for target vision-server
vision__server_EXTERNAL_OBJECTS =

vision-server: CMakeFiles/vision-server.dir/vision-server.cpp.o
vision-server: CMakeFiles/vision-server.dir/build.make
vision-server: /usr/local/lib/libopencv_dnn.so.4.2.0
vision-server: /usr/local/lib/libopencv_gapi.so.4.2.0
vision-server: /usr/local/lib/libopencv_highgui.so.4.2.0
vision-server: /usr/local/lib/libopencv_ml.so.4.2.0
vision-server: /usr/local/lib/libopencv_objdetect.so.4.2.0
vision-server: /usr/local/lib/libopencv_photo.so.4.2.0
vision-server: /usr/local/lib/libopencv_stitching.so.4.2.0
vision-server: /usr/local/lib/libopencv_video.so.4.2.0
vision-server: /usr/local/lib/libopencv_videoio.so.4.2.0
vision-server: /usr/local/lib/libopencv_imgcodecs.so.4.2.0
vision-server: /usr/local/lib/libopencv_calib3d.so.4.2.0
vision-server: /usr/local/lib/libopencv_features2d.so.4.2.0
vision-server: /usr/local/lib/libopencv_flann.so.4.2.0
vision-server: /usr/local/lib/libopencv_imgproc.so.4.2.0
vision-server: /usr/local/lib/libopencv_core.so.4.2.0
vision-server: /usr/local/lib/libzmq.so.5.2.3
vision-server: CMakeFiles/vision-server.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/spencer/src/rugg-stacking-linux/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable vision-server"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/vision-server.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/vision-server.dir/build: vision-server

.PHONY : CMakeFiles/vision-server.dir/build

CMakeFiles/vision-server.dir/requires: CMakeFiles/vision-server.dir/vision-server.cpp.o.requires

.PHONY : CMakeFiles/vision-server.dir/requires

CMakeFiles/vision-server.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/vision-server.dir/cmake_clean.cmake
.PHONY : CMakeFiles/vision-server.dir/clean

CMakeFiles/vision-server.dir/depend:
	cd /home/spencer/src/rugg-stacking-linux && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/spencer/src/rugg-stacking-linux /home/spencer/src/rugg-stacking-linux /home/spencer/src/rugg-stacking-linux /home/spencer/src/rugg-stacking-linux /home/spencer/src/rugg-stacking-linux/CMakeFiles/vision-server.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/vision-server.dir/depend

