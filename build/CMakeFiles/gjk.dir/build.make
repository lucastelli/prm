# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

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
CMAKE_SOURCE_DIR = /home/luca/software/projectACA/project

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/luca/software/projectACA/project/build

# Include any dependencies generated for this target.
include CMakeFiles/gjk.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/gjk.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/gjk.dir/flags.make

CMakeFiles/gjk.dir/gjk.cpp.o: CMakeFiles/gjk.dir/flags.make
CMakeFiles/gjk.dir/gjk.cpp.o: ../gjk.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/luca/software/projectACA/project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/gjk.dir/gjk.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gjk.dir/gjk.cpp.o -c /home/luca/software/projectACA/project/gjk.cpp

CMakeFiles/gjk.dir/gjk.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gjk.dir/gjk.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/luca/software/projectACA/project/gjk.cpp > CMakeFiles/gjk.dir/gjk.cpp.i

CMakeFiles/gjk.dir/gjk.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gjk.dir/gjk.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/luca/software/projectACA/project/gjk.cpp -o CMakeFiles/gjk.dir/gjk.cpp.s

# Object files for target gjk
gjk_OBJECTS = \
"CMakeFiles/gjk.dir/gjk.cpp.o"

# External object files for target gjk
gjk_EXTERNAL_OBJECTS =

gjk: CMakeFiles/gjk.dir/gjk.cpp.o
gjk: CMakeFiles/gjk.dir/build.make
gjk: class/libclass.a
gjk: /usr/local/lib/libopencv_dnn.so.4.2.0
gjk: /usr/local/lib/libopencv_gapi.so.4.2.0
gjk: /usr/local/lib/libopencv_highgui.so.4.2.0
gjk: /usr/local/lib/libopencv_ml.so.4.2.0
gjk: /usr/local/lib/libopencv_objdetect.so.4.2.0
gjk: /usr/local/lib/libopencv_photo.so.4.2.0
gjk: /usr/local/lib/libopencv_stitching.so.4.2.0
gjk: /usr/local/lib/libopencv_video.so.4.2.0
gjk: /usr/local/lib/libopencv_videoio.so.4.2.0
gjk: functions/libfunctions.a
gjk: /usr/local/lib/libopencv_imgcodecs.so.4.2.0
gjk: /usr/local/lib/libopencv_calib3d.so.4.2.0
gjk: /usr/local/lib/libopencv_features2d.so.4.2.0
gjk: /usr/local/lib/libopencv_flann.so.4.2.0
gjk: /usr/local/lib/libopencv_imgproc.so.4.2.0
gjk: /usr/local/lib/libopencv_core.so.4.2.0
gjk: CMakeFiles/gjk.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/luca/software/projectACA/project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable gjk"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gjk.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/gjk.dir/build: gjk

.PHONY : CMakeFiles/gjk.dir/build

CMakeFiles/gjk.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/gjk.dir/cmake_clean.cmake
.PHONY : CMakeFiles/gjk.dir/clean

CMakeFiles/gjk.dir/depend:
	cd /home/luca/software/projectACA/project/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/luca/software/projectACA/project /home/luca/software/projectACA/project /home/luca/software/projectACA/project/build /home/luca/software/projectACA/project/build /home/luca/software/projectACA/project/build/CMakeFiles/gjk.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/gjk.dir/depend

