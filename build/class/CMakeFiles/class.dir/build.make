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
include class/CMakeFiles/class.dir/depend.make

# Include the progress variables for this target.
include class/CMakeFiles/class.dir/progress.make

# Include the compile flags for this target's objects.
include class/CMakeFiles/class.dir/flags.make

class/CMakeFiles/class.dir/joint.cpp.o: class/CMakeFiles/class.dir/flags.make
class/CMakeFiles/class.dir/joint.cpp.o: ../class/joint.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/luca/software/projectACA/project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object class/CMakeFiles/class.dir/joint.cpp.o"
	cd /home/luca/software/projectACA/project/build/class && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/class.dir/joint.cpp.o -c /home/luca/software/projectACA/project/class/joint.cpp

class/CMakeFiles/class.dir/joint.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/class.dir/joint.cpp.i"
	cd /home/luca/software/projectACA/project/build/class && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/luca/software/projectACA/project/class/joint.cpp > CMakeFiles/class.dir/joint.cpp.i

class/CMakeFiles/class.dir/joint.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/class.dir/joint.cpp.s"
	cd /home/luca/software/projectACA/project/build/class && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/luca/software/projectACA/project/class/joint.cpp -o CMakeFiles/class.dir/joint.cpp.s

class/CMakeFiles/class.dir/linear.cpp.o: class/CMakeFiles/class.dir/flags.make
class/CMakeFiles/class.dir/linear.cpp.o: ../class/linear.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/luca/software/projectACA/project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object class/CMakeFiles/class.dir/linear.cpp.o"
	cd /home/luca/software/projectACA/project/build/class && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/class.dir/linear.cpp.o -c /home/luca/software/projectACA/project/class/linear.cpp

class/CMakeFiles/class.dir/linear.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/class.dir/linear.cpp.i"
	cd /home/luca/software/projectACA/project/build/class && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/luca/software/projectACA/project/class/linear.cpp > CMakeFiles/class.dir/linear.cpp.i

class/CMakeFiles/class.dir/linear.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/class.dir/linear.cpp.s"
	cd /home/luca/software/projectACA/project/build/class && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/luca/software/projectACA/project/class/linear.cpp -o CMakeFiles/class.dir/linear.cpp.s

class/CMakeFiles/class.dir/robot.cpp.o: class/CMakeFiles/class.dir/flags.make
class/CMakeFiles/class.dir/robot.cpp.o: ../class/robot.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/luca/software/projectACA/project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object class/CMakeFiles/class.dir/robot.cpp.o"
	cd /home/luca/software/projectACA/project/build/class && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/class.dir/robot.cpp.o -c /home/luca/software/projectACA/project/class/robot.cpp

class/CMakeFiles/class.dir/robot.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/class.dir/robot.cpp.i"
	cd /home/luca/software/projectACA/project/build/class && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/luca/software/projectACA/project/class/robot.cpp > CMakeFiles/class.dir/robot.cpp.i

class/CMakeFiles/class.dir/robot.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/class.dir/robot.cpp.s"
	cd /home/luca/software/projectACA/project/build/class && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/luca/software/projectACA/project/class/robot.cpp -o CMakeFiles/class.dir/robot.cpp.s

class/CMakeFiles/class.dir/rotoidal.cpp.o: class/CMakeFiles/class.dir/flags.make
class/CMakeFiles/class.dir/rotoidal.cpp.o: ../class/rotoidal.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/luca/software/projectACA/project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object class/CMakeFiles/class.dir/rotoidal.cpp.o"
	cd /home/luca/software/projectACA/project/build/class && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/class.dir/rotoidal.cpp.o -c /home/luca/software/projectACA/project/class/rotoidal.cpp

class/CMakeFiles/class.dir/rotoidal.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/class.dir/rotoidal.cpp.i"
	cd /home/luca/software/projectACA/project/build/class && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/luca/software/projectACA/project/class/rotoidal.cpp > CMakeFiles/class.dir/rotoidal.cpp.i

class/CMakeFiles/class.dir/rotoidal.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/class.dir/rotoidal.cpp.s"
	cd /home/luca/software/projectACA/project/build/class && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/luca/software/projectACA/project/class/rotoidal.cpp -o CMakeFiles/class.dir/rotoidal.cpp.s

# Object files for target class
class_OBJECTS = \
"CMakeFiles/class.dir/joint.cpp.o" \
"CMakeFiles/class.dir/linear.cpp.o" \
"CMakeFiles/class.dir/robot.cpp.o" \
"CMakeFiles/class.dir/rotoidal.cpp.o"

# External object files for target class
class_EXTERNAL_OBJECTS =

class/libclass.a: class/CMakeFiles/class.dir/joint.cpp.o
class/libclass.a: class/CMakeFiles/class.dir/linear.cpp.o
class/libclass.a: class/CMakeFiles/class.dir/robot.cpp.o
class/libclass.a: class/CMakeFiles/class.dir/rotoidal.cpp.o
class/libclass.a: class/CMakeFiles/class.dir/build.make
class/libclass.a: class/CMakeFiles/class.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/luca/software/projectACA/project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX static library libclass.a"
	cd /home/luca/software/projectACA/project/build/class && $(CMAKE_COMMAND) -P CMakeFiles/class.dir/cmake_clean_target.cmake
	cd /home/luca/software/projectACA/project/build/class && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/class.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
class/CMakeFiles/class.dir/build: class/libclass.a

.PHONY : class/CMakeFiles/class.dir/build

class/CMakeFiles/class.dir/clean:
	cd /home/luca/software/projectACA/project/build/class && $(CMAKE_COMMAND) -P CMakeFiles/class.dir/cmake_clean.cmake
.PHONY : class/CMakeFiles/class.dir/clean

class/CMakeFiles/class.dir/depend:
	cd /home/luca/software/projectACA/project/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/luca/software/projectACA/project /home/luca/software/projectACA/project/class /home/luca/software/projectACA/project/build /home/luca/software/projectACA/project/build/class /home/luca/software/projectACA/project/build/class/CMakeFiles/class.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : class/CMakeFiles/class.dir/depend

