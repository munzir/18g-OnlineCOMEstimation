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
CMAKE_SOURCE_DIR = /home/apatel435/Desktop/WholeBodyControlAttempt1/18gBalancingWithCoMError/lqrWrapper

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/apatel435/Desktop/WholeBodyControlAttempt1/18gBalancingWithCoMError/lqrWrapper/build

# Include any dependencies generated for this target.
include CMakeFiles/lqrTest.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/lqrTest.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/lqrTest.dir/flags.make

CMakeFiles/lqrTest.dir/lqr.cpp.o: CMakeFiles/lqrTest.dir/flags.make
CMakeFiles/lqrTest.dir/lqr.cpp.o: ../lqr.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/apatel435/Desktop/WholeBodyControlAttempt1/18gBalancingWithCoMError/lqrWrapper/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/lqrTest.dir/lqr.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/lqrTest.dir/lqr.cpp.o -c /home/apatel435/Desktop/WholeBodyControlAttempt1/18gBalancingWithCoMError/lqrWrapper/lqr.cpp

CMakeFiles/lqrTest.dir/lqr.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lqrTest.dir/lqr.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/apatel435/Desktop/WholeBodyControlAttempt1/18gBalancingWithCoMError/lqrWrapper/lqr.cpp > CMakeFiles/lqrTest.dir/lqr.cpp.i

CMakeFiles/lqrTest.dir/lqr.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lqrTest.dir/lqr.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/apatel435/Desktop/WholeBodyControlAttempt1/18gBalancingWithCoMError/lqrWrapper/lqr.cpp -o CMakeFiles/lqrTest.dir/lqr.cpp.s

CMakeFiles/lqrTest.dir/lqr.cpp.o.requires:

.PHONY : CMakeFiles/lqrTest.dir/lqr.cpp.o.requires

CMakeFiles/lqrTest.dir/lqr.cpp.o.provides: CMakeFiles/lqrTest.dir/lqr.cpp.o.requires
	$(MAKE) -f CMakeFiles/lqrTest.dir/build.make CMakeFiles/lqrTest.dir/lqr.cpp.o.provides.build
.PHONY : CMakeFiles/lqrTest.dir/lqr.cpp.o.provides

CMakeFiles/lqrTest.dir/lqr.cpp.o.provides.build: CMakeFiles/lqrTest.dir/lqr.cpp.o


# Object files for target lqrTest
lqrTest_OBJECTS = \
"CMakeFiles/lqrTest.dir/lqr.cpp.o"

# External object files for target lqrTest
lqrTest_EXTERNAL_OBJECTS =

lqrTest: CMakeFiles/lqrTest.dir/lqr.cpp.o
lqrTest: CMakeFiles/lqrTest.dir/build.make
lqrTest: /usr/lib/libdart-utils-urdf.so.6.5.0
lqrTest: /usr/lib/libdart-gui.so.6.5.0
lqrTest: /usr/lib/x86_64-linux-gnu/liburdfdom_sensor.so
lqrTest: /usr/lib/x86_64-linux-gnu/liburdfdom_model_state.so
lqrTest: /usr/lib/x86_64-linux-gnu/liburdfdom_model.so
lqrTest: /usr/lib/x86_64-linux-gnu/liburdfdom_world.so
lqrTest: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
lqrTest: /usr/lib/libdart-utils.so.6.5.0
lqrTest: /usr/lib/libdart.so.6.5.0
lqrTest: /usr/lib/x86_64-linux-gnu/libccd.so
lqrTest: /usr/lib/libfcl.so
lqrTest: /usr/lib/x86_64-linux-gnu/libassimp.so
lqrTest: /usr/lib/x86_64-linux-gnu/libboost_regex.so
lqrTest: /usr/lib/x86_64-linux-gnu/libboost_system.so
lqrTest: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
lqrTest: /usr/lib/libdart-external-odelcpsolver.so.6.5.0
lqrTest: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
lqrTest: /usr/lib/x86_64-linux-gnu/libglut.so
lqrTest: /usr/lib/x86_64-linux-gnu/libXmu.so
lqrTest: /usr/lib/x86_64-linux-gnu/libXi.so
lqrTest: /usr/lib/x86_64-linux-gnu/libGLU.so
lqrTest: /usr/lib/x86_64-linux-gnu/libGL.so
lqrTest: /usr/lib/libdart-external-lodepng.so.6.5.0
lqrTest: /usr/lib/libdart-external-imgui.so.6.5.0
lqrTest: CMakeFiles/lqrTest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/apatel435/Desktop/WholeBodyControlAttempt1/18gBalancingWithCoMError/lqrWrapper/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable lqrTest"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lqrTest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/lqrTest.dir/build: lqrTest

.PHONY : CMakeFiles/lqrTest.dir/build

CMakeFiles/lqrTest.dir/requires: CMakeFiles/lqrTest.dir/lqr.cpp.o.requires

.PHONY : CMakeFiles/lqrTest.dir/requires

CMakeFiles/lqrTest.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/lqrTest.dir/cmake_clean.cmake
.PHONY : CMakeFiles/lqrTest.dir/clean

CMakeFiles/lqrTest.dir/depend:
	cd /home/apatel435/Desktop/WholeBodyControlAttempt1/18gBalancingWithCoMError/lqrWrapper/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/apatel435/Desktop/WholeBodyControlAttempt1/18gBalancingWithCoMError/lqrWrapper /home/apatel435/Desktop/WholeBodyControlAttempt1/18gBalancingWithCoMError/lqrWrapper /home/apatel435/Desktop/WholeBodyControlAttempt1/18gBalancingWithCoMError/lqrWrapper/build /home/apatel435/Desktop/WholeBodyControlAttempt1/18gBalancingWithCoMError/lqrWrapper/build /home/apatel435/Desktop/WholeBodyControlAttempt1/18gBalancingWithCoMError/lqrWrapper/build/CMakeFiles/lqrTest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/lqrTest.dir/depend

