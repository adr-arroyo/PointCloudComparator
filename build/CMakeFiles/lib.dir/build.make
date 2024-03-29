# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.7

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
CMAKE_SOURCE_DIR = /home/adrian/PointCloudComparator

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/adrian/PointCloudComparator/build

# Include any dependencies generated for this target.
include CMakeFiles/lib.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/lib.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/lib.dir/flags.make

CMakeFiles/lib.dir/src/segmentation.cpp.o: CMakeFiles/lib.dir/flags.make
CMakeFiles/lib.dir/src/segmentation.cpp.o: ../src/segmentation.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/adrian/PointCloudComparator/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/lib.dir/src/segmentation.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/lib.dir/src/segmentation.cpp.o -c /home/adrian/PointCloudComparator/src/segmentation.cpp

CMakeFiles/lib.dir/src/segmentation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lib.dir/src/segmentation.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/adrian/PointCloudComparator/src/segmentation.cpp > CMakeFiles/lib.dir/src/segmentation.cpp.i

CMakeFiles/lib.dir/src/segmentation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lib.dir/src/segmentation.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/adrian/PointCloudComparator/src/segmentation.cpp -o CMakeFiles/lib.dir/src/segmentation.cpp.s

CMakeFiles/lib.dir/src/segmentation.cpp.o.requires:

.PHONY : CMakeFiles/lib.dir/src/segmentation.cpp.o.requires

CMakeFiles/lib.dir/src/segmentation.cpp.o.provides: CMakeFiles/lib.dir/src/segmentation.cpp.o.requires
	$(MAKE) -f CMakeFiles/lib.dir/build.make CMakeFiles/lib.dir/src/segmentation.cpp.o.provides.build
.PHONY : CMakeFiles/lib.dir/src/segmentation.cpp.o.provides

CMakeFiles/lib.dir/src/segmentation.cpp.o.provides.build: CMakeFiles/lib.dir/src/segmentation.cpp.o


CMakeFiles/lib.dir/src/comparator.cpp.o: CMakeFiles/lib.dir/flags.make
CMakeFiles/lib.dir/src/comparator.cpp.o: ../src/comparator.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/adrian/PointCloudComparator/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/lib.dir/src/comparator.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/lib.dir/src/comparator.cpp.o -c /home/adrian/PointCloudComparator/src/comparator.cpp

CMakeFiles/lib.dir/src/comparator.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lib.dir/src/comparator.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/adrian/PointCloudComparator/src/comparator.cpp > CMakeFiles/lib.dir/src/comparator.cpp.i

CMakeFiles/lib.dir/src/comparator.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lib.dir/src/comparator.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/adrian/PointCloudComparator/src/comparator.cpp -o CMakeFiles/lib.dir/src/comparator.cpp.s

CMakeFiles/lib.dir/src/comparator.cpp.o.requires:

.PHONY : CMakeFiles/lib.dir/src/comparator.cpp.o.requires

CMakeFiles/lib.dir/src/comparator.cpp.o.provides: CMakeFiles/lib.dir/src/comparator.cpp.o.requires
	$(MAKE) -f CMakeFiles/lib.dir/build.make CMakeFiles/lib.dir/src/comparator.cpp.o.provides.build
.PHONY : CMakeFiles/lib.dir/src/comparator.cpp.o.provides

CMakeFiles/lib.dir/src/comparator.cpp.o.provides.build: CMakeFiles/lib.dir/src/comparator.cpp.o


# Object files for target lib
lib_OBJECTS = \
"CMakeFiles/lib.dir/src/segmentation.cpp.o" \
"CMakeFiles/lib.dir/src/comparator.cpp.o"

# External object files for target lib
lib_EXTERNAL_OBJECTS =

liblib.a: CMakeFiles/lib.dir/src/segmentation.cpp.o
liblib.a: CMakeFiles/lib.dir/src/comparator.cpp.o
liblib.a: CMakeFiles/lib.dir/build.make
liblib.a: CMakeFiles/lib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/adrian/PointCloudComparator/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library liblib.a"
	$(CMAKE_COMMAND) -P CMakeFiles/lib.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/lib.dir/build: liblib.a

.PHONY : CMakeFiles/lib.dir/build

CMakeFiles/lib.dir/requires: CMakeFiles/lib.dir/src/segmentation.cpp.o.requires
CMakeFiles/lib.dir/requires: CMakeFiles/lib.dir/src/comparator.cpp.o.requires

.PHONY : CMakeFiles/lib.dir/requires

CMakeFiles/lib.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/lib.dir/cmake_clean.cmake
.PHONY : CMakeFiles/lib.dir/clean

CMakeFiles/lib.dir/depend:
	cd /home/adrian/PointCloudComparator/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/adrian/PointCloudComparator /home/adrian/PointCloudComparator /home/adrian/PointCloudComparator/build /home/adrian/PointCloudComparator/build /home/adrian/PointCloudComparator/build/CMakeFiles/lib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/lib.dir/depend

