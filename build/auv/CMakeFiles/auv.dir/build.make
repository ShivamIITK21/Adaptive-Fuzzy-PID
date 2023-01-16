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
CMAKE_SOURCE_DIR = /home/shivam/adaptive-fuzzy-pid-auv

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/shivam/adaptive-fuzzy-pid-auv/build

# Include any dependencies generated for this target.
include auv/CMakeFiles/auv.dir/depend.make

# Include the progress variables for this target.
include auv/CMakeFiles/auv.dir/progress.make

# Include the compile flags for this target's objects.
include auv/CMakeFiles/auv.dir/flags.make

auv/CMakeFiles/auv.dir/src/auv.cpp.o: auv/CMakeFiles/auv.dir/flags.make
auv/CMakeFiles/auv.dir/src/auv.cpp.o: ../auv/src/auv.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shivam/adaptive-fuzzy-pid-auv/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object auv/CMakeFiles/auv.dir/src/auv.cpp.o"
	cd /home/shivam/adaptive-fuzzy-pid-auv/build/auv && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/auv.dir/src/auv.cpp.o -c /home/shivam/adaptive-fuzzy-pid-auv/auv/src/auv.cpp

auv/CMakeFiles/auv.dir/src/auv.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/auv.dir/src/auv.cpp.i"
	cd /home/shivam/adaptive-fuzzy-pid-auv/build/auv && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/shivam/adaptive-fuzzy-pid-auv/auv/src/auv.cpp > CMakeFiles/auv.dir/src/auv.cpp.i

auv/CMakeFiles/auv.dir/src/auv.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/auv.dir/src/auv.cpp.s"
	cd /home/shivam/adaptive-fuzzy-pid-auv/build/auv && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/shivam/adaptive-fuzzy-pid-auv/auv/src/auv.cpp -o CMakeFiles/auv.dir/src/auv.cpp.s

# Object files for target auv
auv_OBJECTS = \
"CMakeFiles/auv.dir/src/auv.cpp.o"

# External object files for target auv
auv_EXTERNAL_OBJECTS =

auv/libauv.a: auv/CMakeFiles/auv.dir/src/auv.cpp.o
auv/libauv.a: auv/CMakeFiles/auv.dir/build.make
auv/libauv.a: auv/CMakeFiles/auv.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/shivam/adaptive-fuzzy-pid-auv/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libauv.a"
	cd /home/shivam/adaptive-fuzzy-pid-auv/build/auv && $(CMAKE_COMMAND) -P CMakeFiles/auv.dir/cmake_clean_target.cmake
	cd /home/shivam/adaptive-fuzzy-pid-auv/build/auv && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/auv.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
auv/CMakeFiles/auv.dir/build: auv/libauv.a

.PHONY : auv/CMakeFiles/auv.dir/build

auv/CMakeFiles/auv.dir/clean:
	cd /home/shivam/adaptive-fuzzy-pid-auv/build/auv && $(CMAKE_COMMAND) -P CMakeFiles/auv.dir/cmake_clean.cmake
.PHONY : auv/CMakeFiles/auv.dir/clean

auv/CMakeFiles/auv.dir/depend:
	cd /home/shivam/adaptive-fuzzy-pid-auv/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/shivam/adaptive-fuzzy-pid-auv /home/shivam/adaptive-fuzzy-pid-auv/auv /home/shivam/adaptive-fuzzy-pid-auv/build /home/shivam/adaptive-fuzzy-pid-auv/build/auv /home/shivam/adaptive-fuzzy-pid-auv/build/auv/CMakeFiles/auv.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : auv/CMakeFiles/auv.dir/depend

