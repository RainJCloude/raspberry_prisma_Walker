# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.26

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/lib/python3.8/dist-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /usr/local/lib/python3.8/dist-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/claudio/raspberry_prisma_walker

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/claudio/raspberry_prisma_walker/build

# Include any dependencies generated for this target.
include CMakeFiles/prisma_walker.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/prisma_walker.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/prisma_walker.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/prisma_walker.dir/flags.make

CMakeFiles/prisma_walker.dir/Environment.cpp.o: CMakeFiles/prisma_walker.dir/flags.make
CMakeFiles/prisma_walker.dir/Environment.cpp.o: /home/claudio/raspberry_prisma_walker/Environment.cpp
CMakeFiles/prisma_walker.dir/Environment.cpp.o: CMakeFiles/prisma_walker.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/claudio/raspberry_prisma_walker/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/prisma_walker.dir/Environment.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/prisma_walker.dir/Environment.cpp.o -MF CMakeFiles/prisma_walker.dir/Environment.cpp.o.d -o CMakeFiles/prisma_walker.dir/Environment.cpp.o -c /home/claudio/raspberry_prisma_walker/Environment.cpp

CMakeFiles/prisma_walker.dir/Environment.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/prisma_walker.dir/Environment.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/claudio/raspberry_prisma_walker/Environment.cpp > CMakeFiles/prisma_walker.dir/Environment.cpp.i

CMakeFiles/prisma_walker.dir/Environment.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/prisma_walker.dir/Environment.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/claudio/raspberry_prisma_walker/Environment.cpp -o CMakeFiles/prisma_walker.dir/Environment.cpp.s

# Object files for target prisma_walker
prisma_walker_OBJECTS = \
"CMakeFiles/prisma_walker.dir/Environment.cpp.o"

# External object files for target prisma_walker
prisma_walker_EXTERNAL_OBJECTS =

/home/claudio/raspberry_prisma_walker/bin/prisma_walker.cpython-38-x86_64-linux-gnu.so: CMakeFiles/prisma_walker.dir/Environment.cpp.o
/home/claudio/raspberry_prisma_walker/bin/prisma_walker.cpython-38-x86_64-linux-gnu.so: CMakeFiles/prisma_walker.dir/build.make
/home/claudio/raspberry_prisma_walker/bin/prisma_walker.cpython-38-x86_64-linux-gnu.so: /home/claudio/raspberry_prisma_walker/bin/libdxl_x64_cpp.so
/home/claudio/raspberry_prisma_walker/bin/prisma_walker.cpython-38-x86_64-linux-gnu.so: /home/claudio/raspberry_prisma_walker/bin/libhebic++.so.3.7.0
/home/claudio/raspberry_prisma_walker/bin/prisma_walker.cpython-38-x86_64-linux-gnu.so: /home/claudio/raspberry_prisma_walker/hebi-cpp/hebi/lib/linux_x86_64/libhebi.so
/home/claudio/raspberry_prisma_walker/bin/prisma_walker.cpython-38-x86_64-linux-gnu.so: CMakeFiles/prisma_walker.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/claudio/raspberry_prisma_walker/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared module /home/claudio/raspberry_prisma_walker/bin/prisma_walker.cpython-38-x86_64-linux-gnu.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/prisma_walker.dir/link.txt --verbose=$(VERBOSE)
	/usr/bin/strip /home/claudio/raspberry_prisma_walker/bin/prisma_walker.cpython-38-x86_64-linux-gnu.so

# Rule to build all files generated by this target.
CMakeFiles/prisma_walker.dir/build: /home/claudio/raspberry_prisma_walker/bin/prisma_walker.cpython-38-x86_64-linux-gnu.so
.PHONY : CMakeFiles/prisma_walker.dir/build

CMakeFiles/prisma_walker.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/prisma_walker.dir/cmake_clean.cmake
.PHONY : CMakeFiles/prisma_walker.dir/clean

CMakeFiles/prisma_walker.dir/depend:
	cd /home/claudio/raspberry_prisma_walker/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/claudio/raspberry_prisma_walker /home/claudio/raspberry_prisma_walker /home/claudio/raspberry_prisma_walker/build /home/claudio/raspberry_prisma_walker/build /home/claudio/raspberry_prisma_walker/build/CMakeFiles/prisma_walker.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/prisma_walker.dir/depend

