# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/kir/modbus_client

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kir/modbus_client/build

# Include any dependencies generated for this target.
include CMakeFiles/modbus_client.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/modbus_client.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/modbus_client.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/modbus_client.dir/flags.make

CMakeFiles/modbus_client.dir/src/main.cpp.o: CMakeFiles/modbus_client.dir/flags.make
CMakeFiles/modbus_client.dir/src/main.cpp.o: ../src/main.cpp
CMakeFiles/modbus_client.dir/src/main.cpp.o: CMakeFiles/modbus_client.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kir/modbus_client/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/modbus_client.dir/src/main.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/modbus_client.dir/src/main.cpp.o -MF CMakeFiles/modbus_client.dir/src/main.cpp.o.d -o CMakeFiles/modbus_client.dir/src/main.cpp.o -c /home/kir/modbus_client/src/main.cpp

CMakeFiles/modbus_client.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modbus_client.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kir/modbus_client/src/main.cpp > CMakeFiles/modbus_client.dir/src/main.cpp.i

CMakeFiles/modbus_client.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modbus_client.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kir/modbus_client/src/main.cpp -o CMakeFiles/modbus_client.dir/src/main.cpp.s

# Object files for target modbus_client
modbus_client_OBJECTS = \
"CMakeFiles/modbus_client.dir/src/main.cpp.o"

# External object files for target modbus_client
modbus_client_EXTERNAL_OBJECTS =

modbus_client: CMakeFiles/modbus_client.dir/src/main.cpp.o
modbus_client: CMakeFiles/modbus_client.dir/build.make
modbus_client: CMakeFiles/modbus_client.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kir/modbus_client/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable modbus_client"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/modbus_client.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/modbus_client.dir/build: modbus_client
.PHONY : CMakeFiles/modbus_client.dir/build

CMakeFiles/modbus_client.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/modbus_client.dir/cmake_clean.cmake
.PHONY : CMakeFiles/modbus_client.dir/clean

CMakeFiles/modbus_client.dir/depend:
	cd /home/kir/modbus_client/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kir/modbus_client /home/kir/modbus_client /home/kir/modbus_client/build /home/kir/modbus_client/build /home/kir/modbus_client/build/CMakeFiles/modbus_client.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/modbus_client.dir/depend

