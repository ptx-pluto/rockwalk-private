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
CMAKE_SOURCE_DIR = /home/nazir/ws_moveit/src/freenect_stack/freenect_camera

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/nazir/ws_moveit/build/freenect_camera

# Include any dependencies generated for this target.
include CMakeFiles/freenect_nodelet.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/freenect_nodelet.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/freenect_nodelet.dir/flags.make

CMakeFiles/freenect_nodelet.dir/src/nodelets/driver.cpp.o: CMakeFiles/freenect_nodelet.dir/flags.make
CMakeFiles/freenect_nodelet.dir/src/nodelets/driver.cpp.o: /home/nazir/ws_moveit/src/freenect_stack/freenect_camera/src/nodelets/driver.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nazir/ws_moveit/build/freenect_camera/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/freenect_nodelet.dir/src/nodelets/driver.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/freenect_nodelet.dir/src/nodelets/driver.cpp.o -c /home/nazir/ws_moveit/src/freenect_stack/freenect_camera/src/nodelets/driver.cpp

CMakeFiles/freenect_nodelet.dir/src/nodelets/driver.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/freenect_nodelet.dir/src/nodelets/driver.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nazir/ws_moveit/src/freenect_stack/freenect_camera/src/nodelets/driver.cpp > CMakeFiles/freenect_nodelet.dir/src/nodelets/driver.cpp.i

CMakeFiles/freenect_nodelet.dir/src/nodelets/driver.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/freenect_nodelet.dir/src/nodelets/driver.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nazir/ws_moveit/src/freenect_stack/freenect_camera/src/nodelets/driver.cpp -o CMakeFiles/freenect_nodelet.dir/src/nodelets/driver.cpp.s

CMakeFiles/freenect_nodelet.dir/src/nodelets/driver.cpp.o.requires:

.PHONY : CMakeFiles/freenect_nodelet.dir/src/nodelets/driver.cpp.o.requires

CMakeFiles/freenect_nodelet.dir/src/nodelets/driver.cpp.o.provides: CMakeFiles/freenect_nodelet.dir/src/nodelets/driver.cpp.o.requires
	$(MAKE) -f CMakeFiles/freenect_nodelet.dir/build.make CMakeFiles/freenect_nodelet.dir/src/nodelets/driver.cpp.o.provides.build
.PHONY : CMakeFiles/freenect_nodelet.dir/src/nodelets/driver.cpp.o.provides

CMakeFiles/freenect_nodelet.dir/src/nodelets/driver.cpp.o.provides.build: CMakeFiles/freenect_nodelet.dir/src/nodelets/driver.cpp.o


# Object files for target freenect_nodelet
freenect_nodelet_OBJECTS = \
"CMakeFiles/freenect_nodelet.dir/src/nodelets/driver.cpp.o"

# External object files for target freenect_nodelet
freenect_nodelet_EXTERNAL_OBJECTS =

/home/nazir/ws_moveit/devel/.private/freenect_camera/lib/libfreenect_nodelet.so: CMakeFiles/freenect_nodelet.dir/src/nodelets/driver.cpp.o
/home/nazir/ws_moveit/devel/.private/freenect_camera/lib/libfreenect_nodelet.so: CMakeFiles/freenect_nodelet.dir/build.make
/home/nazir/ws_moveit/devel/.private/freenect_camera/lib/libfreenect_nodelet.so: /opt/ros/kinetic/lib/libcamera_info_manager.so
/home/nazir/ws_moveit/devel/.private/freenect_camera/lib/libfreenect_nodelet.so: /opt/ros/kinetic/lib/libdynamic_reconfigure_config_init_mutex.so
/home/nazir/ws_moveit/devel/.private/freenect_camera/lib/libfreenect_nodelet.so: /opt/ros/kinetic/lib/libimage_transport.so
/home/nazir/ws_moveit/devel/.private/freenect_camera/lib/libfreenect_nodelet.so: /opt/ros/kinetic/lib/libmessage_filters.so
/home/nazir/ws_moveit/devel/.private/freenect_camera/lib/libfreenect_nodelet.so: /opt/ros/kinetic/lib/libnodeletlib.so
/home/nazir/ws_moveit/devel/.private/freenect_camera/lib/libfreenect_nodelet.so: /opt/ros/kinetic/lib/libbondcpp.so
/home/nazir/ws_moveit/devel/.private/freenect_camera/lib/libfreenect_nodelet.so: /usr/lib/x86_64-linux-gnu/libuuid.so
/home/nazir/ws_moveit/devel/.private/freenect_camera/lib/libfreenect_nodelet.so: /opt/ros/kinetic/lib/libroscpp.so
/home/nazir/ws_moveit/devel/.private/freenect_camera/lib/libfreenect_nodelet.so: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/nazir/ws_moveit/devel/.private/freenect_camera/lib/libfreenect_nodelet.so: /opt/ros/kinetic/lib/libxmlrpcpp.so
/home/nazir/ws_moveit/devel/.private/freenect_camera/lib/libfreenect_nodelet.so: /opt/ros/kinetic/lib/libroscpp_serialization.so
/home/nazir/ws_moveit/devel/.private/freenect_camera/lib/libfreenect_nodelet.so: /opt/ros/kinetic/lib/libclass_loader.so
/home/nazir/ws_moveit/devel/.private/freenect_camera/lib/libfreenect_nodelet.so: /usr/lib/libPocoFoundation.so
/home/nazir/ws_moveit/devel/.private/freenect_camera/lib/libfreenect_nodelet.so: /usr/lib/x86_64-linux-gnu/libdl.so
/home/nazir/ws_moveit/devel/.private/freenect_camera/lib/libfreenect_nodelet.so: /opt/ros/kinetic/lib/librosconsole.so
/home/nazir/ws_moveit/devel/.private/freenect_camera/lib/libfreenect_nodelet.so: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
/home/nazir/ws_moveit/devel/.private/freenect_camera/lib/libfreenect_nodelet.so: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
/home/nazir/ws_moveit/devel/.private/freenect_camera/lib/libfreenect_nodelet.so: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/nazir/ws_moveit/devel/.private/freenect_camera/lib/libfreenect_nodelet.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/nazir/ws_moveit/devel/.private/freenect_camera/lib/libfreenect_nodelet.so: /opt/ros/kinetic/lib/librostime.so
/home/nazir/ws_moveit/devel/.private/freenect_camera/lib/libfreenect_nodelet.so: /opt/ros/kinetic/lib/libcpp_common.so
/home/nazir/ws_moveit/devel/.private/freenect_camera/lib/libfreenect_nodelet.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/nazir/ws_moveit/devel/.private/freenect_camera/lib/libfreenect_nodelet.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/nazir/ws_moveit/devel/.private/freenect_camera/lib/libfreenect_nodelet.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/nazir/ws_moveit/devel/.private/freenect_camera/lib/libfreenect_nodelet.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/nazir/ws_moveit/devel/.private/freenect_camera/lib/libfreenect_nodelet.so: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/nazir/ws_moveit/devel/.private/freenect_camera/lib/libfreenect_nodelet.so: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/nazir/ws_moveit/devel/.private/freenect_camera/lib/libfreenect_nodelet.so: /opt/ros/kinetic/lib/libroslib.so
/home/nazir/ws_moveit/devel/.private/freenect_camera/lib/libfreenect_nodelet.so: /opt/ros/kinetic/lib/librospack.so
/home/nazir/ws_moveit/devel/.private/freenect_camera/lib/libfreenect_nodelet.so: /usr/lib/x86_64-linux-gnu/libpython2.7.so
/home/nazir/ws_moveit/devel/.private/freenect_camera/lib/libfreenect_nodelet.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/nazir/ws_moveit/devel/.private/freenect_camera/lib/libfreenect_nodelet.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/nazir/ws_moveit/devel/.private/freenect_camera/lib/libfreenect_nodelet.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/nazir/ws_moveit/devel/.private/freenect_camera/lib/libfreenect_nodelet.so: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/nazir/ws_moveit/devel/.private/freenect_camera/lib/libfreenect_nodelet.so: /opt/ros/kinetic/lib/libfreenect.so
/home/nazir/ws_moveit/devel/.private/freenect_camera/lib/libfreenect_nodelet.so: CMakeFiles/freenect_nodelet.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nazir/ws_moveit/build/freenect_camera/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/nazir/ws_moveit/devel/.private/freenect_camera/lib/libfreenect_nodelet.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/freenect_nodelet.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/freenect_nodelet.dir/build: /home/nazir/ws_moveit/devel/.private/freenect_camera/lib/libfreenect_nodelet.so

.PHONY : CMakeFiles/freenect_nodelet.dir/build

CMakeFiles/freenect_nodelet.dir/requires: CMakeFiles/freenect_nodelet.dir/src/nodelets/driver.cpp.o.requires

.PHONY : CMakeFiles/freenect_nodelet.dir/requires

CMakeFiles/freenect_nodelet.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/freenect_nodelet.dir/cmake_clean.cmake
.PHONY : CMakeFiles/freenect_nodelet.dir/clean

CMakeFiles/freenect_nodelet.dir/depend:
	cd /home/nazir/ws_moveit/build/freenect_camera && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nazir/ws_moveit/src/freenect_stack/freenect_camera /home/nazir/ws_moveit/src/freenect_stack/freenect_camera /home/nazir/ws_moveit/build/freenect_camera /home/nazir/ws_moveit/build/freenect_camera /home/nazir/ws_moveit/build/freenect_camera/CMakeFiles/freenect_nodelet.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/freenect_nodelet.dir/depend

