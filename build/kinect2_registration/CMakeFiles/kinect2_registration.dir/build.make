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
CMAKE_SOURCE_DIR = /home/nazir/ws_moveit/src/iai_kinect2/kinect2_registration

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/nazir/ws_moveit/build/kinect2_registration

# Include any dependencies generated for this target.
include CMakeFiles/kinect2_registration.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/kinect2_registration.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/kinect2_registration.dir/flags.make

CMakeFiles/kinect2_registration.dir/src/kinect2_registration.cpp.o: CMakeFiles/kinect2_registration.dir/flags.make
CMakeFiles/kinect2_registration.dir/src/kinect2_registration.cpp.o: /home/nazir/ws_moveit/src/iai_kinect2/kinect2_registration/src/kinect2_registration.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nazir/ws_moveit/build/kinect2_registration/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/kinect2_registration.dir/src/kinect2_registration.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/kinect2_registration.dir/src/kinect2_registration.cpp.o -c /home/nazir/ws_moveit/src/iai_kinect2/kinect2_registration/src/kinect2_registration.cpp

CMakeFiles/kinect2_registration.dir/src/kinect2_registration.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kinect2_registration.dir/src/kinect2_registration.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nazir/ws_moveit/src/iai_kinect2/kinect2_registration/src/kinect2_registration.cpp > CMakeFiles/kinect2_registration.dir/src/kinect2_registration.cpp.i

CMakeFiles/kinect2_registration.dir/src/kinect2_registration.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kinect2_registration.dir/src/kinect2_registration.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nazir/ws_moveit/src/iai_kinect2/kinect2_registration/src/kinect2_registration.cpp -o CMakeFiles/kinect2_registration.dir/src/kinect2_registration.cpp.s

CMakeFiles/kinect2_registration.dir/src/kinect2_registration.cpp.o.requires:

.PHONY : CMakeFiles/kinect2_registration.dir/src/kinect2_registration.cpp.o.requires

CMakeFiles/kinect2_registration.dir/src/kinect2_registration.cpp.o.provides: CMakeFiles/kinect2_registration.dir/src/kinect2_registration.cpp.o.requires
	$(MAKE) -f CMakeFiles/kinect2_registration.dir/build.make CMakeFiles/kinect2_registration.dir/src/kinect2_registration.cpp.o.provides.build
.PHONY : CMakeFiles/kinect2_registration.dir/src/kinect2_registration.cpp.o.provides

CMakeFiles/kinect2_registration.dir/src/kinect2_registration.cpp.o.provides.build: CMakeFiles/kinect2_registration.dir/src/kinect2_registration.cpp.o


CMakeFiles/kinect2_registration.dir/src/depth_registration_cpu.cpp.o: CMakeFiles/kinect2_registration.dir/flags.make
CMakeFiles/kinect2_registration.dir/src/depth_registration_cpu.cpp.o: /home/nazir/ws_moveit/src/iai_kinect2/kinect2_registration/src/depth_registration_cpu.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nazir/ws_moveit/build/kinect2_registration/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/kinect2_registration.dir/src/depth_registration_cpu.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/kinect2_registration.dir/src/depth_registration_cpu.cpp.o -c /home/nazir/ws_moveit/src/iai_kinect2/kinect2_registration/src/depth_registration_cpu.cpp

CMakeFiles/kinect2_registration.dir/src/depth_registration_cpu.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kinect2_registration.dir/src/depth_registration_cpu.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nazir/ws_moveit/src/iai_kinect2/kinect2_registration/src/depth_registration_cpu.cpp > CMakeFiles/kinect2_registration.dir/src/depth_registration_cpu.cpp.i

CMakeFiles/kinect2_registration.dir/src/depth_registration_cpu.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kinect2_registration.dir/src/depth_registration_cpu.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nazir/ws_moveit/src/iai_kinect2/kinect2_registration/src/depth_registration_cpu.cpp -o CMakeFiles/kinect2_registration.dir/src/depth_registration_cpu.cpp.s

CMakeFiles/kinect2_registration.dir/src/depth_registration_cpu.cpp.o.requires:

.PHONY : CMakeFiles/kinect2_registration.dir/src/depth_registration_cpu.cpp.o.requires

CMakeFiles/kinect2_registration.dir/src/depth_registration_cpu.cpp.o.provides: CMakeFiles/kinect2_registration.dir/src/depth_registration_cpu.cpp.o.requires
	$(MAKE) -f CMakeFiles/kinect2_registration.dir/build.make CMakeFiles/kinect2_registration.dir/src/depth_registration_cpu.cpp.o.provides.build
.PHONY : CMakeFiles/kinect2_registration.dir/src/depth_registration_cpu.cpp.o.provides

CMakeFiles/kinect2_registration.dir/src/depth_registration_cpu.cpp.o.provides.build: CMakeFiles/kinect2_registration.dir/src/depth_registration_cpu.cpp.o


CMakeFiles/kinect2_registration.dir/src/depth_registration_opencl.cpp.o: CMakeFiles/kinect2_registration.dir/flags.make
CMakeFiles/kinect2_registration.dir/src/depth_registration_opencl.cpp.o: /home/nazir/ws_moveit/src/iai_kinect2/kinect2_registration/src/depth_registration_opencl.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nazir/ws_moveit/build/kinect2_registration/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/kinect2_registration.dir/src/depth_registration_opencl.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/kinect2_registration.dir/src/depth_registration_opencl.cpp.o -c /home/nazir/ws_moveit/src/iai_kinect2/kinect2_registration/src/depth_registration_opencl.cpp

CMakeFiles/kinect2_registration.dir/src/depth_registration_opencl.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kinect2_registration.dir/src/depth_registration_opencl.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nazir/ws_moveit/src/iai_kinect2/kinect2_registration/src/depth_registration_opencl.cpp > CMakeFiles/kinect2_registration.dir/src/depth_registration_opencl.cpp.i

CMakeFiles/kinect2_registration.dir/src/depth_registration_opencl.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kinect2_registration.dir/src/depth_registration_opencl.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nazir/ws_moveit/src/iai_kinect2/kinect2_registration/src/depth_registration_opencl.cpp -o CMakeFiles/kinect2_registration.dir/src/depth_registration_opencl.cpp.s

CMakeFiles/kinect2_registration.dir/src/depth_registration_opencl.cpp.o.requires:

.PHONY : CMakeFiles/kinect2_registration.dir/src/depth_registration_opencl.cpp.o.requires

CMakeFiles/kinect2_registration.dir/src/depth_registration_opencl.cpp.o.provides: CMakeFiles/kinect2_registration.dir/src/depth_registration_opencl.cpp.o.requires
	$(MAKE) -f CMakeFiles/kinect2_registration.dir/build.make CMakeFiles/kinect2_registration.dir/src/depth_registration_opencl.cpp.o.provides.build
.PHONY : CMakeFiles/kinect2_registration.dir/src/depth_registration_opencl.cpp.o.provides

CMakeFiles/kinect2_registration.dir/src/depth_registration_opencl.cpp.o.provides.build: CMakeFiles/kinect2_registration.dir/src/depth_registration_opencl.cpp.o


# Object files for target kinect2_registration
kinect2_registration_OBJECTS = \
"CMakeFiles/kinect2_registration.dir/src/kinect2_registration.cpp.o" \
"CMakeFiles/kinect2_registration.dir/src/depth_registration_cpu.cpp.o" \
"CMakeFiles/kinect2_registration.dir/src/depth_registration_opencl.cpp.o"

# External object files for target kinect2_registration
kinect2_registration_EXTERNAL_OBJECTS =

/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: CMakeFiles/kinect2_registration.dir/src/kinect2_registration.cpp.o
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: CMakeFiles/kinect2_registration.dir/src/depth_registration_cpu.cpp.o
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: CMakeFiles/kinect2_registration.dir/src/depth_registration_opencl.cpp.o
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: CMakeFiles/kinect2_registration.dir/build.make
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/libroscpp.so
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/librosconsole.so
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/libroscpp_serialization.so
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/librostime.so
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/libxmlrpcpp.so
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/libcpp_common.so
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/libopencv_stitching3.so.3.2.0
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/libopencv_superres3.so.3.2.0
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/libopencv_videostab3.so.3.2.0
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/libopencv_aruco3.so.3.2.0
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/libopencv_bgsegm3.so.3.2.0
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/libopencv_bioinspired3.so.3.2.0
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/libopencv_ccalib3.so.3.2.0
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/libopencv_cvv3.so.3.2.0
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/libopencv_datasets3.so.3.2.0
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/libopencv_dpm3.so.3.2.0
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/libopencv_face3.so.3.2.0
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/libopencv_fuzzy3.so.3.2.0
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/libopencv_hdf3.so.3.2.0
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/libopencv_line_descriptor3.so.3.2.0
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/libopencv_optflow3.so.3.2.0
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/libopencv_plot3.so.3.2.0
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/libopencv_reg3.so.3.2.0
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/libopencv_saliency3.so.3.2.0
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/libopencv_stereo3.so.3.2.0
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/libopencv_structured_light3.so.3.2.0
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/libopencv_surface_matching3.so.3.2.0
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/libopencv_text3.so.3.2.0
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/libopencv_xfeatures2d3.so.3.2.0
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/libopencv_ximgproc3.so.3.2.0
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/libopencv_xobjdetect3.so.3.2.0
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/libopencv_xphoto3.so.3.2.0
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libOpenCL.so
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/libopencv_shape3.so.3.2.0
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/libopencv_video3.so.3.2.0
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/libopencv_viz3.so.3.2.0
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/libopencv_phase_unwrapping3.so.3.2.0
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/libopencv_rgbd3.so.3.2.0
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/libopencv_calib3d3.so.3.2.0
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/libopencv_features2d3.so.3.2.0
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/libopencv_flann3.so.3.2.0
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/libopencv_objdetect3.so.3.2.0
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/libopencv_ml3.so.3.2.0
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/libopencv_highgui3.so.3.2.0
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/libopencv_photo3.so.3.2.0
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/libopencv_videoio3.so.3.2.0
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/libopencv_imgcodecs3.so.3.2.0
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/libopencv_imgproc3.so.3.2.0
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: /opt/ros/kinetic/lib/libopencv_core3.so.3.2.0
/home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so: CMakeFiles/kinect2_registration.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nazir/ws_moveit/build/kinect2_registration/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX shared library /home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/kinect2_registration.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/kinect2_registration.dir/build: /home/nazir/ws_moveit/devel/.private/kinect2_registration/lib/libkinect2_registration.so

.PHONY : CMakeFiles/kinect2_registration.dir/build

CMakeFiles/kinect2_registration.dir/requires: CMakeFiles/kinect2_registration.dir/src/kinect2_registration.cpp.o.requires
CMakeFiles/kinect2_registration.dir/requires: CMakeFiles/kinect2_registration.dir/src/depth_registration_cpu.cpp.o.requires
CMakeFiles/kinect2_registration.dir/requires: CMakeFiles/kinect2_registration.dir/src/depth_registration_opencl.cpp.o.requires

.PHONY : CMakeFiles/kinect2_registration.dir/requires

CMakeFiles/kinect2_registration.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/kinect2_registration.dir/cmake_clean.cmake
.PHONY : CMakeFiles/kinect2_registration.dir/clean

CMakeFiles/kinect2_registration.dir/depend:
	cd /home/nazir/ws_moveit/build/kinect2_registration && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nazir/ws_moveit/src/iai_kinect2/kinect2_registration /home/nazir/ws_moveit/src/iai_kinect2/kinect2_registration /home/nazir/ws_moveit/build/kinect2_registration /home/nazir/ws_moveit/build/kinect2_registration /home/nazir/ws_moveit/build/kinect2_registration/CMakeFiles/kinect2_registration.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/kinect2_registration.dir/depend

