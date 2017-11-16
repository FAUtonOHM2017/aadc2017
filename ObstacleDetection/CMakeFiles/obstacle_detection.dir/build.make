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
CMAKE_SOURCE_DIR = /home/aadc/ADTF/aadc2017/src/aadcUser

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/aadc/ADTF/aadc2017/src/aadcUser

# Include any dependencies generated for this target.
include ObstacleDetection/CMakeFiles/obstacle_detection.dir/depend.make

# Include the progress variables for this target.
include ObstacleDetection/CMakeFiles/obstacle_detection.dir/progress.make

# Include the compile flags for this target's objects.
include ObstacleDetection/CMakeFiles/obstacle_detection.dir/flags.make

ObstacleDetection/CMakeFiles/obstacle_detection.dir/ObstacleDetection.cpp.o: ObstacleDetection/CMakeFiles/obstacle_detection.dir/flags.make
ObstacleDetection/CMakeFiles/obstacle_detection.dir/ObstacleDetection.cpp.o: ObstacleDetection/ObstacleDetection.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aadc/ADTF/aadc2017/src/aadcUser/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object ObstacleDetection/CMakeFiles/obstacle_detection.dir/ObstacleDetection.cpp.o"
	cd /home/aadc/ADTF/aadc2017/src/aadcUser/ObstacleDetection && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/obstacle_detection.dir/ObstacleDetection.cpp.o -c /home/aadc/ADTF/aadc2017/src/aadcUser/ObstacleDetection/ObstacleDetection.cpp

ObstacleDetection/CMakeFiles/obstacle_detection.dir/ObstacleDetection.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/obstacle_detection.dir/ObstacleDetection.cpp.i"
	cd /home/aadc/ADTF/aadc2017/src/aadcUser/ObstacleDetection && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aadc/ADTF/aadc2017/src/aadcUser/ObstacleDetection/ObstacleDetection.cpp > CMakeFiles/obstacle_detection.dir/ObstacleDetection.cpp.i

ObstacleDetection/CMakeFiles/obstacle_detection.dir/ObstacleDetection.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/obstacle_detection.dir/ObstacleDetection.cpp.s"
	cd /home/aadc/ADTF/aadc2017/src/aadcUser/ObstacleDetection && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aadc/ADTF/aadc2017/src/aadcUser/ObstacleDetection/ObstacleDetection.cpp -o CMakeFiles/obstacle_detection.dir/ObstacleDetection.cpp.s

ObstacleDetection/CMakeFiles/obstacle_detection.dir/ObstacleDetection.cpp.o.requires:

.PHONY : ObstacleDetection/CMakeFiles/obstacle_detection.dir/ObstacleDetection.cpp.o.requires

ObstacleDetection/CMakeFiles/obstacle_detection.dir/ObstacleDetection.cpp.o.provides: ObstacleDetection/CMakeFiles/obstacle_detection.dir/ObstacleDetection.cpp.o.requires
	$(MAKE) -f ObstacleDetection/CMakeFiles/obstacle_detection.dir/build.make ObstacleDetection/CMakeFiles/obstacle_detection.dir/ObstacleDetection.cpp.o.provides.build
.PHONY : ObstacleDetection/CMakeFiles/obstacle_detection.dir/ObstacleDetection.cpp.o.provides

ObstacleDetection/CMakeFiles/obstacle_detection.dir/ObstacleDetection.cpp.o.provides.build: ObstacleDetection/CMakeFiles/obstacle_detection.dir/ObstacleDetection.cpp.o


# Object files for target obstacle_detection
obstacle_detection_OBJECTS = \
"CMakeFiles/obstacle_detection.dir/ObstacleDetection.cpp.o"

# External object files for target obstacle_detection
obstacle_detection_EXTERNAL_OBJECTS =

ObstacleDetection/obstacle_detection.plb: ObstacleDetection/CMakeFiles/obstacle_detection.dir/ObstacleDetection.cpp.o
ObstacleDetection/obstacle_detection.plb: ObstacleDetection/CMakeFiles/obstacle_detection.dir/build.make
ObstacleDetection/obstacle_detection.plb: /opt/adtf/2.14.0/lib/libadtfgfx_2140.a
ObstacleDetection/obstacle_detection.plb: /opt/adtf/2.14.0/lib/libadtfsdk_2140.a
ObstacleDetection/obstacle_detection.plb: /opt/adtf/2.14.0/lib/libadtfucom_190.a
ObstacleDetection/obstacle_detection.plb: /opt/adtf/2.14.0/lib/libadtfutil_1170.a
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_cudabgsegm.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_cudaobjdetect.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_cudastereo.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_stitching.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_superres.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_videostab.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_aruco.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_bgsegm.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_bioinspired.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_ccalib.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_dpm.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_fuzzy.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_line_descriptor.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_optflow.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_reg.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_saliency.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_stereo.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_structured_light.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_surface_matching.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_tracking.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_xfeatures2d.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_ximgproc.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_xobjdetect.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_xphoto.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_cudafeatures2d.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_shape.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_cudacodec.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_cudaoptflow.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_cudalegacy.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_cudawarping.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_phase_unwrapping.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_rgbd.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_calib3d.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_video.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_datasets.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_dnn.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_face.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_plot.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_text.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_features2d.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_flann.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_objdetect.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_ml.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_highgui.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_photo.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_cudaimgproc.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_cudafilters.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_cudaarithm.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_videoio.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_imgcodecs.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_imgproc.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_core.so.3.2.0
ObstacleDetection/obstacle_detection.plb: /opt/opencv/3.2.0/lib/libopencv_cudev.so.3.2.0
ObstacleDetection/obstacle_detection.plb: ObstacleDetection/CMakeFiles/obstacle_detection.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/aadc/ADTF/aadc2017/src/aadcUser/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared module obstacle_detection.plb"
	cd /home/aadc/ADTF/aadc2017/src/aadcUser/ObstacleDetection && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/obstacle_detection.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
ObstacleDetection/CMakeFiles/obstacle_detection.dir/build: ObstacleDetection/obstacle_detection.plb

.PHONY : ObstacleDetection/CMakeFiles/obstacle_detection.dir/build

ObstacleDetection/CMakeFiles/obstacle_detection.dir/requires: ObstacleDetection/CMakeFiles/obstacle_detection.dir/ObstacleDetection.cpp.o.requires

.PHONY : ObstacleDetection/CMakeFiles/obstacle_detection.dir/requires

ObstacleDetection/CMakeFiles/obstacle_detection.dir/clean:
	cd /home/aadc/ADTF/aadc2017/src/aadcUser/ObstacleDetection && $(CMAKE_COMMAND) -P CMakeFiles/obstacle_detection.dir/cmake_clean.cmake
.PHONY : ObstacleDetection/CMakeFiles/obstacle_detection.dir/clean

ObstacleDetection/CMakeFiles/obstacle_detection.dir/depend:
	cd /home/aadc/ADTF/aadc2017/src/aadcUser && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/aadc/ADTF/aadc2017/src/aadcUser /home/aadc/ADTF/aadc2017/src/aadcUser/ObstacleDetection /home/aadc/ADTF/aadc2017/src/aadcUser /home/aadc/ADTF/aadc2017/src/aadcUser/ObstacleDetection /home/aadc/ADTF/aadc2017/src/aadcUser/ObstacleDetection/CMakeFiles/obstacle_detection.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ObstacleDetection/CMakeFiles/obstacle_detection.dir/depend

