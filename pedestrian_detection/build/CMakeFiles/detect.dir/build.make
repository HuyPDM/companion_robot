# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/jetson/Desktop/person_detection/pedestrian_detection

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jetson/Desktop/person_detection/pedestrian_detection/build

# Include any dependencies generated for this target.
include CMakeFiles/detect.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/detect.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/detect.dir/flags.make

CMakeFiles/detect.dir/test.cpp.o: CMakeFiles/detect.dir/flags.make
CMakeFiles/detect.dir/test.cpp.o: ../test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jetson/Desktop/person_detection/pedestrian_detection/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/detect.dir/test.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/detect.dir/test.cpp.o -c /home/jetson/Desktop/person_detection/pedestrian_detection/test.cpp

CMakeFiles/detect.dir/test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/detect.dir/test.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jetson/Desktop/person_detection/pedestrian_detection/test.cpp > CMakeFiles/detect.dir/test.cpp.i

CMakeFiles/detect.dir/test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/detect.dir/test.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jetson/Desktop/person_detection/pedestrian_detection/test.cpp -o CMakeFiles/detect.dir/test.cpp.s

CMakeFiles/detect.dir/test.cpp.o.requires:

.PHONY : CMakeFiles/detect.dir/test.cpp.o.requires

CMakeFiles/detect.dir/test.cpp.o.provides: CMakeFiles/detect.dir/test.cpp.o.requires
	$(MAKE) -f CMakeFiles/detect.dir/build.make CMakeFiles/detect.dir/test.cpp.o.provides.build
.PHONY : CMakeFiles/detect.dir/test.cpp.o.provides

CMakeFiles/detect.dir/test.cpp.o.provides.build: CMakeFiles/detect.dir/test.cpp.o


CMakeFiles/detect.dir/object_detection.cpp.o: CMakeFiles/detect.dir/flags.make
CMakeFiles/detect.dir/object_detection.cpp.o: ../object_detection.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jetson/Desktop/person_detection/pedestrian_detection/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/detect.dir/object_detection.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/detect.dir/object_detection.cpp.o -c /home/jetson/Desktop/person_detection/pedestrian_detection/object_detection.cpp

CMakeFiles/detect.dir/object_detection.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/detect.dir/object_detection.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jetson/Desktop/person_detection/pedestrian_detection/object_detection.cpp > CMakeFiles/detect.dir/object_detection.cpp.i

CMakeFiles/detect.dir/object_detection.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/detect.dir/object_detection.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jetson/Desktop/person_detection/pedestrian_detection/object_detection.cpp -o CMakeFiles/detect.dir/object_detection.cpp.s

CMakeFiles/detect.dir/object_detection.cpp.o.requires:

.PHONY : CMakeFiles/detect.dir/object_detection.cpp.o.requires

CMakeFiles/detect.dir/object_detection.cpp.o.provides: CMakeFiles/detect.dir/object_detection.cpp.o.requires
	$(MAKE) -f CMakeFiles/detect.dir/build.make CMakeFiles/detect.dir/object_detection.cpp.o.provides.build
.PHONY : CMakeFiles/detect.dir/object_detection.cpp.o.provides

CMakeFiles/detect.dir/object_detection.cpp.o.provides.build: CMakeFiles/detect.dir/object_detection.cpp.o


CMakeFiles/detect.dir/camera_tracking.cpp.o: CMakeFiles/detect.dir/flags.make
CMakeFiles/detect.dir/camera_tracking.cpp.o: ../camera_tracking.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jetson/Desktop/person_detection/pedestrian_detection/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/detect.dir/camera_tracking.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/detect.dir/camera_tracking.cpp.o -c /home/jetson/Desktop/person_detection/pedestrian_detection/camera_tracking.cpp

CMakeFiles/detect.dir/camera_tracking.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/detect.dir/camera_tracking.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jetson/Desktop/person_detection/pedestrian_detection/camera_tracking.cpp > CMakeFiles/detect.dir/camera_tracking.cpp.i

CMakeFiles/detect.dir/camera_tracking.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/detect.dir/camera_tracking.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jetson/Desktop/person_detection/pedestrian_detection/camera_tracking.cpp -o CMakeFiles/detect.dir/camera_tracking.cpp.s

CMakeFiles/detect.dir/camera_tracking.cpp.o.requires:

.PHONY : CMakeFiles/detect.dir/camera_tracking.cpp.o.requires

CMakeFiles/detect.dir/camera_tracking.cpp.o.provides: CMakeFiles/detect.dir/camera_tracking.cpp.o.requires
	$(MAKE) -f CMakeFiles/detect.dir/build.make CMakeFiles/detect.dir/camera_tracking.cpp.o.provides.build
.PHONY : CMakeFiles/detect.dir/camera_tracking.cpp.o.provides

CMakeFiles/detect.dir/camera_tracking.cpp.o.provides.build: CMakeFiles/detect.dir/camera_tracking.cpp.o


CMakeFiles/detect.dir/robot_move.cpp.o: CMakeFiles/detect.dir/flags.make
CMakeFiles/detect.dir/robot_move.cpp.o: ../robot_move.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jetson/Desktop/person_detection/pedestrian_detection/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/detect.dir/robot_move.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/detect.dir/robot_move.cpp.o -c /home/jetson/Desktop/person_detection/pedestrian_detection/robot_move.cpp

CMakeFiles/detect.dir/robot_move.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/detect.dir/robot_move.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jetson/Desktop/person_detection/pedestrian_detection/robot_move.cpp > CMakeFiles/detect.dir/robot_move.cpp.i

CMakeFiles/detect.dir/robot_move.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/detect.dir/robot_move.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jetson/Desktop/person_detection/pedestrian_detection/robot_move.cpp -o CMakeFiles/detect.dir/robot_move.cpp.s

CMakeFiles/detect.dir/robot_move.cpp.o.requires:

.PHONY : CMakeFiles/detect.dir/robot_move.cpp.o.requires

CMakeFiles/detect.dir/robot_move.cpp.o.provides: CMakeFiles/detect.dir/robot_move.cpp.o.requires
	$(MAKE) -f CMakeFiles/detect.dir/build.make CMakeFiles/detect.dir/robot_move.cpp.o.provides.build
.PHONY : CMakeFiles/detect.dir/robot_move.cpp.o.provides

CMakeFiles/detect.dir/robot_move.cpp.o.provides.build: CMakeFiles/detect.dir/robot_move.cpp.o


CMakeFiles/detect.dir/src/anchor_creator.cpp.o: CMakeFiles/detect.dir/flags.make
CMakeFiles/detect.dir/src/anchor_creator.cpp.o: ../src/anchor_creator.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jetson/Desktop/person_detection/pedestrian_detection/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/detect.dir/src/anchor_creator.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/detect.dir/src/anchor_creator.cpp.o -c /home/jetson/Desktop/person_detection/pedestrian_detection/src/anchor_creator.cpp

CMakeFiles/detect.dir/src/anchor_creator.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/detect.dir/src/anchor_creator.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jetson/Desktop/person_detection/pedestrian_detection/src/anchor_creator.cpp > CMakeFiles/detect.dir/src/anchor_creator.cpp.i

CMakeFiles/detect.dir/src/anchor_creator.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/detect.dir/src/anchor_creator.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jetson/Desktop/person_detection/pedestrian_detection/src/anchor_creator.cpp -o CMakeFiles/detect.dir/src/anchor_creator.cpp.s

CMakeFiles/detect.dir/src/anchor_creator.cpp.o.requires:

.PHONY : CMakeFiles/detect.dir/src/anchor_creator.cpp.o.requires

CMakeFiles/detect.dir/src/anchor_creator.cpp.o.provides: CMakeFiles/detect.dir/src/anchor_creator.cpp.o.requires
	$(MAKE) -f CMakeFiles/detect.dir/build.make CMakeFiles/detect.dir/src/anchor_creator.cpp.o.provides.build
.PHONY : CMakeFiles/detect.dir/src/anchor_creator.cpp.o.provides

CMakeFiles/detect.dir/src/anchor_creator.cpp.o.provides.build: CMakeFiles/detect.dir/src/anchor_creator.cpp.o


CMakeFiles/detect.dir/src/config.cpp.o: CMakeFiles/detect.dir/flags.make
CMakeFiles/detect.dir/src/config.cpp.o: ../src/config.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jetson/Desktop/person_detection/pedestrian_detection/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/detect.dir/src/config.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/detect.dir/src/config.cpp.o -c /home/jetson/Desktop/person_detection/pedestrian_detection/src/config.cpp

CMakeFiles/detect.dir/src/config.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/detect.dir/src/config.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jetson/Desktop/person_detection/pedestrian_detection/src/config.cpp > CMakeFiles/detect.dir/src/config.cpp.i

CMakeFiles/detect.dir/src/config.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/detect.dir/src/config.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jetson/Desktop/person_detection/pedestrian_detection/src/config.cpp -o CMakeFiles/detect.dir/src/config.cpp.s

CMakeFiles/detect.dir/src/config.cpp.o.requires:

.PHONY : CMakeFiles/detect.dir/src/config.cpp.o.requires

CMakeFiles/detect.dir/src/config.cpp.o.provides: CMakeFiles/detect.dir/src/config.cpp.o.requires
	$(MAKE) -f CMakeFiles/detect.dir/build.make CMakeFiles/detect.dir/src/config.cpp.o.provides.build
.PHONY : CMakeFiles/detect.dir/src/config.cpp.o.provides

CMakeFiles/detect.dir/src/config.cpp.o.provides.build: CMakeFiles/detect.dir/src/config.cpp.o


CMakeFiles/detect.dir/src/fhog.cpp.o: CMakeFiles/detect.dir/flags.make
CMakeFiles/detect.dir/src/fhog.cpp.o: ../src/fhog.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jetson/Desktop/person_detection/pedestrian_detection/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/detect.dir/src/fhog.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/detect.dir/src/fhog.cpp.o -c /home/jetson/Desktop/person_detection/pedestrian_detection/src/fhog.cpp

CMakeFiles/detect.dir/src/fhog.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/detect.dir/src/fhog.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jetson/Desktop/person_detection/pedestrian_detection/src/fhog.cpp > CMakeFiles/detect.dir/src/fhog.cpp.i

CMakeFiles/detect.dir/src/fhog.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/detect.dir/src/fhog.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jetson/Desktop/person_detection/pedestrian_detection/src/fhog.cpp -o CMakeFiles/detect.dir/src/fhog.cpp.s

CMakeFiles/detect.dir/src/fhog.cpp.o.requires:

.PHONY : CMakeFiles/detect.dir/src/fhog.cpp.o.requires

CMakeFiles/detect.dir/src/fhog.cpp.o.provides: CMakeFiles/detect.dir/src/fhog.cpp.o.requires
	$(MAKE) -f CMakeFiles/detect.dir/build.make CMakeFiles/detect.dir/src/fhog.cpp.o.provides.build
.PHONY : CMakeFiles/detect.dir/src/fhog.cpp.o.provides

CMakeFiles/detect.dir/src/fhog.cpp.o.provides.build: CMakeFiles/detect.dir/src/fhog.cpp.o


CMakeFiles/detect.dir/src/kcftracker.cpp.o: CMakeFiles/detect.dir/flags.make
CMakeFiles/detect.dir/src/kcftracker.cpp.o: ../src/kcftracker.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jetson/Desktop/person_detection/pedestrian_detection/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/detect.dir/src/kcftracker.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/detect.dir/src/kcftracker.cpp.o -c /home/jetson/Desktop/person_detection/pedestrian_detection/src/kcftracker.cpp

CMakeFiles/detect.dir/src/kcftracker.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/detect.dir/src/kcftracker.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jetson/Desktop/person_detection/pedestrian_detection/src/kcftracker.cpp > CMakeFiles/detect.dir/src/kcftracker.cpp.i

CMakeFiles/detect.dir/src/kcftracker.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/detect.dir/src/kcftracker.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jetson/Desktop/person_detection/pedestrian_detection/src/kcftracker.cpp -o CMakeFiles/detect.dir/src/kcftracker.cpp.s

CMakeFiles/detect.dir/src/kcftracker.cpp.o.requires:

.PHONY : CMakeFiles/detect.dir/src/kcftracker.cpp.o.requires

CMakeFiles/detect.dir/src/kcftracker.cpp.o.provides: CMakeFiles/detect.dir/src/kcftracker.cpp.o.requires
	$(MAKE) -f CMakeFiles/detect.dir/build.make CMakeFiles/detect.dir/src/kcftracker.cpp.o.provides.build
.PHONY : CMakeFiles/detect.dir/src/kcftracker.cpp.o.provides

CMakeFiles/detect.dir/src/kcftracker.cpp.o.provides.build: CMakeFiles/detect.dir/src/kcftracker.cpp.o


CMakeFiles/detect.dir/src/utils.cpp.o: CMakeFiles/detect.dir/flags.make
CMakeFiles/detect.dir/src/utils.cpp.o: ../src/utils.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jetson/Desktop/person_detection/pedestrian_detection/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/detect.dir/src/utils.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/detect.dir/src/utils.cpp.o -c /home/jetson/Desktop/person_detection/pedestrian_detection/src/utils.cpp

CMakeFiles/detect.dir/src/utils.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/detect.dir/src/utils.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jetson/Desktop/person_detection/pedestrian_detection/src/utils.cpp > CMakeFiles/detect.dir/src/utils.cpp.i

CMakeFiles/detect.dir/src/utils.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/detect.dir/src/utils.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jetson/Desktop/person_detection/pedestrian_detection/src/utils.cpp -o CMakeFiles/detect.dir/src/utils.cpp.s

CMakeFiles/detect.dir/src/utils.cpp.o.requires:

.PHONY : CMakeFiles/detect.dir/src/utils.cpp.o.requires

CMakeFiles/detect.dir/src/utils.cpp.o.provides: CMakeFiles/detect.dir/src/utils.cpp.o.requires
	$(MAKE) -f CMakeFiles/detect.dir/build.make CMakeFiles/detect.dir/src/utils.cpp.o.provides.build
.PHONY : CMakeFiles/detect.dir/src/utils.cpp.o.provides

CMakeFiles/detect.dir/src/utils.cpp.o.provides.build: CMakeFiles/detect.dir/src/utils.cpp.o


# Object files for target detect
detect_OBJECTS = \
"CMakeFiles/detect.dir/test.cpp.o" \
"CMakeFiles/detect.dir/object_detection.cpp.o" \
"CMakeFiles/detect.dir/camera_tracking.cpp.o" \
"CMakeFiles/detect.dir/robot_move.cpp.o" \
"CMakeFiles/detect.dir/src/anchor_creator.cpp.o" \
"CMakeFiles/detect.dir/src/config.cpp.o" \
"CMakeFiles/detect.dir/src/fhog.cpp.o" \
"CMakeFiles/detect.dir/src/kcftracker.cpp.o" \
"CMakeFiles/detect.dir/src/utils.cpp.o"

# External object files for target detect
detect_EXTERNAL_OBJECTS =

detect: CMakeFiles/detect.dir/test.cpp.o
detect: CMakeFiles/detect.dir/object_detection.cpp.o
detect: CMakeFiles/detect.dir/camera_tracking.cpp.o
detect: CMakeFiles/detect.dir/robot_move.cpp.o
detect: CMakeFiles/detect.dir/src/anchor_creator.cpp.o
detect: CMakeFiles/detect.dir/src/config.cpp.o
detect: CMakeFiles/detect.dir/src/fhog.cpp.o
detect: CMakeFiles/detect.dir/src/kcftracker.cpp.o
detect: CMakeFiles/detect.dir/src/utils.cpp.o
detect: CMakeFiles/detect.dir/build.make
detect: /usr/local/lib/libopencv_gapi.so.4.4.0
detect: /usr/local/lib/libopencv_stitching.so.4.4.0
detect: /usr/local/lib/libopencv_alphamat.so.4.4.0
detect: /usr/local/lib/libopencv_aruco.so.4.4.0
detect: /usr/local/lib/libopencv_bgsegm.so.4.4.0
detect: /usr/local/lib/libopencv_bioinspired.so.4.4.0
detect: /usr/local/lib/libopencv_ccalib.so.4.4.0
detect: /usr/local/lib/libopencv_cudabgsegm.so.4.4.0
detect: /usr/local/lib/libopencv_cudafeatures2d.so.4.4.0
detect: /usr/local/lib/libopencv_cudaobjdetect.so.4.4.0
detect: /usr/local/lib/libopencv_cudastereo.so.4.4.0
detect: /usr/local/lib/libopencv_dnn_objdetect.so.4.4.0
detect: /usr/local/lib/libopencv_dnn_superres.so.4.4.0
detect: /usr/local/lib/libopencv_dpm.so.4.4.0
detect: /usr/local/lib/libopencv_face.so.4.4.0
detect: /usr/local/lib/libopencv_freetype.so.4.4.0
detect: /usr/local/lib/libopencv_fuzzy.so.4.4.0
detect: /usr/local/lib/libopencv_hfs.so.4.4.0
detect: /usr/local/lib/libopencv_img_hash.so.4.4.0
detect: /usr/local/lib/libopencv_intensity_transform.so.4.4.0
detect: /usr/local/lib/libopencv_line_descriptor.so.4.4.0
detect: /usr/local/lib/libopencv_quality.so.4.4.0
detect: /usr/local/lib/libopencv_rapid.so.4.4.0
detect: /usr/local/lib/libopencv_reg.so.4.4.0
detect: /usr/local/lib/libopencv_rgbd.so.4.4.0
detect: /usr/local/lib/libopencv_saliency.so.4.4.0
detect: /usr/local/lib/libopencv_stereo.so.4.4.0
detect: /usr/local/lib/libopencv_structured_light.so.4.4.0
detect: /usr/local/lib/libopencv_superres.so.4.4.0
detect: /usr/local/lib/libopencv_surface_matching.so.4.4.0
detect: /usr/local/lib/libopencv_tracking.so.4.4.0
detect: /usr/local/lib/libopencv_videostab.so.4.4.0
detect: /usr/local/lib/libopencv_xfeatures2d.so.4.4.0
detect: /usr/local/lib/libopencv_xobjdetect.so.4.4.0
detect: /usr/local/lib/libopencv_xphoto.so.4.4.0
detect: ncnn_build/src/libncnn.a
detect: /usr/local/lib/libopencv_shape.so.4.4.0
detect: /usr/local/lib/libopencv_highgui.so.4.4.0
detect: /usr/local/lib/libopencv_datasets.so.4.4.0
detect: /usr/local/lib/libopencv_plot.so.4.4.0
detect: /usr/local/lib/libopencv_text.so.4.4.0
detect: /usr/local/lib/libopencv_dnn.so.4.4.0
detect: /usr/local/lib/libopencv_ml.so.4.4.0
detect: /usr/local/lib/libopencv_phase_unwrapping.so.4.4.0
detect: /usr/local/lib/libopencv_cudacodec.so.4.4.0
detect: /usr/local/lib/libopencv_videoio.so.4.4.0
detect: /usr/local/lib/libopencv_cudaoptflow.so.4.4.0
detect: /usr/local/lib/libopencv_cudalegacy.so.4.4.0
detect: /usr/local/lib/libopencv_cudawarping.so.4.4.0
detect: /usr/local/lib/libopencv_optflow.so.4.4.0
detect: /usr/local/lib/libopencv_ximgproc.so.4.4.0
detect: /usr/local/lib/libopencv_video.so.4.4.0
detect: /usr/local/lib/libopencv_imgcodecs.so.4.4.0
detect: /usr/local/lib/libopencv_objdetect.so.4.4.0
detect: /usr/local/lib/libopencv_calib3d.so.4.4.0
detect: /usr/local/lib/libopencv_features2d.so.4.4.0
detect: /usr/local/lib/libopencv_flann.so.4.4.0
detect: /usr/local/lib/libopencv_photo.so.4.4.0
detect: /usr/local/lib/libopencv_cudaimgproc.so.4.4.0
detect: /usr/local/lib/libopencv_cudafilters.so.4.4.0
detect: /usr/local/lib/libopencv_imgproc.so.4.4.0
detect: /usr/local/lib/libopencv_cudaarithm.so.4.4.0
detect: /usr/local/lib/libopencv_core.so.4.4.0
detect: /usr/local/lib/libopencv_cudev.so.4.4.0
detect: /usr/lib/gcc/aarch64-linux-gnu/7/libgomp.so
detect: /usr/lib/aarch64-linux-gnu/libpthread.so
detect: /usr/lib/aarch64-linux-gnu/libvulkan.so
detect: ncnn_build/glslang/glslang/libglslang.a
detect: ncnn_build/glslang/SPIRV/libSPIRV.a
detect: ncnn_build/glslang/glslang/libMachineIndependent.a
detect: ncnn_build/glslang/OGLCompilersDLL/libOGLCompiler.a
detect: ncnn_build/glslang/glslang/OSDependent/Unix/libOSDependent.a
detect: ncnn_build/glslang/glslang/libGenericCodeGen.a
detect: CMakeFiles/detect.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jetson/Desktop/person_detection/pedestrian_detection/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Linking CXX executable detect"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/detect.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/detect.dir/build: detect

.PHONY : CMakeFiles/detect.dir/build

CMakeFiles/detect.dir/requires: CMakeFiles/detect.dir/test.cpp.o.requires
CMakeFiles/detect.dir/requires: CMakeFiles/detect.dir/object_detection.cpp.o.requires
CMakeFiles/detect.dir/requires: CMakeFiles/detect.dir/camera_tracking.cpp.o.requires
CMakeFiles/detect.dir/requires: CMakeFiles/detect.dir/robot_move.cpp.o.requires
CMakeFiles/detect.dir/requires: CMakeFiles/detect.dir/src/anchor_creator.cpp.o.requires
CMakeFiles/detect.dir/requires: CMakeFiles/detect.dir/src/config.cpp.o.requires
CMakeFiles/detect.dir/requires: CMakeFiles/detect.dir/src/fhog.cpp.o.requires
CMakeFiles/detect.dir/requires: CMakeFiles/detect.dir/src/kcftracker.cpp.o.requires
CMakeFiles/detect.dir/requires: CMakeFiles/detect.dir/src/utils.cpp.o.requires

.PHONY : CMakeFiles/detect.dir/requires

CMakeFiles/detect.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/detect.dir/cmake_clean.cmake
.PHONY : CMakeFiles/detect.dir/clean

CMakeFiles/detect.dir/depend:
	cd /home/jetson/Desktop/person_detection/pedestrian_detection/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jetson/Desktop/person_detection/pedestrian_detection /home/jetson/Desktop/person_detection/pedestrian_detection /home/jetson/Desktop/person_detection/pedestrian_detection/build /home/jetson/Desktop/person_detection/pedestrian_detection/build /home/jetson/Desktop/person_detection/pedestrian_detection/build/CMakeFiles/detect.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/detect.dir/depend
