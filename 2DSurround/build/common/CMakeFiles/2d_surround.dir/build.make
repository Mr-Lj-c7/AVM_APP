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
CMAKE_SOURCE_DIR = /home/user/Projects/360SurroundView_2D/2DSurround

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/user/Projects/360SurroundView_2D/2DSurround/build

# Include any dependencies generated for this target.
include common/CMakeFiles/2d_surround.dir/depend.make

# Include the progress variables for this target.
include common/CMakeFiles/2d_surround.dir/progress.make

# Include the compile flags for this target's objects.
include common/CMakeFiles/2d_surround.dir/flags.make

common/CMakeFiles/2d_surround.dir/src/ParamSettings.cpp.o: common/CMakeFiles/2d_surround.dir/flags.make
common/CMakeFiles/2d_surround.dir/src/ParamSettings.cpp.o: ../common/src/ParamSettings.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/Projects/360SurroundView_2D/2DSurround/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object common/CMakeFiles/2d_surround.dir/src/ParamSettings.cpp.o"
	cd /home/user/Projects/360SurroundView_2D/2DSurround/build/common && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/2d_surround.dir/src/ParamSettings.cpp.o -c /home/user/Projects/360SurroundView_2D/2DSurround/common/src/ParamSettings.cpp

common/CMakeFiles/2d_surround.dir/src/ParamSettings.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/2d_surround.dir/src/ParamSettings.cpp.i"
	cd /home/user/Projects/360SurroundView_2D/2DSurround/build/common && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/Projects/360SurroundView_2D/2DSurround/common/src/ParamSettings.cpp > CMakeFiles/2d_surround.dir/src/ParamSettings.cpp.i

common/CMakeFiles/2d_surround.dir/src/ParamSettings.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/2d_surround.dir/src/ParamSettings.cpp.s"
	cd /home/user/Projects/360SurroundView_2D/2DSurround/build/common && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/Projects/360SurroundView_2D/2DSurround/common/src/ParamSettings.cpp -o CMakeFiles/2d_surround.dir/src/ParamSettings.cpp.s

common/CMakeFiles/2d_surround.dir/src/FisheysCameraModel.cpp.o: common/CMakeFiles/2d_surround.dir/flags.make
common/CMakeFiles/2d_surround.dir/src/FisheysCameraModel.cpp.o: ../common/src/FisheysCameraModel.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/Projects/360SurroundView_2D/2DSurround/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object common/CMakeFiles/2d_surround.dir/src/FisheysCameraModel.cpp.o"
	cd /home/user/Projects/360SurroundView_2D/2DSurround/build/common && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/2d_surround.dir/src/FisheysCameraModel.cpp.o -c /home/user/Projects/360SurroundView_2D/2DSurround/common/src/FisheysCameraModel.cpp

common/CMakeFiles/2d_surround.dir/src/FisheysCameraModel.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/2d_surround.dir/src/FisheysCameraModel.cpp.i"
	cd /home/user/Projects/360SurroundView_2D/2DSurround/build/common && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/Projects/360SurroundView_2D/2DSurround/common/src/FisheysCameraModel.cpp > CMakeFiles/2d_surround.dir/src/FisheysCameraModel.cpp.i

common/CMakeFiles/2d_surround.dir/src/FisheysCameraModel.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/2d_surround.dir/src/FisheysCameraModel.cpp.s"
	cd /home/user/Projects/360SurroundView_2D/2DSurround/build/common && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/Projects/360SurroundView_2D/2DSurround/common/src/FisheysCameraModel.cpp -o CMakeFiles/2d_surround.dir/src/FisheysCameraModel.cpp.s

common/CMakeFiles/2d_surround.dir/src/CalibrateCamera.cpp.o: common/CMakeFiles/2d_surround.dir/flags.make
common/CMakeFiles/2d_surround.dir/src/CalibrateCamera.cpp.o: ../common/src/CalibrateCamera.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/Projects/360SurroundView_2D/2DSurround/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object common/CMakeFiles/2d_surround.dir/src/CalibrateCamera.cpp.o"
	cd /home/user/Projects/360SurroundView_2D/2DSurround/build/common && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/2d_surround.dir/src/CalibrateCamera.cpp.o -c /home/user/Projects/360SurroundView_2D/2DSurround/common/src/CalibrateCamera.cpp

common/CMakeFiles/2d_surround.dir/src/CalibrateCamera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/2d_surround.dir/src/CalibrateCamera.cpp.i"
	cd /home/user/Projects/360SurroundView_2D/2DSurround/build/common && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/Projects/360SurroundView_2D/2DSurround/common/src/CalibrateCamera.cpp > CMakeFiles/2d_surround.dir/src/CalibrateCamera.cpp.i

common/CMakeFiles/2d_surround.dir/src/CalibrateCamera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/2d_surround.dir/src/CalibrateCamera.cpp.s"
	cd /home/user/Projects/360SurroundView_2D/2DSurround/build/common && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/Projects/360SurroundView_2D/2DSurround/common/src/CalibrateCamera.cpp -o CMakeFiles/2d_surround.dir/src/CalibrateCamera.cpp.s

common/CMakeFiles/2d_surround.dir/src/BirdView.cpp.o: common/CMakeFiles/2d_surround.dir/flags.make
common/CMakeFiles/2d_surround.dir/src/BirdView.cpp.o: ../common/src/BirdView.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/Projects/360SurroundView_2D/2DSurround/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object common/CMakeFiles/2d_surround.dir/src/BirdView.cpp.o"
	cd /home/user/Projects/360SurroundView_2D/2DSurround/build/common && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/2d_surround.dir/src/BirdView.cpp.o -c /home/user/Projects/360SurroundView_2D/2DSurround/common/src/BirdView.cpp

common/CMakeFiles/2d_surround.dir/src/BirdView.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/2d_surround.dir/src/BirdView.cpp.i"
	cd /home/user/Projects/360SurroundView_2D/2DSurround/build/common && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/Projects/360SurroundView_2D/2DSurround/common/src/BirdView.cpp > CMakeFiles/2d_surround.dir/src/BirdView.cpp.i

common/CMakeFiles/2d_surround.dir/src/BirdView.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/2d_surround.dir/src/BirdView.cpp.s"
	cd /home/user/Projects/360SurroundView_2D/2DSurround/build/common && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/Projects/360SurroundView_2D/2DSurround/common/src/BirdView.cpp -o CMakeFiles/2d_surround.dir/src/BirdView.cpp.s

common/CMakeFiles/2d_surround.dir/src/UtilsView.cpp.o: common/CMakeFiles/2d_surround.dir/flags.make
common/CMakeFiles/2d_surround.dir/src/UtilsView.cpp.o: ../common/src/UtilsView.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/Projects/360SurroundView_2D/2DSurround/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object common/CMakeFiles/2d_surround.dir/src/UtilsView.cpp.o"
	cd /home/user/Projects/360SurroundView_2D/2DSurround/build/common && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/2d_surround.dir/src/UtilsView.cpp.o -c /home/user/Projects/360SurroundView_2D/2DSurround/common/src/UtilsView.cpp

common/CMakeFiles/2d_surround.dir/src/UtilsView.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/2d_surround.dir/src/UtilsView.cpp.i"
	cd /home/user/Projects/360SurroundView_2D/2DSurround/build/common && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/Projects/360SurroundView_2D/2DSurround/common/src/UtilsView.cpp > CMakeFiles/2d_surround.dir/src/UtilsView.cpp.i

common/CMakeFiles/2d_surround.dir/src/UtilsView.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/2d_surround.dir/src/UtilsView.cpp.s"
	cd /home/user/Projects/360SurroundView_2D/2DSurround/build/common && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/Projects/360SurroundView_2D/2DSurround/common/src/UtilsView.cpp -o CMakeFiles/2d_surround.dir/src/UtilsView.cpp.s

# Object files for target 2d_surround
2d_surround_OBJECTS = \
"CMakeFiles/2d_surround.dir/src/ParamSettings.cpp.o" \
"CMakeFiles/2d_surround.dir/src/FisheysCameraModel.cpp.o" \
"CMakeFiles/2d_surround.dir/src/CalibrateCamera.cpp.o" \
"CMakeFiles/2d_surround.dir/src/BirdView.cpp.o" \
"CMakeFiles/2d_surround.dir/src/UtilsView.cpp.o"

# External object files for target 2d_surround
2d_surround_EXTERNAL_OBJECTS =

../lib/lib2d_surround.so: common/CMakeFiles/2d_surround.dir/src/ParamSettings.cpp.o
../lib/lib2d_surround.so: common/CMakeFiles/2d_surround.dir/src/FisheysCameraModel.cpp.o
../lib/lib2d_surround.so: common/CMakeFiles/2d_surround.dir/src/CalibrateCamera.cpp.o
../lib/lib2d_surround.so: common/CMakeFiles/2d_surround.dir/src/BirdView.cpp.o
../lib/lib2d_surround.so: common/CMakeFiles/2d_surround.dir/src/UtilsView.cpp.o
../lib/lib2d_surround.so: common/CMakeFiles/2d_surround.dir/build.make
../lib/lib2d_surround.so: /usr/local/lib/libopencv_gapi.so.4.5.5
../lib/lib2d_surround.so: /usr/local/lib/libopencv_highgui.so.4.5.5
../lib/lib2d_surround.so: /usr/local/lib/libopencv_ml.so.4.5.5
../lib/lib2d_surround.so: /usr/local/lib/libopencv_objdetect.so.4.5.5
../lib/lib2d_surround.so: /usr/local/lib/libopencv_photo.so.4.5.5
../lib/lib2d_surround.so: /usr/local/lib/libopencv_stitching.so.4.5.5
../lib/lib2d_surround.so: /usr/local/lib/libopencv_video.so.4.5.5
../lib/lib2d_surround.so: /usr/local/lib/libopencv_videoio.so.4.5.5
../lib/lib2d_surround.so: /usr/local/lib/libopencv_imgcodecs.so.4.5.5
../lib/lib2d_surround.so: /usr/local/lib/libopencv_dnn.so.4.5.5
../lib/lib2d_surround.so: /usr/local/lib/libopencv_calib3d.so.4.5.5
../lib/lib2d_surround.so: /usr/local/lib/libopencv_features2d.so.4.5.5
../lib/lib2d_surround.so: /usr/local/lib/libopencv_flann.so.4.5.5
../lib/lib2d_surround.so: /usr/local/lib/libopencv_imgproc.so.4.5.5
../lib/lib2d_surround.so: /usr/local/lib/libopencv_core.so.4.5.5
../lib/lib2d_surround.so: common/CMakeFiles/2d_surround.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/user/Projects/360SurroundView_2D/2DSurround/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX shared library ../../lib/lib2d_surround.so"
	cd /home/user/Projects/360SurroundView_2D/2DSurround/build/common && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/2d_surround.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
common/CMakeFiles/2d_surround.dir/build: ../lib/lib2d_surround.so

.PHONY : common/CMakeFiles/2d_surround.dir/build

common/CMakeFiles/2d_surround.dir/clean:
	cd /home/user/Projects/360SurroundView_2D/2DSurround/build/common && $(CMAKE_COMMAND) -P CMakeFiles/2d_surround.dir/cmake_clean.cmake
.PHONY : common/CMakeFiles/2d_surround.dir/clean

common/CMakeFiles/2d_surround.dir/depend:
	cd /home/user/Projects/360SurroundView_2D/2DSurround/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/Projects/360SurroundView_2D/2DSurround /home/user/Projects/360SurroundView_2D/2DSurround/common /home/user/Projects/360SurroundView_2D/2DSurround/build /home/user/Projects/360SurroundView_2D/2DSurround/build/common /home/user/Projects/360SurroundView_2D/2DSurround/build/common/CMakeFiles/2d_surround.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : common/CMakeFiles/2d_surround.dir/depend

