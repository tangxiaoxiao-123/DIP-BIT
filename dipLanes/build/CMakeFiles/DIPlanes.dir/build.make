# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.18

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

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = "C:\Program Files\CMake\bin\cmake.exe"

# The command to remove a file.
RM = "C:\Program Files\CMake\bin\cmake.exe" -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = D:\study\DIP\bigwork\dipLanes

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = D:\study\DIP\bigwork\dipLanes\build

# Include any dependencies generated for this target.
include CMakeFiles/DIPlanes.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/DIPlanes.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/DIPlanes.dir/flags.make

CMakeFiles/DIPlanes.dir/src/main.cpp.obj: CMakeFiles/DIPlanes.dir/flags.make
CMakeFiles/DIPlanes.dir/src/main.cpp.obj: CMakeFiles/DIPlanes.dir/includes_CXX.rsp
CMakeFiles/DIPlanes.dir/src/main.cpp.obj: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=D:\study\DIP\bigwork\dipLanes\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/DIPlanes.dir/src/main.cpp.obj"
	D:\x86_64-8.1.0-release-posix-seh-rt_v6-rev0\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles\DIPlanes.dir\src\main.cpp.obj -c D:\study\DIP\bigwork\dipLanes\src\main.cpp

CMakeFiles/DIPlanes.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/DIPlanes.dir/src/main.cpp.i"
	D:\x86_64-8.1.0-release-posix-seh-rt_v6-rev0\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E D:\study\DIP\bigwork\dipLanes\src\main.cpp > CMakeFiles\DIPlanes.dir\src\main.cpp.i

CMakeFiles/DIPlanes.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/DIPlanes.dir/src/main.cpp.s"
	D:\x86_64-8.1.0-release-posix-seh-rt_v6-rev0\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S D:\study\DIP\bigwork\dipLanes\src\main.cpp -o CMakeFiles\DIPlanes.dir\src\main.cpp.s

CMakeFiles/DIPlanes.dir/src/myGuass.cpp.obj: CMakeFiles/DIPlanes.dir/flags.make
CMakeFiles/DIPlanes.dir/src/myGuass.cpp.obj: CMakeFiles/DIPlanes.dir/includes_CXX.rsp
CMakeFiles/DIPlanes.dir/src/myGuass.cpp.obj: ../src/myGuass.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=D:\study\DIP\bigwork\dipLanes\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/DIPlanes.dir/src/myGuass.cpp.obj"
	D:\x86_64-8.1.0-release-posix-seh-rt_v6-rev0\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles\DIPlanes.dir\src\myGuass.cpp.obj -c D:\study\DIP\bigwork\dipLanes\src\myGuass.cpp

CMakeFiles/DIPlanes.dir/src/myGuass.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/DIPlanes.dir/src/myGuass.cpp.i"
	D:\x86_64-8.1.0-release-posix-seh-rt_v6-rev0\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E D:\study\DIP\bigwork\dipLanes\src\myGuass.cpp > CMakeFiles\DIPlanes.dir\src\myGuass.cpp.i

CMakeFiles/DIPlanes.dir/src/myGuass.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/DIPlanes.dir/src/myGuass.cpp.s"
	D:\x86_64-8.1.0-release-posix-seh-rt_v6-rev0\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S D:\study\DIP\bigwork\dipLanes\src\myGuass.cpp -o CMakeFiles\DIPlanes.dir\src\myGuass.cpp.s

CMakeFiles/DIPlanes.dir/src/myCanny.cpp.obj: CMakeFiles/DIPlanes.dir/flags.make
CMakeFiles/DIPlanes.dir/src/myCanny.cpp.obj: CMakeFiles/DIPlanes.dir/includes_CXX.rsp
CMakeFiles/DIPlanes.dir/src/myCanny.cpp.obj: ../src/myCanny.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=D:\study\DIP\bigwork\dipLanes\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/DIPlanes.dir/src/myCanny.cpp.obj"
	D:\x86_64-8.1.0-release-posix-seh-rt_v6-rev0\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles\DIPlanes.dir\src\myCanny.cpp.obj -c D:\study\DIP\bigwork\dipLanes\src\myCanny.cpp

CMakeFiles/DIPlanes.dir/src/myCanny.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/DIPlanes.dir/src/myCanny.cpp.i"
	D:\x86_64-8.1.0-release-posix-seh-rt_v6-rev0\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E D:\study\DIP\bigwork\dipLanes\src\myCanny.cpp > CMakeFiles\DIPlanes.dir\src\myCanny.cpp.i

CMakeFiles/DIPlanes.dir/src/myCanny.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/DIPlanes.dir/src/myCanny.cpp.s"
	D:\x86_64-8.1.0-release-posix-seh-rt_v6-rev0\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S D:\study\DIP\bigwork\dipLanes\src\myCanny.cpp -o CMakeFiles\DIPlanes.dir\src\myCanny.cpp.s

CMakeFiles/DIPlanes.dir/src/myHough.cpp.obj: CMakeFiles/DIPlanes.dir/flags.make
CMakeFiles/DIPlanes.dir/src/myHough.cpp.obj: CMakeFiles/DIPlanes.dir/includes_CXX.rsp
CMakeFiles/DIPlanes.dir/src/myHough.cpp.obj: ../src/myHough.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=D:\study\DIP\bigwork\dipLanes\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/DIPlanes.dir/src/myHough.cpp.obj"
	D:\x86_64-8.1.0-release-posix-seh-rt_v6-rev0\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles\DIPlanes.dir\src\myHough.cpp.obj -c D:\study\DIP\bigwork\dipLanes\src\myHough.cpp

CMakeFiles/DIPlanes.dir/src/myHough.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/DIPlanes.dir/src/myHough.cpp.i"
	D:\x86_64-8.1.0-release-posix-seh-rt_v6-rev0\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E D:\study\DIP\bigwork\dipLanes\src\myHough.cpp > CMakeFiles\DIPlanes.dir\src\myHough.cpp.i

CMakeFiles/DIPlanes.dir/src/myHough.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/DIPlanes.dir/src/myHough.cpp.s"
	D:\x86_64-8.1.0-release-posix-seh-rt_v6-rev0\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S D:\study\DIP\bigwork\dipLanes\src\myHough.cpp -o CMakeFiles\DIPlanes.dir\src\myHough.cpp.s

# Object files for target DIPlanes
DIPlanes_OBJECTS = \
"CMakeFiles/DIPlanes.dir/src/main.cpp.obj" \
"CMakeFiles/DIPlanes.dir/src/myGuass.cpp.obj" \
"CMakeFiles/DIPlanes.dir/src/myCanny.cpp.obj" \
"CMakeFiles/DIPlanes.dir/src/myHough.cpp.obj"

# External object files for target DIPlanes
DIPlanes_EXTERNAL_OBJECTS =

DIPlanes.exe: CMakeFiles/DIPlanes.dir/src/main.cpp.obj
DIPlanes.exe: CMakeFiles/DIPlanes.dir/src/myGuass.cpp.obj
DIPlanes.exe: CMakeFiles/DIPlanes.dir/src/myCanny.cpp.obj
DIPlanes.exe: CMakeFiles/DIPlanes.dir/src/myHough.cpp.obj
DIPlanes.exe: CMakeFiles/DIPlanes.dir/build.make
DIPlanes.exe: D:/OPENCV/bin/libopencv_calib3d451.dll
DIPlanes.exe: D:/OPENCV/bin/libopencv_core451.dll
DIPlanes.exe: D:/OPENCV/bin/libopencv_dnn451.dll
DIPlanes.exe: D:/OPENCV/bin/libopencv_features2d451.dll
DIPlanes.exe: D:/OPENCV/bin/libopencv_flann451.dll
DIPlanes.exe: D:/OPENCV/bin/libopencv_gapi451.dll
DIPlanes.exe: D:/OPENCV/bin/libopencv_highgui451.dll
DIPlanes.exe: D:/OPENCV/bin/libopencv_imgcodecs451.dll
DIPlanes.exe: D:/OPENCV/bin/libopencv_imgproc451.dll
DIPlanes.exe: D:/OPENCV/bin/libopencv_ml451.dll
DIPlanes.exe: D:/OPENCV/bin/libopencv_objdetect451.dll
DIPlanes.exe: D:/OPENCV/bin/libopencv_photo451.dll
DIPlanes.exe: D:/OPENCV/bin/libopencv_stitching451.dll
DIPlanes.exe: D:/OPENCV/bin/libopencv_video451.dll
DIPlanes.exe: D:/OPENCV/bin/libopencv_videoio451.dll
DIPlanes.exe: libmyJNI.dll.a
DIPlanes.exe: D:/OPENCV/bin/libopencv_dnn451.dll
DIPlanes.exe: D:/OPENCV/bin/libopencv_imgcodecs451.dll
DIPlanes.exe: D:/OPENCV/bin/libopencv_calib3d451.dll
DIPlanes.exe: D:/OPENCV/bin/libopencv_features2d451.dll
DIPlanes.exe: D:/OPENCV/bin/libopencv_flann451.dll
DIPlanes.exe: D:/OPENCV/bin/libopencv_imgproc451.dll
DIPlanes.exe: D:/OPENCV/bin/libopencv_core451.dll
DIPlanes.exe: CMakeFiles/DIPlanes.dir/linklibs.rsp
DIPlanes.exe: CMakeFiles/DIPlanes.dir/objects1.rsp
DIPlanes.exe: CMakeFiles/DIPlanes.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=D:\study\DIP\bigwork\dipLanes\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable DIPlanes.exe"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\DIPlanes.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/DIPlanes.dir/build: DIPlanes.exe

.PHONY : CMakeFiles/DIPlanes.dir/build

CMakeFiles/DIPlanes.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\DIPlanes.dir\cmake_clean.cmake
.PHONY : CMakeFiles/DIPlanes.dir/clean

CMakeFiles/DIPlanes.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" D:\study\DIP\bigwork\dipLanes D:\study\DIP\bigwork\dipLanes D:\study\DIP\bigwork\dipLanes\build D:\study\DIP\bigwork\dipLanes\build D:\study\DIP\bigwork\dipLanes\build\CMakeFiles\DIPlanes.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/DIPlanes.dir/depend

