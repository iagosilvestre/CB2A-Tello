# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.28

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/oem/tello_ros_ws/src/CB2A-Tello/detectRed

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/oem/tello_ros_ws/src/CB2A-Tello/detectRed/build

# Include any dependencies generated for this target.
include CMakeFiles/detect_redPixel_image.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/detect_redPixel_image.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/detect_redPixel_image.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/detect_redPixel_image.dir/flags.make

CMakeFiles/detect_redPixel_image.dir/detect_redPixel_image.cpp.o: CMakeFiles/detect_redPixel_image.dir/flags.make
CMakeFiles/detect_redPixel_image.dir/detect_redPixel_image.cpp.o: /home/oem/tello_ros_ws/src/CB2A-Tello/detectRed/detect_redPixel_image.cpp
CMakeFiles/detect_redPixel_image.dir/detect_redPixel_image.cpp.o: CMakeFiles/detect_redPixel_image.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/oem/tello_ros_ws/src/CB2A-Tello/detectRed/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/detect_redPixel_image.dir/detect_redPixel_image.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/detect_redPixel_image.dir/detect_redPixel_image.cpp.o -MF CMakeFiles/detect_redPixel_image.dir/detect_redPixel_image.cpp.o.d -o CMakeFiles/detect_redPixel_image.dir/detect_redPixel_image.cpp.o -c /home/oem/tello_ros_ws/src/CB2A-Tello/detectRed/detect_redPixel_image.cpp

CMakeFiles/detect_redPixel_image.dir/detect_redPixel_image.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/detect_redPixel_image.dir/detect_redPixel_image.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/oem/tello_ros_ws/src/CB2A-Tello/detectRed/detect_redPixel_image.cpp > CMakeFiles/detect_redPixel_image.dir/detect_redPixel_image.cpp.i

CMakeFiles/detect_redPixel_image.dir/detect_redPixel_image.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/detect_redPixel_image.dir/detect_redPixel_image.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/oem/tello_ros_ws/src/CB2A-Tello/detectRed/detect_redPixel_image.cpp -o CMakeFiles/detect_redPixel_image.dir/detect_redPixel_image.cpp.s

# Object files for target detect_redPixel_image
detect_redPixel_image_OBJECTS = \
"CMakeFiles/detect_redPixel_image.dir/detect_redPixel_image.cpp.o"

# External object files for target detect_redPixel_image
detect_redPixel_image_EXTERNAL_OBJECTS =

detect_redPixel_image: CMakeFiles/detect_redPixel_image.dir/detect_redPixel_image.cpp.o
detect_redPixel_image: CMakeFiles/detect_redPixel_image.dir/build.make
detect_redPixel_image: /opt/ros/foxy/lib/librclcpp.so
detect_redPixel_image: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_introspection_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_introspection_cpp.so
detect_redPixel_image: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_cpp.so
detect_redPixel_image: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_generator_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
detect_redPixel_image: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
detect_redPixel_image: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_generator_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
detect_redPixel_image: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
detect_redPixel_image: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_generator_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
detect_redPixel_image: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
detect_redPixel_image: /opt/ros/foxy/lib/librosidl_runtime_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_generator_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
detect_redPixel_image: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
detect_redPixel_image: /opt/ros/foxy/lib/librosidl_runtime_c.so
detect_redPixel_image: /opt/ros/foxy/lib/librosidl_runtime_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_generator_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
detect_redPixel_image: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
detect_redPixel_image: /opt/ros/foxy/lib/librosidl_runtime_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_generator_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
detect_redPixel_image: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
detect_redPixel_image: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_generator_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
detect_redPixel_image: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
detect_redPixel_image: /opt/ros/foxy/lib/librcutils.so
detect_redPixel_image: /opt/ros/foxy/lib/librcpputils.so
detect_redPixel_image: /opt/ros/foxy/lib/librosidl_typesupport_c.so
detect_redPixel_image: /opt/ros/foxy/lib/librosidl_typesupport_cpp.so
detect_redPixel_image: /opt/ros/foxy/lib/librosidl_runtime_c.so
detect_redPixel_image: /opt/ros/foxy/lib/librosidl_typesupport_introspection_c.so
detect_redPixel_image: /opt/ros/foxy/lib/librosidl_typesupport_introspection_cpp.so
detect_redPixel_image: /opt/ros/foxy/lib/libstd_msgs__rosidl_generator_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_cpp.so
detect_redPixel_image: /opt/ros/foxy/lib/liblibstatistics_collector.so
detect_redPixel_image: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_introspection_c.so
detect_redPixel_image: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_generator_c.so
detect_redPixel_image: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_c.so
detect_redPixel_image: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_introspection_cpp.so
detect_redPixel_image: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_cpp.so
detect_redPixel_image: /opt/ros/foxy/lib/librcl.so
detect_redPixel_image: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_introspection_c.so
detect_redPixel_image: /opt/ros/foxy/lib/librcl_interfaces__rosidl_generator_c.so
detect_redPixel_image: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_c.so
detect_redPixel_image: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_introspection_cpp.so
detect_redPixel_image: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_cpp.so
detect_redPixel_image: /opt/ros/foxy/lib/librmw_implementation.so
detect_redPixel_image: /opt/ros/foxy/lib/librmw.so
detect_redPixel_image: /opt/ros/foxy/lib/librcl_logging_spdlog.so
detect_redPixel_image: /usr/local/lib/libspdlog.a
detect_redPixel_image: /opt/ros/foxy/lib/librcl_yaml_param_parser.so
detect_redPixel_image: /opt/ros/foxy/lib/libyaml.so
detect_redPixel_image: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_typesupport_introspection_c.so
detect_redPixel_image: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_generator_c.so
detect_redPixel_image: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_typesupport_c.so
detect_redPixel_image: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_typesupport_introspection_cpp.so
detect_redPixel_image: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_typesupport_cpp.so
detect_redPixel_image: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_typesupport_introspection_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_generator_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_typesupport_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_typesupport_introspection_cpp.so
detect_redPixel_image: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_typesupport_cpp.so
detect_redPixel_image: /opt/ros/foxy/lib/libtracetools.so
detect_redPixel_image: /opt/ros/foxy/lib/libsensor_msgs__rosidl_generator_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_introspection_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_generator_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_introspection_cpp.so
detect_redPixel_image: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_cpp.so
detect_redPixel_image: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_introspection_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libstd_msgs__rosidl_generator_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_introspection_cpp.so
detect_redPixel_image: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_cpp.so
detect_redPixel_image: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_generator_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
detect_redPixel_image: /opt/ros/foxy/lib/librosidl_typesupport_introspection_cpp.so
detect_redPixel_image: /opt/ros/foxy/lib/librosidl_typesupport_introspection_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
detect_redPixel_image: /opt/ros/foxy/lib/librosidl_typesupport_cpp.so
detect_redPixel_image: /opt/ros/foxy/lib/librosidl_typesupport_c.so
detect_redPixel_image: /opt/ros/foxy/lib/librosidl_runtime_c.so
detect_redPixel_image: /opt/ros/foxy/lib/librcpputils.so
detect_redPixel_image: /opt/ros/foxy/lib/librcutils.so
detect_redPixel_image: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_generator_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
detect_redPixel_image: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
detect_redPixel_image: /opt/ros/foxy/lib/librcutils.so
detect_redPixel_image: /opt/ros/foxy/lib/librcpputils.so
detect_redPixel_image: /opt/ros/foxy/lib/librosidl_typesupport_c.so
detect_redPixel_image: /opt/ros/foxy/lib/librosidl_typesupport_cpp.so
detect_redPixel_image: /opt/ros/foxy/lib/librosidl_typesupport_introspection_c.so
detect_redPixel_image: /opt/ros/foxy/lib/librosidl_typesupport_introspection_cpp.so
detect_redPixel_image: /opt/ros/foxy/lib/libstd_msgs__rosidl_generator_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_c.so
detect_redPixel_image: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_cpp.so
detect_redPixel_image: CMakeFiles/detect_redPixel_image.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/oem/tello_ros_ws/src/CB2A-Tello/detectRed/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable detect_redPixel_image"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/detect_redPixel_image.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/detect_redPixel_image.dir/build: detect_redPixel_image
.PHONY : CMakeFiles/detect_redPixel_image.dir/build

CMakeFiles/detect_redPixel_image.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/detect_redPixel_image.dir/cmake_clean.cmake
.PHONY : CMakeFiles/detect_redPixel_image.dir/clean

CMakeFiles/detect_redPixel_image.dir/depend:
	cd /home/oem/tello_ros_ws/src/CB2A-Tello/detectRed/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/oem/tello_ros_ws/src/CB2A-Tello/detectRed /home/oem/tello_ros_ws/src/CB2A-Tello/detectRed /home/oem/tello_ros_ws/src/CB2A-Tello/detectRed/build /home/oem/tello_ros_ws/src/CB2A-Tello/detectRed/build /home/oem/tello_ros_ws/src/CB2A-Tello/detectRed/build/CMakeFiles/detect_redPixel_image.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/detect_redPixel_image.dir/depend
