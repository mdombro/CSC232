# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/matthew/CSC232/Lab5

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/matthew/CSC232/Lab5/build

# Include any dependencies generated for this target.
include CMakeFiles/pfc.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/pfc.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/pfc.dir/flags.make

CMakeFiles/pfc.dir/pfc.cpp.o: CMakeFiles/pfc.dir/flags.make
CMakeFiles/pfc.dir/pfc.cpp.o: ../pfc.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/matthew/CSC232/Lab5/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/pfc.dir/pfc.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/pfc.dir/pfc.cpp.o -c /home/matthew/CSC232/Lab5/pfc.cpp

CMakeFiles/pfc.dir/pfc.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pfc.dir/pfc.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/matthew/CSC232/Lab5/pfc.cpp > CMakeFiles/pfc.dir/pfc.cpp.i

CMakeFiles/pfc.dir/pfc.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pfc.dir/pfc.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/matthew/CSC232/Lab5/pfc.cpp -o CMakeFiles/pfc.dir/pfc.cpp.s

CMakeFiles/pfc.dir/pfc.cpp.o.requires:
.PHONY : CMakeFiles/pfc.dir/pfc.cpp.o.requires

CMakeFiles/pfc.dir/pfc.cpp.o.provides: CMakeFiles/pfc.dir/pfc.cpp.o.requires
	$(MAKE) -f CMakeFiles/pfc.dir/build.make CMakeFiles/pfc.dir/pfc.cpp.o.provides.build
.PHONY : CMakeFiles/pfc.dir/pfc.cpp.o.provides

CMakeFiles/pfc.dir/pfc.cpp.o.provides.build: CMakeFiles/pfc.dir/pfc.cpp.o

CMakeFiles/pfc.dir/point.cpp.o: CMakeFiles/pfc.dir/flags.make
CMakeFiles/pfc.dir/point.cpp.o: ../point.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/matthew/CSC232/Lab5/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/pfc.dir/point.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/pfc.dir/point.cpp.o -c /home/matthew/CSC232/Lab5/point.cpp

CMakeFiles/pfc.dir/point.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pfc.dir/point.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/matthew/CSC232/Lab5/point.cpp > CMakeFiles/pfc.dir/point.cpp.i

CMakeFiles/pfc.dir/point.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pfc.dir/point.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/matthew/CSC232/Lab5/point.cpp -o CMakeFiles/pfc.dir/point.cpp.s

CMakeFiles/pfc.dir/point.cpp.o.requires:
.PHONY : CMakeFiles/pfc.dir/point.cpp.o.requires

CMakeFiles/pfc.dir/point.cpp.o.provides: CMakeFiles/pfc.dir/point.cpp.o.requires
	$(MAKE) -f CMakeFiles/pfc.dir/build.make CMakeFiles/pfc.dir/point.cpp.o.provides.build
.PHONY : CMakeFiles/pfc.dir/point.cpp.o.provides

CMakeFiles/pfc.dir/point.cpp.o.provides.build: CMakeFiles/pfc.dir/point.cpp.o

# Object files for target pfc
pfc_OBJECTS = \
"CMakeFiles/pfc.dir/pfc.cpp.o" \
"CMakeFiles/pfc.dir/point.cpp.o"

# External object files for target pfc
pfc_EXTERNAL_OBJECTS =

pfc: CMakeFiles/pfc.dir/pfc.cpp.o
pfc: CMakeFiles/pfc.dir/point.cpp.o
pfc: CMakeFiles/pfc.dir/build.make
pfc: /opt/ros/indigo/lib/librviz_plugin_tutorials.so
pfc: /opt/ros/indigo/lib/liboctomap.so.1.6
pfc: /opt/ros/indigo/lib/libcompressed_depth_image_transport.so
pfc: /opt/ros/indigo/lib/libgazebo_ros_video.so
pfc: /opt/ros/indigo/lib/librospack.so
pfc: /opt/ros/indigo/lib/libecl_type_traits.so
pfc: /opt/ros/indigo/lib/libnavfn.so
pfc: /opt/ros/indigo/lib/libcollada_parser_plugin.so
pfc: /opt/ros/indigo/lib/libecl_mobile_robot.so
pfc: /opt/ros/indigo/lib/libsensor_base.so
pfc: /opt/ros/indigo/lib/librosconsole_backend_interface.so
pfc: /opt/ros/indigo/lib/libimage_loader.so
pfc: /opt/ros/indigo/lib/libgazebo_ros_joint_state_publisher.so
pfc: /opt/ros/indigo/lib/libmessage_filters.so
pfc: /opt/ros/indigo/lib/librqt_gui_cpp.so
pfc: /opt/ros/indigo/lib/libcollada_urdf.so
pfc: /opt/ros/indigo/lib/libfreenect_nodelet.so
pfc: /opt/ros/indigo/lib/libbase_local_planner.so
pfc: /opt/ros/indigo/lib/libgazebo_ros_camera.so
pfc: /opt/ros/indigo/lib/libfreenect_sync.so.0.5.1
pfc: /opt/ros/indigo/lib/libdepth_image_proc.so
pfc: /opt/ros/indigo/lib/libnodelet_math.so
pfc: /opt/ros/indigo/lib/libclear_costmap_recovery.so
pfc: /opt/ros/indigo/lib/libroscpp.so
pfc: /opt/ros/indigo/lib/libinteractive_markers.so
pfc: /opt/ros/indigo/lib/libpcl_ros_surface.so
pfc: /opt/ros/indigo/lib/libcv_bridge.so
pfc: /opt/ros/indigo/lib/librandom_numbers.so
pfc: /opt/ros/indigo/lib/libtransfer_function.so
pfc: /opt/ros/indigo/lib/libdiagnostic_aggregator.so
pfc: /opt/ros/indigo/lib/libecl_time_lite.so
pfc: /opt/ros/indigo/lib/libpluginlib_tutorials.so
pfc: /opt/ros/indigo/lib/libdwa_local_planner.so
pfc: /opt/ros/indigo/lib/libfreenect_sync.so
pfc: /opt/ros/indigo/lib/libpcl_ros_segmentation.so
pfc: /opt/ros/indigo/lib/libfreenect.so
pfc: /opt/ros/indigo/lib/libtf.so
pfc: /opt/ros/indigo/lib/libDepthImageToLaserScan.so
pfc: /opt/ros/indigo/lib/libparams.so
pfc: /opt/ros/indigo/lib/libxmlrpcpp.so
pfc: /opt/ros/indigo/lib/libecl_linear_algebra.so
pfc: /opt/ros/indigo/lib/libkobuki_auto_docking_nodelet.so
pfc: /opt/ros/indigo/lib/libsensor_range.so
pfc: /opt/ros/indigo/lib/liboctomath.so.1.6.9
pfc: /opt/ros/indigo/lib/libecl_threads.so
pfc: /opt/ros/indigo/lib/libpcl_ros_tf.so
pfc: /opt/ros/indigo/lib/libutils.so
pfc: /opt/ros/indigo/lib/libimage_rotate.so
pfc: /opt/ros/indigo/lib/libstereo_image_proc.so
pfc: /opt/ros/indigo/lib/libvision_reconfigure.so
pfc: /opt/ros/indigo/lib/libkdl_conversions.so
pfc: /opt/ros/indigo/lib/librosconsole_log4cxx.so
pfc: /opt/ros/indigo/lib/libgazebo_ros_utils.so
pfc: /opt/ros/indigo/lib/libgeometric_shapes.so
pfc: /opt/ros/indigo/lib/libkdl_parser.so
pfc: /opt/ros/indigo/lib/libecl_time.so
pfc: /opt/ros/indigo/lib/libstage.so.4.1.1
pfc: /opt/ros/indigo/lib/libDepthImageToLaserScanROS.so
pfc: /opt/ros/indigo/lib/libtf_conversions.so
pfc: /opt/ros/indigo/lib/libgazebo_ros_bumper.so
pfc: /opt/ros/indigo/lib/libMultiCameraPlugin.so
pfc: /opt/ros/indigo/lib/libgazebo_ros_paths_plugin.so
pfc: /opt/ros/indigo/lib/libamcl_map.so
pfc: /opt/ros/indigo/lib/libgazebo_ros_p3d.so
pfc: /opt/ros/indigo/lib/liborocos-kdl.so
pfc: /opt/ros/indigo/lib/libgazebo_ros_hand_of_god.so
pfc: /opt/ros/indigo/lib/libgazebo_ros_diff_drive.so
pfc: /opt/ros/indigo/lib/libkobuki.so
pfc: /opt/ros/indigo/lib/libscanmatcher.so
pfc: /opt/ros/indigo/lib/liboctomap.so
pfc: /opt/ros/indigo/lib/libroslz4.so
pfc: /opt/ros/indigo/lib/liburdf.so
pfc: /opt/ros/indigo/lib/libopenni2_camera_nodelet.so
pfc: /opt/ros/indigo/lib/librosconsole.so
pfc: /opt/ros/indigo/lib/libecl_exceptions.so
pfc: /opt/ros/indigo/lib/libgazebo_ros_template.so
pfc: /opt/ros/indigo/lib/libgazebo_ros_ft_sensor.so
pfc: /opt/ros/indigo/lib/libpolled_camera.so
pfc: /opt/ros/indigo/lib/libcostmap_2d.so
pfc: /opt/ros/indigo/lib/libroslib.so
pfc: /opt/ros/indigo/lib/libkobuki_bumper2pc_nodelet.so
pfc: /opt/ros/indigo/lib/libpcl_ros_filters.so
pfc: /opt/ros/indigo/lib/libdefault_plugin.so
pfc: /opt/ros/indigo/lib/libpointcloud_filters.so
pfc: /opt/ros/indigo/lib/libecl_streams.so
pfc: /opt/ros/indigo/lib/libecl_errors.so
pfc: /opt/ros/indigo/lib/libsensor_odometry.so
pfc: /opt/ros/indigo/lib/libpointcloud_to_laserscan.so
pfc: /opt/ros/indigo/lib/libkobuki_safety_controller_nodelet.so
pfc: /opt/ros/indigo/lib/libmedian.so
pfc: /opt/ros/indigo/lib/libnodeletlib.so
pfc: /opt/ros/indigo/lib/libwarehouse_ros.so
pfc: /opt/ros/indigo/lib/libactionlib.so
pfc: /opt/ros/indigo/lib/libfreenect.so.0.5
pfc: /opt/ros/indigo/lib/libgazebo_ros_multicamera.so
pfc: /opt/ros/indigo/lib/libyocs_velocity_smoother_nodelet.so
pfc: /opt/ros/indigo/lib/libDepthImageToLaserScanNodelet.so
pfc: /opt/ros/indigo/lib/libstage.so
pfc: /opt/ros/indigo/lib/librviz.so
pfc: /opt/ros/indigo/lib/librqt_image_view.so
pfc: /opt/ros/indigo/lib/libgazebo_ros_projector.so
pfc: /opt/ros/indigo/lib/libgridfastslam.so
pfc: /opt/ros/indigo/lib/liboctomath.so
pfc: /opt/ros/indigo/lib/libclass_loader.so
pfc: /opt/ros/indigo/lib/libpano_core.so
pfc: /opt/ros/indigo/lib/liboctomath.so.1.6
pfc: /opt/ros/indigo/lib/libecl_devices.so
pfc: /opt/ros/indigo/lib/libgazebo_ros_force.so
pfc: /opt/ros/indigo/lib/libpcl_ros_features.so
pfc: /opt/ros/indigo/lib/libkobuki_random_walker_nodelet.so
pfc: /opt/ros/indigo/lib/libgazebo_ros_range.so
pfc: /opt/ros/indigo/lib/libimage_geometry.so
pfc: /opt/ros/indigo/lib/libkobuki_auto_docking_ros.so
pfc: /opt/ros/indigo/lib/libgazebo_ros_joint_pose_trajectory.so
pfc: /opt/ros/indigo/lib/libopencv_apps.so
pfc: /opt/ros/indigo/lib/libcamera_info_manager.so
pfc: /opt/ros/indigo/lib/libkobuki_dock_drive.so
pfc: /opt/ros/indigo/lib/librosbag.so
pfc: /opt/ros/indigo/lib/liboctomap.so.1.6.9
pfc: /opt/ros/indigo/lib/liblaser_scan_filters.so
pfc: /opt/ros/indigo/lib/libtrajectory_planner_ros.so
pfc: /opt/ros/indigo/lib/libtopic_tools.so
pfc: /opt/ros/indigo/lib/liblaser_geometry.so
pfc: /opt/ros/indigo/lib/libdynamic_reconfigure_config_init_mutex.so
pfc: /opt/ros/indigo/lib/libtheora_image_transport.so
pfc: /opt/ros/indigo/lib/libpcl_ros_io.so
pfc: /opt/ros/indigo/lib/libfreenect.so.0.5.1
pfc: /opt/ros/indigo/lib/libgazebo_ros_tricycle_drive.so
pfc: /opt/ros/indigo/lib/libgazebo_ros_camera_utils.so
pfc: /opt/ros/indigo/lib/libcollada_parser.so
pfc: /opt/ros/indigo/lib/liblayers.so
pfc: /opt/ros/indigo/lib/libyocs_cmd_vel_mux_nodelet.so
pfc: /opt/ros/indigo/lib/liborocos-kdl.so.1.3.0
pfc: /opt/ros/indigo/lib/librqt_rviz.so
pfc: /opt/ros/indigo/lib/libgazebo_ros_api_plugin.so
pfc: /opt/ros/indigo/lib/libroscpp_serialization.so
pfc: /opt/ros/indigo/lib/libresource_retriever.so
pfc: /opt/ros/indigo/lib/libimage_proc.so
pfc: /opt/ros/indigo/lib/libkobuki_ros.so
pfc: /opt/ros/indigo/lib/libgazebo_ros_skid_steer_drive.so
pfc: /opt/ros/indigo/lib/libamcl_sensors.so
pfc: /opt/ros/indigo/lib/libgazebo_ros_openni_kinect.so
pfc: /opt/ros/indigo/lib/libgazebo_ros_gpu_laser.so
pfc: /opt/ros/indigo/lib/libtf2_ros.so
pfc: /opt/ros/indigo/lib/libbondcpp.so
pfc: /opt/ros/indigo/lib/liborocos-kdl.so.1.3
pfc: /opt/ros/indigo/lib/libgazebo_ros_depth_camera.so
pfc: /opt/ros/indigo/lib/librostime.so
pfc: /opt/ros/indigo/lib/libcamera_calibration_parsers.so
pfc: /opt/ros/indigo/lib/libcompressed_image_transport.so
pfc: /opt/ros/indigo/lib/libeigen_conversions.so
pfc: /opt/ros/indigo/lib/libgazebo_ros_block_laser.so
pfc: /opt/ros/indigo/lib/libamcl_pf.so
pfc: /opt/ros/indigo/lib/libgazebo_ros_imu.so
pfc: /opt/ros/indigo/lib/libturtlebot_follower.so
pfc: /opt/ros/indigo/lib/librosbag_storage.so
pfc: /opt/ros/indigo/lib/libecl_formatters.so
pfc: /opt/ros/indigo/lib/libgazebo_ros_joint_trajectory.so
pfc: /opt/ros/indigo/lib/libmove_base.so
pfc: /opt/ros/indigo/lib/libimage_transport.so
pfc: /opt/ros/indigo/lib/libopenni2_driver_lib.so
pfc: /opt/ros/indigo/lib/libcpp_common.so
pfc: /opt/ros/indigo/lib/libkobuki_nodelet.so
pfc: /opt/ros/indigo/lib/libgazebo_ros_planar_move.so
pfc: /opt/ros/indigo/lib/libqt_gui_cpp.so
pfc: /opt/ros/indigo/lib/libvoxel_grid.so
pfc: /opt/ros/indigo/lib/libgazebo_ros_prosilica.so
pfc: /opt/ros/indigo/lib/libimage_view.so
pfc: /opt/ros/indigo/lib/libgazebo_ros_f3d.so
pfc: /opt/ros/indigo/lib/libgazebo_ros_laser.so
pfc: /opt/ros/indigo/lib/libfreenect_sync.so.0.5
pfc: /opt/ros/indigo/lib/librosconsole_bridge.so
pfc: /opt/ros/indigo/lib/librotate_recovery.so
pfc: /opt/ros/indigo/lib/libecl_geometry.so
pfc: /opt/ros/indigo/lib/libincrement.so
pfc: /opt/ros/indigo/lib/libopenni2_wrapper.so
pfc: /opt/ros/indigo/lib/libimage_transport_plugins.so
pfc: /opt/ros/indigo/lib/libmean.so
pfc: /opt/ros/indigo/lib/libtf2.so
pfc: /opt/ros/indigo/lib/libzeroconf_avahi.so
pfc: /opt/ros/indigo/lib/liborocos-bfl.so
pfc: /opt/ros/indigo/lib/librobot_state_publisher_solver.so
pfc: CMakeFiles/pfc.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable pfc"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pfc.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/pfc.dir/build: pfc
.PHONY : CMakeFiles/pfc.dir/build

CMakeFiles/pfc.dir/requires: CMakeFiles/pfc.dir/pfc.cpp.o.requires
CMakeFiles/pfc.dir/requires: CMakeFiles/pfc.dir/point.cpp.o.requires
.PHONY : CMakeFiles/pfc.dir/requires

CMakeFiles/pfc.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/pfc.dir/cmake_clean.cmake
.PHONY : CMakeFiles/pfc.dir/clean

CMakeFiles/pfc.dir/depend:
	cd /home/matthew/CSC232/Lab5/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/matthew/CSC232/Lab5 /home/matthew/CSC232/Lab5 /home/matthew/CSC232/Lab5/build /home/matthew/CSC232/Lab5/build /home/matthew/CSC232/Lab5/build/CMakeFiles/pfc.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/pfc.dir/depend

