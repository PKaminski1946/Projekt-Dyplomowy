#!/bin/sh

source ~/ros2_ws/install/setup.bash

ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 &
ros2 run ros_gz_image image_bridge /camera /camera &


cd ~/PX4-Autopilot
make PX4_GZ_WORLD="tracking_circle" px4_sitl gz_x500_mono_cam
param set NAV_RCL_ACT 0
param set NAV_DLL_ACT 0
commander mode manual 
