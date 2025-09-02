#!/bin/bash
source /opt/ros/${ROS_DISTRO}/setup.bash
ros2 run camera_calibration cameracalibrator --size 11x8 --square 0.03 --ros-args -r image:=/image_raw -p camera:=/daheng_camera