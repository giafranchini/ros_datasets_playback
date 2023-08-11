#!/bin/bash
set -e

if [ -f /home/user/noetic_source/devel_isolated/setup.bash ]; then source /home/user/noetic_source/devel_isolated/setup.bash; fi
if [ -f /opt/ros/humble/setup.bash ]; then source /opt/ros/humble/setup.bash; fi
if [ -f /home/user/humble_ws/install/local_setup.bash ]; then source /home/user/humble_ws/install/local_setup.bash; fi

export ROS_MASTER_URI=http://d6fc67979143:11311/
export ROS_DOMAIN_ID=88

ros2 run ros1_bridge dynamic_bridge