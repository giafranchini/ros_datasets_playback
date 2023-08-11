#!/bin/bash
set -e

if [ -f /opt/ros/humble/setup.bash ]; then source /opt/ros/humble/setup.bash; fi
if [ -f /home/user/humble_ws/install/local_setup.bash ]; then source /home/user/humble_ws/install/local_setup.bash; fi

export ROS_DOMAIN_ID=88 

ros2 topic echo /clock rosgraph_msgs/Clock &
sleep 3
ros2 launch etna_s3li_playback etna_s3li_utils.launch.py