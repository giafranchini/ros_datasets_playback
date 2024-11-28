#!/bin/bash
set -e

source ${ROS1_INSTALL_PATH}/setup.bash
source ${ROS2_INSTALL_PATH}/setup.bash
source ${ROS2_WS_INSTALL_PATH}/local_setup.bash

ros2 run ros1_bridge dynamic_bridge