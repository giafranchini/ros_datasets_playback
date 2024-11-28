#!/bin/bash
set -e

if [ -z "$1" ]
  then
    echo "No bag path supplied!"
    exit 0
fi

if [ "$1" == "-h" ]; then
  echo "Usage: `basename $0` [-h] [-p path] -- replay a rosbag file

where:
    -h  show this help text
    -p  the bag file path"
  exit 0
fi

bag_file="$1"

source ${ROS1_INSTALL_PATH}/setup.bash

rosbag play -l --clock $bag_file
