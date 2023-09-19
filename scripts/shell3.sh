#!/bin/bash
set -e

if [ -f /home/user/noetic_source/devel_isolated/setup.bash ]; then source /home/user/noetic_source/devel_isolated/setup.bash; fi

rosbag play -l /home/user/humble_ws/src/etna_s3li_playback/bags/s3li_traverse_1.bag --clock
