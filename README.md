ros_datasets_playback
=============

Configurations, launch files and utilities to replay ROS and ROS2 datasets.

**N.B.** This is a **ROS2** package which leverages the [ros1_bridge](https://github.com/ros2/ros1_bridge) to replay ROS bags.

## ROS bags replay
To enable replay of ROS bags in ROS2 environment, some utils are available under the `scripts/` directory.
Before running these, follow the instruction to install the [ros1_bridge](https://github.com/ros2/ros1_bridge). Then, some environment variables need to be exported:

`export ROS1_INSTALL_PATH=/path/to/ros1/install`  
`export ROS2_INSTALL_PATH=/path/to/ros2/install`   
`export ROS2_WS_INSTALL_PATH=/path/to/ros2_ws/install`
