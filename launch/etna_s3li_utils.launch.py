#! /usr/bin/env python3

# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import SetParameter, Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    this_package_dir = get_package_share_directory("etna_s3li_playback")
    imu_utils_package_dir = get_package_share_directory("imu_utils")

    urdf_file_name = 'sensors_setup.urdf'
    urdf = os.path.join(this_package_dir, "urdf", urdf_file_name)
    with open(urdf, 'r') as file:
        robot_description_content = file.read()

    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      parameters=[{
                                          'robot_description': robot_description_content,
                                      }])

    imu_processor = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                imu_utils_package_dir, '/launch', '/imu_processor.launch.py'
            ]),
            launch_arguments=[("topic_in", "/imu/data")]
    )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        robot_state_publisher_node,
        imu_processor,
    ])
