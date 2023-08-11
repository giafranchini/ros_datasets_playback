# Copyright 2020 Hironori Fujimoto
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    sgm_pkg_name = "sgm_gpu"
    package_dir = get_package_share_directory(sgm_pkg_name)

    container_sgm = ComposableNodeContainer(
        name='sgm_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package=sgm_pkg_name,
                plugin='sgm_gpu::SgmGpuNode',
                name='sgm_gpu_node',
                remappings=[
                    ('left_image', '/stereo/left/image_rect'),
                    ('right_image', '/stereo/right/image_rect'),
                    ('left_camera_info', '/stereo/left/camera_info'),
                    ('right_camera_info', '/stereo/right/camera_info')
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='stereo_image_proc',
                plugin='stereo_image_proc::PointCloudNode',
                name='pointcloud_node',
                remappings=[
                    ('left/image_rect_color', '/stereo/left/image_rect'),
                    ('left/camera_info', '/stereo/left/camera_info'),
                    ('right/camera_info', '/stereo/right/camera_info'),
                    ('disparity', '/disparity'),
                    ('points2', '/stereo/points')
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='image_view',
                plugin='image_view::ImageViewNode',
                name='depth_view',
                remappings=[('image', '/depth')],
                parameters=[{'use_sim_time': True}, {'window_name': 'Depth'}],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='image_view',
                plugin='image_view::StereoViewNode',
                name='stereo_view',
                remappings=[('stereo/left/image', '/stereo/left/image_rect'),
                            ('stereo/right/image', '/stereo/right/image_rect'),
                            ('stereo/disparity', '/disparity'), ],
                parameters=[{'window_name': 'Stereo'}],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            # ComposableNode(
            #     package='image_view',
            #     plugin='image_view::ImageViewNode',
            #     name='right_view',
            #     remappings=[
            #         ('image', '/test_stereo_publisher/right/image_raw')],
            #     parameters=[{'window_name': 'Right image'}],
            #     extra_arguments=[{'use_intra_process_comms': True}],
            # ),
            # ComposableNode(
            #     package='image_view',
            #     plugin='image_view::ImageViewNode',
            #     name='left_view',
            #     remappings=[
            #         ('image', '/test_stereo_publisher/left/image_raw')],
            #     parameters=[{'window_name': 'Left image'}],
            #     extra_arguments=[{'use_intra_process_comms': True}],
            # ),
        ],
        output='screen',
    )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        # Node(
        #     package='sgm_gpu',
        #     executable='test_stereo_publisher',
        #     name='test_stereo_publisher'
        # ),
        container_sgm,
    ])
