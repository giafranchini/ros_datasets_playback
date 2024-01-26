
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    bag_relative_path = 'custom_rosbag2__1_double_round/custom_rosbag2__1_double_round_0.db3'
    this_package_dir = get_package_share_directory('etna_s3li_playback')
    smoother_config = 'fixed_lag_smoother_roxy.yaml'
    smoother_params = os.path.join(this_package_dir, 'config', smoother_config)
    
    fuse = Node(
        package='fuse_optimizers',
        executable='fixed_lag_smoother_node',
        name='fixed_lag_smoother_node',
        # prefix=[f'xterm -e valgrind --tool=callgrind'],
        output='screen',
        parameters=[smoother_params],
    )

    imu_rotation = Node(
        package='imu_transformer',
        executable='imu_transformer_node',
        name='imu_transformer_node',
        output='screen',
        arguments=[
            '--ros-args', '-p', 'target_frame:=base_link'],
        remappings=[
            ('imu_in', '/localisation/imu/data'),
            ('imu_out', '/localisation/imu/data_rotated')],
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output={'both': 'log'},
        arguments=[
            '-d', PathJoinSubstitution([this_package_dir, 'rviz', 'roxy_rviz_config.rviz'])
        ],
        condition=IfCondition(LaunchConfiguration('visualize')),
    )

    rosbag2_playback =  ExecuteProcess(
        cmd=['ros2', 'bag', 'play',
            PathJoinSubstitution([this_package_dir, 'bags', bag_relative_path]),
            '--clock', '--loop'],
        output='screen',
    )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        DeclareLaunchArgument('visualize', default_value='true'),
        rosbag2_playback,
        rviz2,
        imu_rotation,
        fuse,
    ])
