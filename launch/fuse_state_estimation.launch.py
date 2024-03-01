
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node, SetParameter
from launch_ros.descriptions import ComposableNode

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    bag_relative_path = 'custom_rosbag2__1_double_round/custom_rosbag2__1_double_round_0.db3'
    this_package_dir = get_package_share_directory('etna_s3li_playback')
    smoother_config = 'fixed_lag_smoother_roxy.yaml'
    ekf_config = 'ekf_roxy.yaml'
    smoother_params = os.path.join(this_package_dir, 'config', smoother_config)
    ekf_params = os.path.join(this_package_dir, 'config', ekf_config)
    playback = True
    
    container_fuse = ComposableNodeContainer(
        name='container_fuse',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='fuse_optimizers',
                plugin='fuse_optimizers::FixedLagSmootherComponent',
                name='fixed_lag_smoother_node',
                parameters=[smoother_params],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='both',
    )

    robot_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params],
        condition=IfCondition(LaunchConfiguration('robot_localization')),
    )

    imu_transformer = Node(
        package='imu_transformer',
        executable='imu_transformer_node',
        name='imu_transformer',
        namespace='localisation/imu',
        parameters=[{'target_frame': 'base_link'}],
        remappings=[
            ('imu_in', '/localisation/imu/data'),
            ('imu_out', '/localisation/imu/data_rotated')],
        output='screen',
    )

    fuse_odom_recorder = Node(
        package='odometry_recorder',
        executable='odometry_recorder_node',
        output={'both': 'log'},
        parameters=[
            {'topic_name': 'fuse/odometry/filtered'},
            {'output_file': 'fuse_odom.csv'},
        ],
        condition=IfCondition(LaunchConfiguration('odom_recording'))
    )

    rl_odom_recorder = Node(
        package='odometry_recorder',
        executable='odometry_recorder_node',
        output={'both': 'log'},
        parameters=[
            {'topic_name': 'odometry/filtered'},
            {'output_file': 'rl_odom.csv'},
        ],
        condition=IfCondition(LaunchConfiguration('odom_recording'))
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
            '--clock'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('playback')),
    )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=playback),
        DeclareLaunchArgument('visualize', default_value='true'),
        DeclareLaunchArgument('playback', default_value=str(playback)),
        DeclareLaunchArgument('odom_recording', default_value='false'),
        DeclareLaunchArgument('robot_localization', default_value='false'),
        rosbag2_playback,
        rviz2,
        container_fuse,
        fuse_odom_recorder,
        rl_odom_recorder,
        imu_transformer,
        robot_localization,
    ])
