
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
    smoother_config = 'roxy_fixed_lag_smoother.yaml'
    ekf_config = 'ekf_roxy.yaml'
    smoother_params = os.path.join(this_package_dir, 'config', smoother_config)
    ekf_params = os.path.join(get_package_share_directory('robot_localization'), 'params', ekf_config)
    
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='DEBUG'
    )

    container_fuse = ComposableNodeContainer(
        name='container_fuse',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        # prefix=['gdbserver localhost:3000'],
        composable_node_descriptions=[
            ComposableNode(
                package='fuse_optimizers',
                plugin='fuse_optimizers::FixedLagSmootherComponent',
                name='fixed_lag_smoother_node',
                parameters=[smoother_params],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='logging_demo',
                plugin='logging_demo::LoggerConfig',
                name='logger_config_fuse',
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

    fuse_odom_recorder = Node(
        package='odometry_recorder',
        executable='odometry_recorder_node',
        output={'both': 'log'},
        parameters=[
            {'topic_name': 'fuse/odometry/filtered'},
            {'output_file': 'fuse_odom.csv'},
        ],
        arguments=[
            '--ros-args', '--log-level', LaunchConfiguration('log_level')
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
        arguments=[
            '--ros-args', '--log-level', LaunchConfiguration('log_level')
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
            '--clock', '--loop'],
        output='screen',
    )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        DeclareLaunchArgument('visualize', default_value='true'),
        DeclareLaunchArgument('odom_recording', default_value='false'),
        rosbag2_playback,
        rviz2,
        declare_log_level,
        imu_rotation,
        container_fuse,
        # robot_localization,
        fuse_odom_recorder,
        rl_odom_recorder,
    ])
