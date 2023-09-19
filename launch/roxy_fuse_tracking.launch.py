
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess

from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import ComposableNodeContainer, Node, SetParameter
from launch_ros.descriptions import ComposableNode

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    this_pkg_name = "etna_s3li_playback"
    this_package_dir = get_package_share_directory(this_pkg_name)
    smoother_params = os.path.join(
        this_package_dir, 'config', 'roxy_fixed_lag_smoother.yaml')
    bag_relative_path = "2023-07-03_17-07-44____20_minutes_test_1/2023-07-03_17-07-44__20_minutes_test/2023-07-03_17-07-44_0.db3"

    log_level = LaunchConfiguration('log_level')
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='INFO'
    )

    container_fuse = ComposableNodeContainer(
        name='container_fuse',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        emulate_tty=True,
        composable_node_descriptions=[
            ComposableNode(
                package='fuse_optimizers',
                plugin='fuse_optimizers::FixedLagSmootherComponent',
                name='fixed_lag_smoother_node',
                parameters=[smoother_params],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package="logging_demo",
                plugin='logging_demo::LoggerConfig',
                name='logger_config_fuse',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='both',
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        output={"both": "log"},
        arguments=[
            "-d", PathJoinSubstitution([this_package_dir, "rviz", "roxy_rviz_config.rviz"])],
        condition=IfCondition(LaunchConfiguration("visualize")),
    )

    rl_odometry_recorder = Node(
        package="odometry_recorder",
        executable="odometry_recorder_node",
        name="rl_odometry_recorder",
        parameters=[
            {"topic_name": "/localisation/odometry/filtered"},
            {"output_file": "rl_odometry2d.csv"}
        ],
    )

    fuse_odometry_recorder = Node(
        package="odometry_recorder",
        executable="odometry_recorder_node",
        name="fuse_odometry_recorder",
        parameters=[
            {"topic_name": "/fuse/odometry/filtered"},
            {"output_file": "fuse_odometry2d.csv"}
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("visualize", default_value="true"),
        SetParameter(name='use_sim_time', value=True),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play',
                 PathJoinSubstitution([this_package_dir, 'bags', bag_relative_path]),
                 '--clock', '-l'],
            output='screen'
        ),
        rviz2,
        declare_log_level,
        container_fuse,
        # rl_odometry_recorder,
        # fuse_odometry_recorder
    ])
