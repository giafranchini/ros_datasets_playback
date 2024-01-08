
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
    config_file = 'roxy_fixed_lag_smoother.yaml'
    smoother_params = os.path.join(
        this_package_dir, 'config', config_file)
    bag_relative_path = "custom_rosbag2__1/custom_rosbag2__1_0.db3"
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='DEBUG'
    )

    container_fuse = ComposableNodeContainer(
        name='container_fuse',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        # emulate_tty=True,
        # prefix=["gdbserver localhost:3000"],
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

    imu_rotation = Node(
        package="imu_transformer",
        executable="imu_transformer_node",
        output="screen",
        remappings=[
            ("imu_in", "/localisation/imu/data"),
            ("imu_out", "/localisation/imu/data_rotated"), 
        ],
        parameters=[
            "target_frame := base_link",
        ],
    )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        DeclareLaunchArgument("visualize", default_value="true"),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play',
                 PathJoinSubstitution([this_package_dir, 'bags', bag_relative_path]),
                 '--clock', '--loop'],
            output='screen',
        ),
        rviz2,
        declare_log_level,
        # imu_rotation,
        # container_fuse,
    ])
