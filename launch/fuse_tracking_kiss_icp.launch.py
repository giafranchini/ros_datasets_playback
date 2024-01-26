from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import ComposableNodeContainer, Node, SetParameter
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    kiss_icp_pkg = FindPackageShare("kiss_icp")
    this_pkg_name = "etna_s3li_playback"
    this_package_dir = get_package_share_directory(this_pkg_name)
    smoother_params = os.path.join(
        this_package_dir, 'config', 'roxy_fixed_lag_smoother.yaml')
    pcl_params = os.path.join(this_package_dir, 'config', 'pcl_transform.yaml')
    d2c_params = os.path.join(this_package_dir, 'config', 'depth_to_cloud.yaml')
    kiss_icp_params = os.path.join(this_package_dir, 'config', 'kiss_icp.yaml')

    log_level = LaunchConfiguration('log_level')
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='INFO'
    )

    container_kiss_icp = ComposableNodeContainer(
        name='container_kiss_icp',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        # prefix=["gdbserver localhost:3000"],
        emulate_tty=True,
        composable_node_descriptions=[
            ComposableNode(
                package='depth_to_cloud',
                plugin='traversability::DepthToCloud',
                name='depth_to_cloud',
                extra_arguments=[{'use_intra_process_comms': True}],
                parameters=[d2c_params],
            ),
            ComposableNode(
                package='pointcloud_processing',
                plugin='pointcloud_processing::PointCloudTransform',
                name='pointcloud_transform',
                extra_arguments=[{'use_intra_process_comms': True}],
                parameters=[PathJoinSubstitution([
                    this_package_dir, 'config', 'pcl_transform.yaml'
                ])],
                remappings=[
                    # ('pointcloud_in', '/bf_lidar/points_raw'),
                    # ('pointcloud_in', '/stereo/points'),
                    ('pointcloud_in', 'point_cloud_from_depth'),
                    # ('pointcloud_in', '/camera_color/points'),
                    ('pointcloud_out', 'pointcloud_transformed'),
                ],
            ),
            ComposableNode(
                package='kiss_icp',
                plugin='kiss_icp_ros::OdometryServer',
                name='odometry_node',
                remappings=[("pointcloud_topic", 'pointcloud_transformed'),
                            ('odometry', 'kiss_icp/odometry')],
                parameters=[kiss_icp_params],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package="logging_demo",
                plugin='logging_demo::LoggerConfig',
                name='logger_config',
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        ],
        output='both',
    )

    container_fuse = ComposableNodeContainer(
        name='container_fuse',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        # prefix=["gdbserver localhost:3000"],
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
            )
        ],
        output='both',
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        output={"both": "log"},
        arguments=[
            "-d", PathJoinSubstitution([kiss_icp_pkg, "rviz", "kiss_icp_ros2.rviz"])],
        condition=IfCondition(LaunchConfiguration("visualize")),
    )

    return LaunchDescription([
        DeclareLaunchArgument("visualize", default_value="true"),
        SetParameter(name='use_sim_time', value=True),
        rviz2,
        declare_log_level,
        container_kiss_icp,
        container_fuse
    ])
