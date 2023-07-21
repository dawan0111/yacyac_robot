import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    # s2lidar package
    g2lidar_prefix = get_package_share_directory("ydlidar_ros2_driver")
    start_g2lidar_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(g2lidar_prefix, "launch", "ydlidar_launch.py"))
    )

    # tracer mini package
    yacyac_prefix = get_package_share_directory("md")
    start_yacyac_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(yacyac_prefix, "launch", "md.launch.py"))
    )

    # start_insta360_cmd = ExecuteProcess(
    #     cmd=["ros2", "run", "insta360_node", "insta360_node"], output="screen"
    # )

    start_um7_cmd = ExecuteProcess(cmd=["ros2", "run", "um7", "um7_node"], output="screen")
    base_to_laser_publisher = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "tf2_ros",
            "static_transform_publisher",
            "0",
            "0",
            "0.1",
            "-3.141592",
            "0",
            "0",
            "base_link",
            "laser",
        ],
        output="screen",
    )
    map_to_odom_publisher = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "tf2_ros",
            "static_transform_publisher",
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "map",
            "odom",
        ],
        output="screen",
    )
    odom_to_base_publisher = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "tf2_ros",
            "static_transform_publisher",
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "odom",
            "base_link",
        ],
        output="screen",
    )
    base_to_imu_publisher = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "tf2_ros",
            "static_transform_publisher",
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "base_link",
            "imu_link",
        ],
        output="screen",
    )

    # start_web_bridge_cmd = ExecuteProcess(
    #     cmd=["ros2", "launch", "rosbridge_server", "rosbridge_websocket_launch.xml"],
    #     output="screen",
    # )

    return LaunchDescription(
        [
            # map_to_odom_publisher,
            base_to_laser_publisher,
            # odom_to_base_publisher,
            base_to_imu_publisher,
            start_g2lidar_cmd,
            start_yacyac_cmd,
            # start_insta360_cmd,
            # start_um7_cmd,
            # start_web_bridge_cmd,
        ]
    )