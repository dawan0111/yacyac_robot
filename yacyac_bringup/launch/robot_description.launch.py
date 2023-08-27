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
    # g2lidar package
    g2lidar_prefix = get_package_share_directory("ydlidar_ros2_driver")
    start_g2lidar_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(g2lidar_prefix, "launch", "ydlidar_launch.py"))
    )

    yacyac_prefix = get_package_share_directory("md")
    start_yacyac_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(yacyac_prefix, "launch", "md.launch.py"))
    )

    yacyac_camera_prefix = get_package_share_directory("yacyac_camera")
    start_yacyac_camera_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(yacyac_camera_prefix, "launch", "camera.launch.py"))
    )

    yacyac_servo_prefix = get_package_share_directory("yacyac_servo")
    start_yacyac_servo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(yacyac_servo_prefix, "launch", "yacyac_servo.launch.py"))
    )



    # # rosbridge websocket cmd
    start_web_bridge_cmd = ExecuteProcess(
        cmd=["ros2", "launch", "rosbridge_server", "rosbridge_websocket_launch.xml"], output="screen"
    )

    base_to_laser_publisher = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "tf2_ros",
            "static_transform_publisher",
            "0.2",
            "0",
            "0.18",
            "0",
            "0",
            "0",
            "base_link",
            "laser_frame",
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


    return LaunchDescription(
        [
            base_to_laser_publisher,
            base_to_imu_publisher,
            start_g2lidar_cmd,
            start_yacyac_cmd,
            start_web_bridge_cmd,
            start_yacyac_camera_cmd,
            start_yacyac_servo_cmd,
            
            Node(
                name="yacyac_io_node",
                package="yacyac_io",
                executable="tts",
                output="screen",
                emulate_tty=True,
            )
        ]
    )
