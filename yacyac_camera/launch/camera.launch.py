import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_param_dir = LaunchConfiguration(
        "camera_param_dir",
        default=os.path.join(
            get_package_share_directory("yacyac_camera"), "param", "md_param.yaml"
        ),
    )
    DeclareLaunchArgument(
        "camera_param_dir",
        default_value=camera_param_dir,
        description="Full path of camera parameter file",
    ),

    return LaunchDescription(
        [
            Node(
                package="yacyac_camera",
                executable="camera_qr_pipeline",
                output="screen",
                emulate_tty=True,
                parameters=[camera_param_dir],
            )
        ]
    )
