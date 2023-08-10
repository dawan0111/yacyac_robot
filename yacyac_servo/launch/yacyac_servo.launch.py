# my_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    servo_ctrl_node = Node(
        package='yacyac_servo',
        executable='servo_ctrl',
        name='servo_ctrl_node',
        output='screen',
    )
    read_write_node = Node(
        package="dynamixel_sdk_examples",
        executable="read_write_node",
        output="screen",
    )
    
    return LaunchDescription([servo_ctrl_node, read_write_node])

