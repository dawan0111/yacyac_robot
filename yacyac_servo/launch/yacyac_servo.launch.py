# my_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # servo_ctrl 노드를 실행하는 런치 액션
    servo_ctrl_node = Node(
            package='yacyac_servo',
            executable='servo_ctrl',
            name='servo_ctrl_node',
            output='screen',
        ),
    # read_write_node 노드를 실행하는 런치 액션
    read_write_node = Node(
        package="dynamixel_sdk_examples",
        executable="read_write_node",
        output="screen",
        # parameters=[{"use_sim_time": False}],  # 필요한 경우 추가적인 파라미터 설정
    )

    return LaunchDescription([servo_ctrl_node, read_write_node])

