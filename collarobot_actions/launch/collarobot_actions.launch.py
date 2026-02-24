from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='collarobot_actions',
            executable='pick_place_node',
            name='pick_place_node',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='collarobot_actions',
            executable='gesture_node',
            name='gesture_node',
            output='screen',
            emulate_tty=True,
        ),
    ])
