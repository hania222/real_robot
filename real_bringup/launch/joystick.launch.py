from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # Runs websocket server + publishes Twist on /cmd_vel
        Node(
            package='real_bringup',
            executable='joystick_ws_node',
            name='joystick_ws_node',
            output='screen',
        ),

    ])