import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_desc = get_package_share_directory('real_description')
    urdf_path = os.path.join(pkg_desc, 'urdf', 'Wheeled_Base.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        # Replaces joint_state_publisher_gui
        # Drives wheels from /cmd_vel for visualization
        Node(
            package='real_bringup',
            executable='diff_drive_sim_node',
            name='diff_drive_sim',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
    ])