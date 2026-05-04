import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_bringup = get_package_share_directory('real_bringup')
    slam_params = os.path.join(pkg_bringup, 'config', 'slam_params.yaml')

    # Delayed 5s — waits for bringup.launch.py to bring up
    # micro-ROS agent, EKF, and /scan before SLAM starts.
    # SLAM needs: /scan + odom→CHASSIS TF to be available.
    slam_toolbox = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('slam_toolbox'),
                        'launch',
                        'online_async_launch.py',
                    ])
                ]),
                launch_arguments={
                    'slam_params_file': slam_params,
                    'use_sim_time':     'false',
                }.items(),
            )
        ],
    )

    return LaunchDescription([
        slam_toolbox,
    ])