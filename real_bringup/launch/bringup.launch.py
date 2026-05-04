import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_desc    = get_package_share_directory('real_description')
    pkg_bringup = get_package_share_directory('real_bringup')

    # Read URDF directly
    urdf_path = os.path.join(pkg_desc, 'urdf', 'Wheeled_Base.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    ekf_config = os.path.join(pkg_bringup, 'config', 'ekf.yaml')

    # ── Robot State Publisher ──────────────────────────────────────
    # Reads URDF, publishes /robot_description, builds TF tree
    # from /joint_states (provided by micro-ROS ESP32)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': False},
        ],
    )

    # ── RPLidar A1 driver ─────────────────────────────────────────
    # Publishes /scan — physically connected via USB to Pi 5
    # serial_port is /dev/ttyUSB0 by default (may need udev rule on Pi)
    rplidar = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar',
        output='screen',
        parameters=[{
            'serial_port':      '/dev/ttyUSB0',
            'serial_baudrate':  115200,
            'frame_id':         'lidar_link',
            'angle_compensate': True,
            'scan_mode':        'Standard',
        }],
    )

    # ── micro-ROS agent ───────────────────────────────────────────
    # Bridges ESP32 ↔ ROS2 graph over USB serial
    # ESP32 publishes: /odom, /imu/data, /joint_states
    # ESP32 subscribes: /cmd_vel
    # serial_port: ESP32 is /dev/ttyUSB1 when RPLidar is on /dev/ttyUSB0
    # (use udev rules to make these names fixed — see README)
# ── micro-ROS agent ───────────────────────────────────────────
    # Uncomment when running on Pi 5 with ESP32 connected.
    # Not available via apt — must be built from source on Pi 5.
    # micro_ros_agent = Node(
    #     package='micro_ros_agent',
    #     executable='micro_ros_agent',
    #     name='micro_ros_agent',
    #     output='screen',
    #     arguments=['serial', '--dev', '/dev/esp32', '-v4'],
    # )

    # ── EKF node ─────────────────────────────────────────────────
    # Fuses /odom (encoders) + /imu/data (gyro)
    # Publishes /odometry/filtered → used by Nav2 instead of raw /odom
    # Delayed 3s to let micro-ROS agent connect and ESP32 start publishing
    ekf_node = TimerAction(
        period=3.0,
        actions=[Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': False}],
        )],
    )

    return LaunchDescription([
        robot_state_publisher,
        rplidar,
       # micro_ros_agent,
        ekf_node,
    ])