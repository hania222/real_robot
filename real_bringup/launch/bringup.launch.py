import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_package_share(package_name: str) -> str:
    return get_package_share_directory(package_name)


def generate_launch_description():
    bringup_share     = get_package_share('real_bringup')
    description_share = get_package_share('real_description')

    urdf_path  = os.path.join(description_share, 'urdf', 'Robot_Final.urdf')
    ekf_config = os.path.join(bringup_share, 'config', 'ekf.yaml')

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/esp32',
        description='Serial port for the ESP32 hardware bridge',
    )
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Baud rate for ESP32 serial',
    )

    serial_port = LaunchConfiguration('serial_port')
    baud_rate   = LaunchConfiguration('baud_rate')

    with open(urdf_path, 'r') as urdf_file:
        robot_description_xml = urdf_file.read()

    # Publishes /robot_description and broadcasts the TF tree from /joint_states
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description_xml},
            {'use_sim_time': False},
        ],
    )

    # Reads encoder/IMU data from ESP32 over serial
    # Publishes /odom, /imu/data, /joint_states, /pid_debug
    # and broadcasts the odom -> CHASSIS TF
    hardware_bridge = Node(
        package='real_controllers',
        executable='hardware_bridge_node',
        name='hardware_bridge_node',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'baud_rate':   baud_rate,
        }],
    )

    # Fuses /odom + /imu/data → publishes /odometry/filtered
    # Delayed 3 s to let the hardware bridge connect and ESP32 start publishing.
    ekf_node = TimerAction(
        period=3.0,
        actions=[Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                ekf_config,
                {'use_sim_time': False},
            ],
        )],
    )

    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        robot_state_publisher,
        hardware_bridge,
        ekf_node,
    ])