import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
    GroupAction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



def pkg(name: str) -> str:
    return get_package_share_directory(name)

def generate_launch_description():
    bringup_share    = pkg('real_bringup')
    desc_share       = pkg('real_description')
    controllers_share = pkg('real_controllers')

    urdf_file        = os.path.join(desc_share, 'urdf', 'Robot_Final.urdf')
    controllers_yaml = os.path.join(controllers_share, 'config', 'controllers.yaml')
    ekf_yaml         = os.path.join(bringup_share, 'config', 'ekf.yaml')

    # Arguments
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
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/rplidar',
        description='Serial port for the RPLidar A1',
    )

    serial_port = LaunchConfiguration('serial_port')
    baud_rate   = LaunchConfiguration('baud_rate')
    lidar_port  = LaunchConfiguration('lidar_port')

    # Robot description (URDF
    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()

    robot_description = {'robot_description': robot_description_content}

    # 1. robot_state_publisher 
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': False},
        ],
    )


    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port':      lidar_port,
            'serial_baudrate':  115200,
            'frame_id':         'LIDAR',
            'angle_compensate': True,
            'scan_mode':        'Standard',
        }],
    )

    # Hardware bridge 
    #   • opens /dev/esp32 at the given baud rate
    #   • reads JSON lines from ESP32: {"lv":…,"rv":…,"lp":…,"rp":…,"ax":…,…}
    #   • publishes /joint_states  (for diff_drive_controller feedback)
    #   • publishes /imu/data      (for EKF fusion, same as before)
    #   • subscribes /left_wheel_velocity_controller/commands  → serial cmd
    #   • subscribes /right_wheel_velocity_controller/commands → serial cmd
    hardware_bridge_node = Node(
        package='real_controllers',
        executable='hardware_bridge_node',
        name='hardware_bridge_node',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'baud_rate':   baud_rate,
        }],
    )

    # ros2_control node (controller_manager)
    # Loads controllers.yaml which defines:
    #   • diff_drive_controller   (publishes /odom, subscribes /cmd_vel)
    #   • joint_state_broadcaster (republishes /joint_states
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[
            robot_description,
            controllers_yaml,
            {'use_sim_time': False},
        ],
    )

    #  Spawn controllers (after controller_manager is up) 
    #
    # TimerAction gives controller_manager 3 s to initialise before spawning controllers 
    spawn_jsb = TimerAction(
        period=5.0,   # 5 s to ensure hardware_bridge_node is publishing /joint_states first
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
                output='screen',
            )
        ],
    )

    spawn_diff_drive = TimerAction(
        period=5.5,   # 0.5 s after JSB so state interfaces are active first
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['diff_drive_controller'],
                output='screen',
            )
        ],
    )

    # ── 7. EKF – robot_localization
    # Fuses /odom (from diff_drive_controller) + /imu/data (from bridge node).
    # ekf.yaml is unchanged from the micro-ROS version.
    # odom_frame: odom,  base_link_frame: CHASSIS  — must match controllers.yaml
    #
    ekf_node = TimerAction(
        period=7.0,   # wait for diff_drive_controller to start publishing /odom
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[
                    ekf_yaml,
                    {'use_sim_time': False},
                ],
                remappings=[
                    ('/odometry/filtered', '/odom_filtered'),  
                ],
            )
        ],
    )

    # ── 8. Joystick / WebSocket bridge (unchanged from micro-ROS version) ────
    joystick_node = Node(
        package='real_bringup',
        executable='joystick_ws_node.py',
        name='joystick_ws_node',
        output='screen',
        # Publishes /cmd_vel (Twist) at port 8765 — same as before
    )

   
    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        lidar_port_arg,

        robot_state_publisher,
        rplidar_node,
        hardware_bridge_node,
        controller_manager_node,

        spawn_jsb,
        spawn_diff_drive,
        ekf_node,

        joystick_node,
    ])