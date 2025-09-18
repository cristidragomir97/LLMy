#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the path to the config file
    pkg_share = get_package_share_directory('leremix_servo_manager')
    config_file = os.path.join(pkg_share, 'config', 'servo_manager.yaml')
    
    # Declare launch arguments
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to the servo manager config file'
    )
    
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Serial port for motor communication'
    )
    
    baud_arg = DeclareLaunchArgument(
        'baud',
        default_value='2000000',
        description='Baud rate for serial communication'
    )

    # Servo manager node
    servo_manager_node = Node(
        package='leremix_servo_manager',
        executable='servo_manager_node',
        name='servo_manager_node',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'port': LaunchConfiguration('port'),
                'baud': LaunchConfiguration('baud'),
            }
        ],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        config_arg,
        port_arg,
        baud_arg,
        servo_manager_node,
    ])