from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    microros_transport_arg = DeclareLaunchArgument(
        'microros_transport',
        default_value='serial',
        description='Transport for micro-ROS agent'
    )
    
    microros_device_arg = DeclareLaunchArgument(
        'microros_device',
        default_value='/dev/ttyUSB0',
        description='Device for micro-ROS agent'
    )
    
    microros_baudrate_arg = DeclareLaunchArgument(
        'microros_baudrate',
        default_value='115200',
        description='Baudrate for micro-ROS agent'
    )

    # Launch configurations
    microros_transport = LaunchConfiguration('microros_transport')
    microros_device = LaunchConfiguration('microros_device')
    microros_baudrate = LaunchConfiguration('microros_baudrate')

    # micro-ROS agent node
    microros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        arguments=[microros_transport, '--dev', microros_device, '-b', microros_baudrate],
        output='screen'
    )

    return LaunchDescription([
        # Launch arguments
        microros_transport_arg,
        microros_device_arg,
        microros_baudrate_arg,
        
        # Base system nodes
        microros_agent,
    ])