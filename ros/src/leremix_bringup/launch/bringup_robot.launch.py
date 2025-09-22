from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml
import os

def load_config():
    """Load the bringup configuration file"""
    config_file = os.path.join(
        FindPackageShare('leremix_bringup').find('leremix_bringup'),
        'config', 'bringup.yaml'
    )
    
    try:
        with open(config_file, 'r') as file:
            return yaml.safe_load(file)
    except FileNotFoundError:
        # Fallback to default values if config file not found
        return {
            'components': {
                'camera': {'enabled': True},
                'imu': {'enabled': True},
                'xbox_controller': {'enabled': True},
                'base_systems': {'enabled': True},
                'control_stack': {'enabled': True},
                'rosbridge': {'enabled': True}
            },
            'servo_manager': {
                'port': '/dev/ttyTHS1',
                'baud_rate': 1000000
            },
            'cmd_vel_mux': {
                'topics': {
                    'xbox': {'topic': 'cmd_vel_xbox', 'timeout': 0.5, 'priority': 10},
                    'nav': {'topic': 'cmd_vel_nav', 'timeout': 0.5, 'priority': 5},
                    'teleop': {'topic': 'cmd_vel_teleop', 'timeout': 0.5, 'priority': 15}
                },
                'output_topic': '/omnidirectional_controller/cmd_vel_unstamped'
            }
        }

def generate_launch_description():
    # Load configuration
    config = load_config()
    
    # Optional config file override argument
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('leremix_bringup'),
            'config',
            'bringup.yaml'
        ]),
        description='Path to bringup configuration file'
    )
    
    config_file = LaunchConfiguration('config_file')

    # Include base systems (servo manager) if enabled
    base_systems_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('leremix_control'),
            'launch',
            'base_systems.launch.py'
        ]),
        launch_arguments={
            'servo_port': str(config['servo_manager']['port']),
            'servo_baud': str(config['servo_manager']['baud_rate']),
        }.items(),
        condition=IfCondition(str(config['components']['base_systems']['enabled']).lower())
    )

    # Include control stack if enabled
    control_stack_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('leremix_control'),
            'launch',
            'control_stack.launch.py'
        ]),
        condition=IfCondition(str(config['components']['control_stack']['enabled']).lower())
    )
    
    # Include leremix_camera launch file if enabled
    camera_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('leremix_camera'),
            'launch',
            'camera.launch.py'
        ]),
        condition=IfCondition(str(config['components']['camera']['enabled']).lower())
    )

    # Include leremix_imu launch file if enabled
    imu_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('leremix_imu'),
            'launch',
            'imu.launch.py'
        ]),
        condition=IfCondition(str(config['components']['imu']['enabled']).lower())
    )

    # Include leremix_teleop_xbox launch file if enabled
    xbox_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('leremix_teleop_xbox'),
            'launch',
            'teleop_xbox.launch.py'
        ]),
        condition=IfCondition(str(config['components']['xbox_controller']['enabled']).lower())
    )

    # cmd_vel mux node for command velocity multiplexing
    cmd_vel_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[{
            'topics': {
                topic_name: {
                    'topic': topic_config['topic'],
                    'timeout': topic_config['timeout'],
                    'priority': topic_config['priority']
                }
                for topic_name, topic_config in config['cmd_vel_mux']['topics'].items()
            }
        }],
        remappings=[
            ('cmd_vel_out', config['cmd_vel_mux']['output_topic'])
        ],
        output='screen'
    )

    # rosboard node for web dashboard
    rosboard = Node(
        package='rosboard',
        executable='rosboard_node',
        output='screen'
    )

    # rosbridge websocket server if enabled
    rosbridge_server = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        output='screen',
        condition=IfCondition(str(config['components']['rosbridge']['enabled']).lower())
    )

    return LaunchDescription([
        # Launch arguments
        config_file_arg,
        
        # Launch includes and nodes
        base_systems_launch,
        control_stack_launch,
        camera_launch,
        imu_launch,
        xbox_launch,
        cmd_vel_mux,
        rosboard,
        rosbridge_server,
    ])