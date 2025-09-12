from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    joy_dev = LaunchConfiguration('joy_dev')
    start_switch = LaunchConfiguration('switch_controllers')

    return LaunchDescription([
        DeclareLaunchArgument('joy_dev', default_value='0'),
        DeclareLaunchArgument('switch_controllers', default_value='true'),

        Node(package='joy', executable='joy_node', name='joy',
             parameters=[{'device_id': joy_dev, 'deadzone': 0.15, 'autorepeat_rate': 25.0}],
             output='screen'),

        Node(package='leremix_teleop_xbox', executable='teleop_xbox', name='teleop_xbox',
             parameters=[{
                 'cmd_vel_topic': '/omnidirectional_controller/cmd_vel_unstamped',
                 'arm_cmd_topic': '/arm_controller/commands',
                 'head_cmd_topic': '/head_controller/commands',
                 'arm_increment': 0.0175,  # 1.0 degree
                 'arm_rate': 10.0,
             }],
             output='screen'),
    ])
