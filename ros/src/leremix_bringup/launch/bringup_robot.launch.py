from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Enable camera (leremix_camera)'
    )
    
    use_imu_arg = DeclareLaunchArgument(
        'use_imu',
        default_value='true',
        description='Enable IMU (leremix_imu)'
    )
    
    use_xbox_arg = DeclareLaunchArgument(
        'use_xbox',
        default_value='true',
        description='Enable Xbox controller (leremix_teleop_xbox)'
    )
    
    use_base_systems_arg = DeclareLaunchArgument(
        'use_base_systems',
        default_value='true',
        description='Enable base systems (micro-ROS agent)'
    )
    
    use_control_stack_arg = DeclareLaunchArgument(
        'use_control_stack',
        default_value='true',
        description='Enable control stack (controllers, robot state publisher)'
    )
    
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
    use_camera = LaunchConfiguration('use_camera')
    use_imu = LaunchConfiguration('use_imu')
    use_xbox = LaunchConfiguration('use_xbox')
    use_base_systems = LaunchConfiguration('use_base_systems')
    use_control_stack = LaunchConfiguration('use_control_stack')
    microros_transport = LaunchConfiguration('microros_transport')
    microros_device = LaunchConfiguration('microros_device')
    microros_baudrate = LaunchConfiguration('microros_baudrate')

    # Include base systems (micro-ROS agent)
    base_systems_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('leremix_control'),
            'launch',
            'base_systems.launch.py'
        ]),
        launch_arguments={
            'microros_transport': microros_transport,
            'microros_device': microros_device,
            'microros_baudrate': microros_baudrate
        }.items(),
        condition=IfCondition(use_base_systems)
    )

    # Include control stack (robot_state_publisher, controller_manager, spawners)
    control_stack_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('leremix_control'),
            'launch',
            'control_stack.launch.py'
        ]),
        condition=IfCondition(use_control_stack)
    )
    
    # Include leremix_camera launch file
    camera_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('leremix_camera'),
            'launch',
            'camera.launch.py'
        ]),
        condition=IfCondition(use_camera)
    )

    # Include leremix_imu launch file
    imu_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('leremix_imu'),
            'launch',
            'imu.launch.py'
        ]),
        condition=IfCondition(use_imu)
    )

    # Include leremix_teleop_xbox launch file
    xbox_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('leremix_teleop_xbox'),
            'launch',
            'teleop_xbox.launch.py'
        ]),
        condition=IfCondition(use_xbox)
    )


    # cmd_vel mux node for command velocity multiplexing
    cmd_vel_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[{
            'topics': {
                'xbox': {
                    'topic': 'cmd_vel_xbox',
                    'timeout': 0.5,
                    'priority': 10
                },
                'nav': {
                    'topic': 'cmd_vel_nav',
                    'timeout': 0.5,
                    'priority': 5
                },
                'teleop': {
                    'topic': 'cmd_vel_teleop',
                    'timeout': 0.5,
                    'priority': 15
                }
            }
        }],
        remappings=[
            ('cmd_vel_out', '/omnidirectional_controller/cmd_vel_unstamped')
        ],
        output='screen'
    )

    # rosboard node for web dashboard
    rosboard = Node(
        package='rosboard',
        executable='rosboard_node',
        output='screen'
    )

    return LaunchDescription([
        # Launch arguments
        use_camera_arg,
        use_imu_arg,
        use_xbox_arg,
        use_base_systems_arg,
        use_control_stack_arg,
        microros_transport_arg,
        microros_device_arg,
        microros_baudrate_arg,
        
        # Launch includes and nodes
        base_systems_launch,
        control_stack_launch,
        camera_launch,
        imu_launch,
        xbox_launch,
        cmd_vel_mux,
        rosboard,
    ])