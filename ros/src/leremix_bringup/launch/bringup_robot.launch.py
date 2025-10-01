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

    use_rplidar_arg = DeclareLaunchArgument(
        'use_rplidar',
        default_value='false',
        description='Enable RPLidar C1'
    )

    use_base_systems_arg = DeclareLaunchArgument(
        'use_base_systems',
        default_value='true',
        description='Enable base systems (servo manager)'
    )
    
    use_control_stack_arg = DeclareLaunchArgument(
        'use_control_stack',
        default_value='true',
        description='Enable control stack (controllers, robot state publisher)'
    )
    
    servo_port_arg = DeclareLaunchArgument(
        'servo_port',
        default_value='/dev/ttyTHS1',
        description='Serial port for servo manager'
    )
    
    servo_baud_arg = DeclareLaunchArgument(
        'servo_baud',
        default_value='1000000',
        description='Baudrate for servo manager'
    )

    # Launch configurations
    use_camera = LaunchConfiguration('use_camera')
    use_imu = LaunchConfiguration('use_imu')
    use_xbox = LaunchConfiguration('use_xbox')
    use_rplidar = LaunchConfiguration('use_rplidar')
    use_base_systems = LaunchConfiguration('use_base_systems')
    use_control_stack = LaunchConfiguration('use_control_stack')
    servo_port = LaunchConfiguration('servo_port')
    servo_baud = LaunchConfiguration('servo_baud')

    # Include base systems (servo manager)
    base_systems_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('leremix_control'),
            'launch',
            'base_systems.launch.py'
        ]),
        launch_arguments={
            'servo_port': servo_port,
            'servo_baud': servo_baud,
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

    # Include RPLidar C1 launch file
    rplidar_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('rplidar_ros'),
            'launch',
            'view_rplidar_c1_launch.py'
        ]),
        condition=IfCondition(use_rplidar)
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
        use_rplidar_arg,
        use_base_systems_arg,
        use_control_stack_arg,
        servo_port_arg,
        servo_baud_arg,

        # Launch includes and nodes
        base_systems_launch,
        control_stack_launch,
        camera_launch,
        imu_launch,
        xbox_launch,
        rplidar_launch,
        cmd_vel_mux,
        rosboard,
    ])