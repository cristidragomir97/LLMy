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

    use_servo_manager_arg = DeclareLaunchArgument(
        'use_servo_manager',
        default_value='true',
        description='Enable servo manager'
    )
    
    use_control_stack_arg = DeclareLaunchArgument(
        'use_control_stack',
        default_value='true',
        description='Enable control stack (controllers, robot state publisher)'
    )

    # Launch configurations
    use_camera = LaunchConfiguration('use_camera')
    use_imu = LaunchConfiguration('use_imu')
    use_xbox = LaunchConfiguration('use_xbox')
    use_servo_manager = LaunchConfiguration('use_servo_manager')
    use_control_stack = LaunchConfiguration('use_control_stack')

    # Include servo manager launch
    servo_manager_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('leremix_servo_manager'),
            'launch',
            'servo_manager.launch.py'
        ]),
        condition=IfCondition(use_servo_manager)
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
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('leremix_bringup'),
                'config',
                'twist_mux.yaml'
            ])
        ],
        remappings=[
            ('cmd_vel_out', '/omnidirectional_controller/cmd_vel_unstamped')
        ],
        output='screen'
    )

    return LaunchDescription([
        # Launch arguments
        use_camera_arg,
        use_imu_arg,
        use_xbox_arg,
        use_servo_manager_arg,
        use_control_stack_arg,

        # Launch includes and nodes
        servo_manager_launch,
        control_stack_launch,
        #camera_launch,
        imu_launch,
        xbox_launch,
        cmd_vel_mux,  # Temporarily commented out
    ])