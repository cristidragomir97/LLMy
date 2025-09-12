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
    microros_transport = LaunchConfiguration('microros_transport')
    microros_device = LaunchConfiguration('microros_device')
    microros_baudrate = LaunchConfiguration('microros_baudrate')

    # Include robot core components (robot_state_publisher, controller_manager, spawners, microros_agent)
    robot_bringup = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('leremix_control'),
            'launch',
            'robot_bringup.launch.py'
        ]),
        launch_arguments={
            'microros_transport': microros_transport,
            'microros_device': microros_device,
            'microros_baudrate': microros_baudrate
        }.items()
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
        microros_transport_arg,
        microros_device_arg,
        microros_baudrate_arg,
        
        # Launch includes and nodes
        robot_bringup,
        camera_launch,
        imu_launch,
        xbox_launch,
        rosboard,
    ])