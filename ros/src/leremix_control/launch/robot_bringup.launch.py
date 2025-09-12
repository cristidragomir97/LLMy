from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

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

    # Get package share directories
    controllers_cfg = PathJoinSubstitution([
        FindPackageShare('leremix_control_plugin'),
        'config',
        'controllers.yaml'
    ])
    hw_cfg = PathJoinSubstitution([
        FindPackageShare('leremix_control_plugin'),
        'config',
        'ros2_control_esp32_bridge.yaml'
    ])

    # Robot description
    robot_description_content = Command([
        'xacro ', 
        PathJoinSubstitution([
            FindPackageShare('leremix_description'),
            'urdf',
            'LeRemix.xacro'
        ])
    ])

    # Robot state publisher - publishes robot_description topic
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # ros2_control node (controller_manager) - will use robot_description topic
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[{'robot_description': robot_description_content}, controllers_cfg, hw_cfg]
    )

    # Controller spawners - delayed to wait for controller_manager
    spawner_joint_state_broadcaster = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )
    
    spawner_omnidirectional_controller = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['omnidirectional_controller', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )
    
    spawner_arm_controller = TimerAction(
        period=9.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['arm_controller', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )

    spawner_head_controller = TimerAction(
        period=11.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['head_controller', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )

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
        
        # Core robot nodes
        robot_state_publisher,
        controller_manager,
        microros_agent,
        
        # Controller spawners
        spawner_joint_state_broadcaster,
        spawner_omnidirectional_controller,
        spawner_arm_controller,
        spawner_head_controller,
    ])