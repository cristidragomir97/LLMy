from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    spawn_z    = LaunchConfiguration("spawn_z")
    cm_timeout = LaunchConfiguration("cm_timeout")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_spawn_z    = DeclareLaunchArgument("spawn_z",    default_value="0.05")
    declare_cm_timeout = DeclareLaunchArgument("cm_timeout", default_value="60")
    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="true")

    # Use the overlay (includes base xacro + gazebo overlay + ros2_control sim backend)
    xacro_file = PathJoinSubstitution([FindPackageShare("leremix_gazebo"), "urdf", "leremix_gazebo_overlay.xacro"])
    robot_description = {"robot_description": Command([FindExecutable(name="xacro"), " ", xacro_file])}

    # Bridge configuration file
    bridge_config = PathJoinSubstitution([
        FindPackageShare('leremix_gazebo'),
        'config',
        'gz_ros_bridge.yaml'
    ])

    # Set GZ_SIM_RESOURCE_PATH for model discovery
    # Point to the share directory so model://leremix_description/meshes/... URIs work
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            PathJoinSubstitution([FindPackageShare('leremix_description'), '..']),
            ':',
            PathJoinSubstitution([FindPackageShare('leremix_description'), 'meshes'])
        ]
    )

    # Robot State Publisher
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
        output="screen"
    )

    # Gazebo Sim (new Gazebo) - using default empty world
    # -r: start running immediately, -v 4: verbose level 4
    # Add -s flag to run server-only (headless) if GUI is not needed
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            ])
        ),
        launch_arguments=[
            ("gz_args", ["-r -v 4 ", PathJoinSubstitution([FindPackageShare("leremix_gazebo"), "worlds", "sticky_floor.world"])])
        ],
    )

    # Spawn robot using ros_gz_sim create
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_leremix",
        arguments=[
            "-name", "LeRemix",
            "-topic", "robot_description",
            "-z", spawn_z,
            "-x", "0.2",
            "-y", "0.2"
        ],
        output="screen"
    )

    # Clock bridge - must start first for use_sim_time to work properly
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen"
    )

    # ROS-Gazebo Bridge
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        arguments=[
            "--ros-args",
            "-p", ["config_file:=", bridge_config]
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen"
    )

    # Image bridge for efficient camera transport (optional but recommended)
    image_bridge_head_rgb = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="image_bridge_head_rgb",
        arguments=["/head_camera/rgb"],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen"
    )

    image_bridge_head_depth = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="image_bridge_head_depth",
        arguments=["/head_camera/depth"],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen"
    )

    image_bridge_wrist = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="image_bridge_wrist",
        arguments=["/wrist_camera/rgb"],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen"
    )

    # Controller Spawners — make sure names match your YAML (leremix_control/config/*.yaml)
    jsb = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_joint_state_broadcaster",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", cm_timeout],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    omni_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_base_controller",
        arguments=["omnidirectional_controller", "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", cm_timeout],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_arm_controller",
        arguments=["arm_controller", "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", cm_timeout],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    head_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_head_controller",
        arguments=["head_controller", "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", cm_timeout],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # Chain: spawn entity → JS broadcaster → base & arm & head
    after_spawn = RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb]))
    after_js    = RegisterEventHandler(OnProcessExit(target_action=jsb,   on_exit=[omni_controller, arm_controller, head_controller]))

    return LaunchDescription([
        declare_spawn_z,
        declare_cm_timeout,
        declare_use_sim_time,
        gz_resource_path,
        rsp,
        gazebo,
        clock_bridge,
        spawn,
        bridge,
        image_bridge_head_rgb,
        image_bridge_head_depth,
        image_bridge_wrist,
        after_spawn,
        after_js
    ])
