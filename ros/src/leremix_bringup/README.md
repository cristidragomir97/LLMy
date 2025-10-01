# leremix_bringup

Launch configuration package for bringing up the complete LeRemix robot system, providing convenient launch files for different deployment scenarios.

## Overview

The LeRemix Bringup package contains launch files that coordinate multiple LeRemix packages to bring up the complete robot system. It provides different launch configurations for simulation, hardware, and development scenarios.

## Features

- **Complete System Launch**: Single launch files for full robot startup
- **Modular Configuration**: Enable/disable specific components as needed
- **Hardware/Simulation Switch**: Easy switching between real robot and simulation
- **Development Support**: Debug and testing configurations
- **Parameterized Launch**: Configurable robot parameters and components

## Launch Files

### Core Launch Files

#### `robot.launch.py`
Brings up the complete LeRemix robot for hardware operation:
- Robot description and state publisher
- Hardware control plugin and servo manager
- Controller spawning and configuration
- IMU and sensor nodes
- Camera system (if enabled)

```bash
# Launch complete hardware robot
ros2 launch leremix_bringup robot.launch.py

# Launch without camera
ros2 launch leremix_bringup robot.launch.py enable_camera:=false

# Launch with custom servo manager
ros2 launch leremix_bringup robot.launch.py servo_manager_type:=cpp
```

#### `simulation.launch.py`
Brings up the complete robot in Gazebo simulation:
- Gazebo world and robot spawning
- Simulated controllers and hardware interfaces
- Robot description for simulation
- Optional sensor simulation

```bash
# Launch complete simulation
ros2 launch leremix_bringup simulation.launch.py

# Launch simulation without GUI
ros2 launch leremix_bringup simulation.launch.py gui:=false

# Launch with Xbox controller
ros2 launch leremix_bringup simulation.launch.py enable_teleop:=true
```

#### `minimal.launch.py`
Minimal robot setup for development and testing:
- Robot description only
- Basic joint state publisher
- No hardware interfaces

```bash
# Launch minimal robot for development
ros2 launch leremix_bringup minimal.launch.py

# Launch minimal with RViz
ros2 launch leremix_bringup minimal.launch.py enable_rviz:=true
```

### Specialized Launch Files

#### `teleop.launch.py`
Teleoperation-focused launch for manual control:
- Hardware or simulation robot
- Xbox controller teleoperation
- Optional camera feed
- RViz visualization

```bash
# Launch robot with teleoperation
ros2 launch leremix_bringup teleop.launch.py

# Launch simulation teleop
ros2 launch leremix_bringup teleop.launch.py use_sim_time:=true
```

#### `sensors.launch.py`
Sensor-only launch for perception development:
- Camera system
- IMU sensor
- TF publishing
- No motor control

```bash
# Launch sensors only
ros2 launch leremix_bringup sensors.launch.py

# Launch with specific camera
ros2 launch leremix_bringup sensors.launch.py camera_type:=d435i
```

## Launch Arguments

### Common Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | `false` | Use simulation time |
| `enable_camera` | `true` | Launch camera system |
| `enable_imu` | `true` | Launch IMU sensor |
| `enable_teleop` | `false` | Launch Xbox teleoperation |
| `enable_rviz` | `false` | Launch RViz visualization |
| `servo_manager_type` | `python` | Servo manager type: `python` or `cpp` |

### Hardware-Specific Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `robot_description_file` | `LeRemix.xacro` | URDF file to use |
| `controllers_file` | `controllers.hw.yaml` | Controller configuration |
| `servo_config_file` | `servo_manager.yaml` | Servo manager configuration |
| `serial_port` | `/dev/ttyTHS1` | Serial port for motor communication |

### Simulation-Specific Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `world_file` | `turtlebot3_house.world` | Gazebo world file |
| `spawn_x` | `0.0` | Robot spawn X position |
| `spawn_y` | `0.0` | Robot spawn Y position |
| `spawn_z` | `0.05` | Robot spawn Z position |
| `gui` | `true` | Show Gazebo GUI |
| `pause` | `false` | Start simulation paused |

## Usage Examples

### Hardware Deployment

```bash
# Basic robot startup
ros2 launch leremix_bringup robot.launch.py

# Robot with teleoperation
ros2 launch leremix_bringup robot.launch.py enable_teleop:=true

# Robot without camera (faster startup)
ros2 launch leremix_bringup robot.launch.py enable_camera:=false

# Custom serial port
ros2 launch leremix_bringup robot.launch.py serial_port:=/dev/ttyUSB0
```

### Simulation Development

```bash
# Complete simulation
ros2 launch leremix_bringup simulation.launch.py

# Headless simulation for CI
ros2 launch leremix_bringup simulation.launch.py gui:=false

# Simulation with teleoperation
ros2 launch leremix_bringup simulation.launch.py enable_teleop:=true

# Custom spawn position
ros2 launch leremix_bringup simulation.launch.py spawn_x:=2.0 spawn_y:=1.0
```

### Development and Testing

```bash
# Minimal setup for URDF testing
ros2 launch leremix_bringup minimal.launch.py enable_rviz:=true

# Sensor testing only
ros2 launch leremix_bringup sensors.launch.py

# Teleoperation testing
ros2 launch leremix_bringup teleop.launch.py
```

## Configuration Files

The bringup package relies on configuration files from other packages:

### Robot Description
- `leremix_description/urdf/LeRemix.xacro` - Main robot URDF
- `leremix_description/config/joint_limits.yaml` - Joint limits

### Controllers
- `leremix_control/config/controllers.hw.yaml` - Hardware controllers
- `leremix_control/config/controllers.sim.yaml` - Simulation controllers

### Servo Manager
- `leremix_servo_manager*/config/servo_manager.yaml` - Motor configuration

### Sensors
- `leremix_camera/config/realsense.yaml` - Camera configuration
- `leremix_imu/config/imu.yaml` - IMU configuration

## System Dependencies

### Hardware Dependencies
- Serial communication drivers
- Camera drivers (RealSense SDK)
- IMU libraries (Adafruit CircuitPython)
- USB/Bluetooth drivers (Xbox controller)

### Software Dependencies
- All LeRemix packages
- ROS2 Humble
- Gazebo Classic
- RViz2
- ros2_control
- Navigation stack (optional)

## Troubleshooting

### Launch Failures

```bash
# Check package dependencies
rosdep check --from-paths . --ignore-src

# Verify all packages built successfully
colcon list --packages-select leremix_*

# Check launch file syntax
ros2 launch --show-args leremix_bringup robot.launch.py
```

### Hardware Issues

```bash
# Check serial permissions
ls -la /dev/ttyTHS1
sudo usermod -a -G dialout $USER

# Verify camera connection
ros2 topic list | grep camera

# Check motor communication
ros2 topic echo /motor_manager/joint_states --once
```

### Simulation Issues

```bash
# Check Gazebo installation
gazebo --version

# Verify controller loading
ros2 control list_controllers

# Check robot spawning
ros2 topic echo /joint_states --once
```

## Integration

This package coordinates:
- **leremix_description** - Robot model and visualization
- **leremix_control** - Controller configuration
- **leremix_servo_manager** - Motor control
- **leremix_camera** - Perception system
- **leremix_imu** - Orientation sensing
- **leremix_teleop_xbox** - Manual control
- **leremix_gazebo** - Simulation environment

## Customization

### Adding New Launch Configurations

1. Create new launch file in `launch/` directory
2. Define launch arguments and parameters
3. Include necessary component launch files
4. Test with different parameter combinations
5. Document new arguments and usage

### Modifying Existing Launches

1. Edit appropriate launch file
2. Update argument defaults if needed
3. Test all parameter combinations
4. Update documentation

## Best Practices

- Use descriptive launch file names
- Provide sensible default parameters
- Include comprehensive argument documentation
- Test both hardware and simulation paths
- Use conditional launching for optional components
- Log important startup information

## License

Part of the LeRemix Robot project. See main repository for license information.