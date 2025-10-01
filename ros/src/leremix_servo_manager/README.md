# LeRemix Servo Manager (C++)

C++ implementation of the servo control system for the LeRemix Robot, providing high-performance motor management with real-time communication to FEETECH ST3215 servo motors.

## Overview

The LeRemix Servo Manager C++ package provides a fast, efficient solution for controlling FEETECH ST3215 servo motors. It offers lower latency than the Python version and is optimized for real-time control applications.

## Features

- **High-Performance**: Native C++ implementation for minimal latency
- **Multi-Motor Support**: Controls locomotion, arm, and head motor groups
- **Real-time Communication**: Optimized serial communication with FEETECH servos
- **ROS2 Integration**: Native ROS2 node with proper QoS configuration
- **Motor Telemetry**: Real-time position and velocity feedback
- **Configurable Parameters**: Flexible motor configuration and scaling

## Architecture

### Core Components

- **`servo_manager_node.cpp`**: Main ROS2 node
- **`motor_communication.cpp`**: Low-level FEETECH servo protocol
- **`config_loader.cpp`**: Configuration management
- **Headers**: Type definitions and interface declarations

## Configuration

The servo manager is configured via ROS2 parameters, typically loaded from `config/servo_manager.yaml`:

### Communication Parameters
```yaml
port: "/dev/ttyTHS1"          # Serial port for motor communication
baud: 1000000                 # Baud rate (default: 1 Mbps)
ticks_per_rev: 4096           # Encoder ticks per revolution
telemetry_rate: 50.0          # Telemetry publishing rate (Hz)
```

### Motor Group Configuration
```yaml
# Enable/disable motor groups
loc_enable: true              # Locomotion motors
arm_enable: true              # Arm motors
head_enable: true             # Head motors

# Motor IDs for each group
loc_ids: [1, 2, 3]           # Locomotion motor IDs
arm_ids: [4, 5, 6, 7, 8, 9]  # Arm motor IDs
head_ids: [10, 11]           # Head motor IDs

# Acceleration settings (per motor)
loc_accel: [15, 15, 15]
arm_accel: [20, 20, 15, 15, 25, 25]
head_accel: [15, 15]

# Speed scaling factors (0.0 to 2.0)
loc_speed_scale: 0.2         # Conservative for locomotion
arm_speed_scale: 1.0         # Normal for arm
head_speed_scale: 1.0        # Normal for head
```

## ROS2 Interface

### Subscribed Topics

- **`/motor_manager/base_cmd`** (`Float64MultiArray`)
  - Velocity commands for locomotion motors (rad/s)
  - Array size must match number of locomotion motors

- **`/motor_manager/arm_cmd`** (`Float64MultiArray`)
  - Position commands for arm motors (radians)
  - Array size must match number of arm motors

- **`/motor_manager/head_cmd`** (`Float64MultiArray`)
  - Position commands for head motors (radians)
  - Array size must match number of head motors

### Published Topics

- **`/motor_manager/joint_states`** (`JointState`)
  - Current motor positions and velocities
  - Published at configured telemetry rate
  - Contains all enabled motors with naming: `motor_{id}`

## Build and Installation

### Prerequisites

```bash
# Install dependencies
sudo apt install build-essential cmake
sudo apt install ros-humble-rclcpp ros-humble-std-msgs ros-humble-sensor-msgs
```

### Build Package

```bash
cd /path/to/your/ros2_workspace
colcon build --packages-select leremix_servo_manager
source install/setup.bash
```

## Usage

### Basic Launch
```bash
ros2 launch leremix_servo_manager servo_manager.launch.py
```

### With Custom Configuration
```bash
ros2 launch leremix_servo_manager servo_manager.launch.py \
    config_file:=custom_servo_config.yaml
```

### Test Motor Connectivity
```bash
ros2 launch leremix_servo_manager ping_test.launch.py
```

## Motor Control Examples

### Command Line Testing
```bash
# Move locomotion motors forward
ros2 topic pub /motor_manager/base_cmd std_msgs/Float64MultiArray \
  "{data: [0.5, 0.5, 0.5]}"

# Stop all locomotion motors
ros2 topic pub /motor_manager/base_cmd std_msgs/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0]}"

# Move arm to home position
ros2 topic pub /motor_manager/arm_cmd std_msgs/Float64MultiArray \
  "{data: [0.0, 0.0, -1.5, 1.5, 0.0, 1.0]}"
```

## Performance Characteristics

### C++ vs Python Version

| Feature | C++ Version | Python Version |
|---------|-------------|----------------|
| **Latency** | <1ms | 2-5ms |
| **CPU Usage** | Very Low | Low-Medium |
| **Memory** | Low | Medium |
| **Startup Time** | Fast | Medium |
| **Braking Methods** | Basic | Advanced (3 types) |

### When to Use C++ Version
- Real-time control requirements
- High-frequency motor commands (>30Hz)
- Resource-constrained systems
- Production deployments

### When to Use Python Version
- Development and prototyping
- Advanced braking requirements
- Easier customization
- Debug and monitoring features

## Troubleshooting

### Common Issues

1. **Serial Port Permission Denied**
   ```bash
   sudo usermod -a -G dialout $USER
   # Then logout and login again
   ```

2. **Motors Not Responding**
   - Check physical connections
   - Verify motor IDs in configuration
   - Ensure proper power supply
   - Check serial port settings

3. **High CPU Usage**
   - Reduce telemetry rate
   - Check for communication errors
   - Verify motor connectivity

### Debug Mode
Enable debug logging:
```bash
ros2 run leremix_servo_manager servo_manager_node --ros-args --log-level DEBUG
```

## Development

### Code Structure
```
leremix_servo_manager/
├── include/leremix_servo_manager/
│   ├── servo_manager_node.hpp
│   ├── motor_communication.hpp
│   └── config_loader.hpp
├── src/
│   ├── servo_manager_node.cpp
│   ├── motor_communication.cpp
│   └── config_loader.cpp
├── config/
│   └── servo_manager.yaml
└── launch/
    ├── servo_manager.launch.py
    └── ping_test.launch.py
```

### Adding New Features
1. Update header files with new interfaces
2. Implement functionality in corresponding .cpp files
3. Update CMakeLists.txt if needed
4. Add configuration parameters
5. Test thoroughly before deployment

## Dependencies

- **ROS2**: Humble or newer
- **CMake**: 3.8+
- **ST3215 Library**: FEETECH servo communication
- **Serial Library**: For UART communication

## Integration

This package works with:
- **leremix_control_plugin** - ros2_control hardware bridge
- **leremix_teleop_xbox** - Xbox controller teleoperation
- **leremix_gazebo** - Simulation environment (via hardware bridge)

## Safety Considerations

- Monitor motor temperatures during operation
- Implement emergency stop functionality
- Test motor responses before deploying on robot
- Use conservative speed scaling for initial testing
- Ensure proper power supply capacity

## License

Part of the LeRemix Robot project. See main repository for license information.