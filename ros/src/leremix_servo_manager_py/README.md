# LeRemix Servo Manager (Python)

A Python-based servo control system for the LeRemix Robot, providing advanced motor management with improved braking logic to prevent power spikes.

## Overview

The LeRemix Servo Manager Python package provides a comprehensive solution for controlling FEETECH ST3215 servo motors in the LeRemix Robot. It features modular architecture, configurable braking systems, and robust error handling.

## Features

- **Multi-Motor Support**: Controls locomotion, arm, and head motor groups independently
- **Advanced Braking**: Three configurable braking methods to prevent power spikes
- **Modular Architecture**: Clean separation of concerns across multiple modules
- **Real-time Telemetry**: Publishes motor states at configurable rates
- **Robust Error Handling**: Comprehensive connectivity testing and graceful error recovery
- **ROS2 Integration**: Native ROS2 node with proper QoS configuration

## Architecture

### Core Modules

- **`servo_manager_node.py`**: Main ROS2 node orchestrating all components
- **`config.py`**: Configuration management and parameter validation
- **`motor_manager.py`**: Low-level motor communication and control
- **`brake_system.py`**: Safe motor braking with multiple strategies
- **`command_handlers.py`**: ROS message processing and motor command conversion
- **`telemetry.py`**: Motor state publishing and monitoring

## Configuration

The servo manager is configured via ROS2 parameters, typically loaded from `config/servo_manager.yaml`:

### Communication Parameters
```yaml
port: "/dev/ttyTHS1"          # Serial port for motor communication
baud: 1000000                 # Baud rate (default: 1 Mbps)
ticks_per_rev: 4096           # Encoder ticks per revolution
telemetry_rate: 30.0          # Telemetry publishing rate (Hz)
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

### Braking System Configuration
```yaml
# Braking method: "torque_disable", "velocity_ramp", "position_brake"
brake_method: "torque_disable"  # Gentlest method (recommended)
velocity_ramp_time: 0.5         # Ramp down time for velocity_ramp (seconds)
brake_acceleration: 100         # Acceleration for position_brake method
```

## Braking Methods

The system supports three braking strategies to prevent power spikes:

### 1. Torque Disable (Recommended)
- **Method**: Immediately disables motor torque
- **Pros**: Gentlest on power supply, fastest response
- **Cons**: Motor freewheels to stop
- **Use Case**: Default for most applications

### 2. Velocity Ramp
- **Method**: Gradually reduces velocity to zero over configured time
- **Pros**: Controlled deceleration, maintains some control
- **Cons**: Takes longer to stop
- **Use Case**: When controlled deceleration is important

### 3. Position Brake
- **Method**: Commands motor to hold current position
- **Pros**: Precise stopping, maintains position
- **Cons**: Can cause current spikes if motor is moving fast
- **Use Case**: When position holding is critical

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

## Usage

### Basic Launch
```bash
ros2 launch leremix_servo_manager_py servo_manager.launch.py
```

### With Custom Configuration
```bash
ros2 launch leremix_servo_manager_py servo_manager.launch.py \
    config_file:=custom_servo_config.yaml
```

### Testing Brake Methods
```bash
ros2 launch leremix_servo_manager_py test_brake_methods.launch.py
```

## Motor Control Examples

### Locomotion Control
```python
# Move forward at 0.5 rad/s
msg = Float64MultiArray()
msg.data = [0.5, 0.5, 0.5]  # All three locomotion motors
pub.publish(msg)

# Stop all locomotion motors
msg.data = [0.0, 0.0, 0.0]
pub.publish(msg)
```

### Arm Control
```python
# Move arm to specific joint angles
msg = Float64MultiArray()
msg.data = [0.0, 0.0, -1.5, 1.5, 0.0, 1.0]  # Six arm joints in radians
pub.publish(msg)
```

### Head Control
```python
# Pan and tilt head
msg = Float64MultiArray()
msg.data = [0.5, -0.3]  # Pan right, tilt down slightly
pub.publish(msg)
```

## Monitoring and Diagnostics

### View Motor States
```bash
ros2 topic echo /motor_manager/joint_states
```

### Check Node Status
```bash
ros2 node info /servo_manager_node_py
```

### Monitor Motor Connectivity
The node automatically tests motor connectivity on startup and logs detailed status information.

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

3. **Power Spikes During Operation**
   - Use "torque_disable" braking method
   - Reduce speed scaling factors
   - Check motor acceleration settings

### Debug Mode
Enable debug logging to see detailed brake operations:
```bash
ros2 run leremix_servo_manager_py servo_manager_node.py --ros-args --log-level DEBUG
```

## Development

### Module Structure
```
leremix_servo_manager_py/
├── config.py              # Configuration management
├── motor_manager.py        # Motor communication
├── brake_system.py         # Braking strategies  
├── command_handlers.py     # ROS message processing
├── telemetry.py           # State publishing
└── servo_manager_node.py  # Main ROS2 node
```

### Adding New Features
1. Create new module in appropriate category
2. Add imports to `__init__.py`
3. Update main node to integrate new functionality
4. Add configuration parameters if needed
5. Update documentation

## Dependencies

- **ROS2**: Humble or newer
- **Python**: 3.8+
- **st3215**: FEETECH servo library
- **rclpy**: ROS2 Python client library

## Safety Considerations

- Always use appropriate braking methods for your application
- Monitor motor temperatures during operation
- Implement emergency stop functionality in your control systems
- Test motor responses before deploying on robot
- Use conservative speed scaling for initial testing

## License

Part of the LeRemix Robot project. See main repository for license information.