# leremix_bringup

The LeRemix Bringup package provides a single, modular launch file that coordinates all LeRemix subsystems to bring up the complete robot. It provides flexible configuration through launch arguments to enable/disable specific components as needed.

- **Command Velocity Multiplexing**: Integrates twist_mux for multiple control sources
- **Web Dashboard**: Includes rosboard for browser-based monitoring

## Launch Files

### `bringup_robot.launch.py`

The main launch file that brings up all LeRemix subsystems:

**Components:**
- **Base Systems** (`leremix_control/base_systems.launch.py`): Servo manager for motor control
- **Control Stack** (`leremix_control/control_stack.launch.py`): Robot state publisher, controller manager, and controller spawners
- **Camera** (`leremix_camera/camera.launch.py`): RGB-D camera system
- **IMU** (`leremix_imu/imu.launch.py`): Orientation sensor
- **Xbox Teleoperation** (`leremix_teleop_xbox/teleop_xbox.launch.py`): Manual control interface
- **RPLidar** (`rplidar_ros/view_rplidar_c1_launch.py`): 2D laser scanner (optional)
- **Command Velocity Mux**: Multiplexes control commands from different sources (xbox, nav, teleop)
- **Rosboard**: Web-based dashboard for monitoring and visualization

## Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `use_camera` | `true` | Enable camera system |
| `use_imu` | `true` | Enable IMU sensor |
| `use_xbox` | `true` | Enable Xbox controller teleoperation |
| `use_rplidar` | `false` | Enable RPLidar C1 |
| `use_base_systems` | `true` | Enable servo manager (motor control) |
| `use_control_stack` | `true` | Enable robot state publisher and controllers |
| `servo_port` | `/dev/ttyTHS1` | Serial port for servo communication |
| `servo_baud` | `1000000` | Baudrate for servo communication |

## Usage Examples

### Full System Startup

```bash
# Launch complete robot with all systems
ros2 launch leremix_bringup bringup_robot.launch.py
```

### Custom Configurations

```bash
# Launch without camera (faster startup)
ros2 launch leremix_bringup bringup_robot.launch.py use_camera:=false

# Launch with RPLidar C1
ros2 launch leremix_bringup bringup_robot.launch.py use_rplidar:=true

# Launch without Xbox controller
ros2 launch leremix_bringup bringup_robot.launch.py use_xbox:=false

# Launch with custom serial port
ros2 launch leremix_bringup bringup_robot.launch.py servo_port:=/dev/ttyUSB0

# Minimal setup (only control stack, no sensors or teleop)
ros2 launch leremix_bringup bringup_robot.launch.py use_camera:=false use_imu:=false use_xbox:=false

# Full sensor suite (camera, IMU, and LiDAR)
ros2 launch leremix_bringup bringup_robot.launch.py use_rplidar:=true
```

### Development Scenarios

```bash
# Sensors only (no motor control)
ros2 launch leremix_bringup bringup_robot.launch.py use_base_systems:=false use_control_stack:=false

# Control only (no sensors)
ros2 launch leremix_bringup bringup_robot.launch.py use_camera:=false use_imu:=false
```

## Command Velocity Multiplexing

The launch file includes a `twist_mux` node that multiplexes command velocity from multiple sources:

| Source | Topic | Priority | Description |
|--------|-------|----------|-------------|
| Teleop | `/cmd_vel_teleop` | 15 (highest) | Manual teleoperation override |
| Xbox | `/cmd_vel_xbox` | 10 | Xbox controller input |
| Nav | `/cmd_vel_nav` | 5 (lowest) | Navigation stack commands |

All sources are multiplexed to `/omnidirectional_controller/cmd_vel_unstamped` with a 0.5 second timeout.

## Web Dashboard

The launch file starts a rosboard instance for web-based monitoring. Access the dashboard at:
```
http://localhost:8888
```

Rosboard provides:
- Topic visualization
- TF tree viewer
- Camera feeds
- Service/action inspection
- Parameter configuration
