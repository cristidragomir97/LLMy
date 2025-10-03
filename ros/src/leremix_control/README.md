# leremix_control

Controller configuration and launch files for the LeRemix robot, providing ros2_control setup for omnidirectional base movement, 6-DOF arm, and pan-tilt head control.

## Overview

The leremix_control package manages the robot's control stack by coordinating the servo manager, robot state publisher, controller manager, and controller spawners. It provides modular launch files for different system components.

### Architecture

The control system uses a layered architecture:

```
High-Level Controllers (MoveIt2, Nav2, Teleop)
              ↓
ros2_control Controllers (arm_controller, omnidirectional_controller, etc.)
              ↓
leremix_control_plugin (Hardware Interface)
              ↓
ROS2 Topics (/motor_manager/base_cmd, /arm_cmd, /head_cmd, /joint_states)
              ↓
leremix_servo_manager (Serial Communication)
              ↓
Hardware (ST3215 Servos)
```

### Data Flow

**Command Path:**
1. High-level planners (MoveIt2/Nav2) or teleop nodes publish goals/commands
2. ros2_control controllers convert to joint commands
3. Hardware interface plugin receives commands via ros2_control interfaces
4. Plugin publishes to `/motor_manager/base_cmd`, `/arm_cmd`, `/head_cmd` topics
5. Servo manager receives topic messages and sends serial commands to motors

**Feedback Path:**
1. Servo manager reads motor positions/velocities via serial
2. Servo manager publishes to `/motor_manager/joint_states` topic
3. Hardware interface plugin subscribes and updates ros2_control state interfaces
4. Controllers read state for closed-loop control
5. Joint state broadcaster publishes to `/joint_states` for visualization and planning

## Launch Files

### `base_systems.launch.py`

Launches the low-level servo manager that communicates with the robot's motors:

**Components:**
- Servo manager (from `leremix_servo_manager` package)

```bash
# Launch servo manager with default settings
ros2 launch leremix_control base_systems.launch.py

# Custom serial port
ros2 launch leremix_control base_systems.launch.py servo_port:=/dev/ttyUSB0

# Custom baudrate
ros2 launch leremix_control base_systems.launch.py servo_baud:=115200
```

**Launch Arguments:**
| Argument | Default | Description |
|----------|---------|-------------|
| `servo_port` | `/dev/ttyTHS1` | Serial port for servo communication |
| `servo_baud` | `1000000` | Baudrate for servo communication |
| `servo_config` | `` | Path to servo config file (uses default if empty) |

### `control_stack.launch.py`

Launches the complete ros2_control stack including robot state publisher and all controllers:

**Components:**
- Robot state publisher (publishes URDF and TF)
- Controller manager (ros2_control node)
- Joint state broadcaster (publishes `/joint_states`)
- Omnidirectional controller (base movement)
- Arm controller (6-DOF arm control)
- Head controller (pan-tilt camera control)

```bash
# Launch complete control stack
ros2 launch leremix_control control_stack.launch.py
```

**Spawned Controllers:**
- `joint_state_broadcaster` - Publishes joint states (delay: 5s)
- `omnidirectional_controller` - Base movement control (delay: 7s)
- `arm_controller` - Arm trajectory control (delay: 9s)
- `head_controller` - Head pan/tilt control (delay: 11s)

**Note:** Controllers are spawned with delays to ensure proper initialization sequence.

## Motion System Initialization

The LeRemix motion system follows a carefully orchestrated initialization sequence to ensure proper hardware setup and safe operation:

### Initialization Sequence

1. **Servo Manager Startup** (0-3 seconds)
   - Establishes serial communication with ST3215 servo motors
   - Reads current servo positions and states
   - Publishes initial joint positions to `/motor_manager/joint_states`
   - Performs servo health checks and error detection

2. **Hardware Interface Activation** (3-5 seconds)
   - ros2_control hardware plugin initializes
   - Subscribes to `/motor_manager/joint_states` for state feedback
   - Establishes command topic publishers (`/motor_manager/base_cmd`, `/arm_cmd`, `/head_cmd`)
   - Synchronizes ros2_control state interfaces with actual hardware positions

3. **Controller Manager Initialization** (5 seconds)
   - Loads controller configurations from YAML files
   - Validates joint interfaces and controller compatibility
   - Prepares controller spawning sequence

4. **Sequential Controller Spawning**
   - **Joint State Broadcaster** (5s delay): Publishes unified joint states for visualization
   - **Omnidirectional Controller** (7s delay): Enables base movement control
   - **Arm Controller** (9s delay): Activates 6-DOF arm trajectory control
   - **Head Controller** (11s delay): Enables pan-tilt camera control

### Initialization Safety Features

- **Position Continuity**: Hardware interface ensures smooth transition from servo positions to controller commands
- **Brake Management**: Servos maintain position during initialization until controllers are active
- **Error Handling**: Failed servo connections or controller spawning errors are logged and reported
- **Startup Monitoring**: Each phase includes status feedback for debugging initialization issues

### Expected Startup Behavior

During normal startup, you should observe:

```bash
[INFO] [servo_manager_node]: 🚀 Launching as Normal ROS Node
[INFO] [servo_manager_node]: Serial connection established on /dev/ttyTHS1
[INFO] [servo_manager_node]: Found 9 servos, reading initial positions...
[INFO] [robot_state_publisher]: Robot state publisher started
[INFO] [controller_manager]: Loading controller configurations...
[INFO] [spawner-joint_state_broadcaster]: Controller spawned successfully
[INFO] [spawner-omnidirectional_controller]: Controller spawned successfully
[INFO] [spawner-arm_controller]: Controller spawned successfully  
[INFO] [spawner-head_controller]: Controller spawned successfully
```

### Troubleshooting Initialization

Common initialization issues and solutions:

- **Servo Connection Failures**: Check serial port permissions and cable connections
- **Controller Spawn Timeouts**: Verify hardware interface is receiving joint states
- **Position Discontinuities**: Ensure servos are not in error state before startup
- **Missing Joint States**: Confirm servo manager is publishing to `/motor_manager/joint_states`

## Controllers

### Omnidirectional Controller

Controls the 3-wheeled omnidirectional base:
- **Type**: `omnidirectional_controller/OmnidirectionalController`
- **Command topic**: `/omnidirectional_controller/cmd_vel_unstamped`
- **Message type**: `geometry_msgs/Twist`
- **Controlled joints**: `back_motor_rotation`, `left_motor_rotation`, `right_motor_rotation`

```bash
# Send movement command (forward and rotate)
ros2 topic pub /omnidirectional_controller/cmd_vel_unstamped geometry_msgs/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"
```

### Arm Controller

Controls the 6-DOF robotic arm:
- **Type**: `joint_trajectory_controller/JointTrajectoryController`
- **Action**: `/arm_controller/follow_joint_trajectory`
- **Command topic**: `/arm_controller/joint_trajectory`
- **Controlled joints**: `1`, `2`, `3`, `4`, `5`, `6`

```bash
# Send arm trajectory command
ros2 topic pub /arm_controller/joint_trajectory trajectory_msgs/JointTrajectory \
  "{joint_names: ['1', '2', '3', '4', '5', '6'], \
   points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 2}}]}"
```

### Head Controller

Controls the pan-tilt camera head:
- **Type**: `joint_trajectory_controller/JointTrajectoryController`
- **Action**: `/head_controller/follow_joint_trajectory`
- **Command topic**: `/head_controller/joint_trajectory`
- **Controlled joints**: `camera_pan`, `camera_tilt`

```bash
# Send head movement command
ros2 topic pub /head_controller/joint_trajectory trajectory_msgs/JointTrajectory \
  "{joint_names: ['camera_pan', 'camera_tilt'], \
   points: [{positions: [0.5, 0.3], time_from_start: {sec: 1}}]}"
```

## Configuration Files

### Hardware Interface Configuration

**`config/ros2_control_bridge.yaml`** - Hardware plugin configuration:
- Plugin selection and parameters
- Topic names for motor manager communication
- Joint definitions and interface types

### URDF Configuration

The `leremix_description` package aims to be neutral and not contain simulation specific, or real-hardware specific tags, therefore the **`urdf/LeRemix.hardware.xacro`** file imports the original URDF and adds ros2_control tags describing the command/state interfaces and joint specifications.

## Usage Examples

### Full System Startup

```bash
# Launch servo manager
ros2 launch leremix_control base_systems.launch.py

# In another terminal, launch control stack
ros2 launch leremix_control control_stack.launch.py
```

**Note:** Typically you'll use `leremix_bringup` which launches both together.

### Monitor Controller Status

```bash
# List active controllers
ros2 control list_controllers

# Check controller manager status
ros2 service list | grep controller_manager

# View joint states
ros2 topic echo /joint_states

# Check controller output
ros2 topic echo /motor_manager/base_cmd
ros2 topic echo /motor_manager/arm_cmd
ros2 topic echo /motor_manager/head_cmd
```

### Manual Controller Management

```bash
# Load and activate a controller
ros2 run controller_manager spawner omnidirectional_controller

# Deactivate a controller
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController \
  "{stop_controllers: ['omnidirectional_controller']}"

# List available controllers
ros2 control list_controller_types
```

## Topic Communication

### Published Topics

- `/joint_states` - Current state of all joints (from `joint_state_broadcaster`)
- `/robot_description` - Robot URDF (from `robot_state_publisher`)
- `/tf` and `/tf_static` - Transform tree (from `robot_state_publisher`)

### Subscribed Topics

- `/motor_manager/joint_states` - Joint feedback from servo manager (hardware interface subscribes)

### Command Topics

Controllers publish to these topics (consumed by hardware interface):
- `/motor_manager/base_cmd` - Base wheel velocities (Float64MultiArray)
- `/motor_manager/arm_cmd` - Arm joint positions (Float64MultiArray)
- `/motor_manager/head_cmd` - Head joint positions (Float64MultiArray)

