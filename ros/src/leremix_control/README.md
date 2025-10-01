# leremix_control
Controllers and parameters for the LeRemix robot, providing ros2_control configuration for base movement and arm control.

## Features
- Controller configurations for omnidirectional base movement
- Arm controller parameters for 6-DOF robotic arm + pan-tilt head 
- Hardware and simulation controller configurations
- Test motion script for validating robot control

## Controllers

### Base Controller
- **omnidirectional_controller** - Controls the omnidirectional mobile base
- Accepts `geometry_msgs/Twist` commands on `/omnidirectional_controller/cmd_vel_unstamped`

### Arm Controller
- **arm_controller** - Position control for 7-axis arm (6 joints + camera tilt)
- Accepts trajectory commands via `/arm_controller/joint_trajectory`
- Joint order: ['1', '2', '3', '4', '5', '6', 'camera_tilt']

### Head Controller

## Configuration Files

- `config/controllers.hw.yaml` - Hardware-specific controller settings  
- `config/controllers.sim.yaml` - Simulation controller settings

## Usage

### Load Controllers

```bash
# Hardware deployment
ros2 control load_controller --set-state active omnidirectional_controller
ros2 control load_controller --set-state active arm_controller

# Or use controller manager spawner
ros2 run controller_manager spawner omnidirectional_controller
ros2 run controller_manager spawner arm_controller
```

### Test Motion

```bash
# Run the test motion script
ros2 run leremix_control test_motion.py

# Send base movement commands
ros2 topic pub /omnidirectional_controller/cmd_vel_unstamped geometry_msgs/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"

# Send arm trajectory commands (example: move all joints slightly)
ros2 topic pub /arm_controller/joint_trajectory trajectory_msgs/JointTrajectory \
  "{joint_names: ['1', '2', '3', '4', '5', '6', 'camera_tilt'], \
   points: [{positions: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.0], time_from_start: {sec: 2}}]}"
```

### Monitor Controller Status

```bash
# List active controllers
ros2 control list_controllers

# Check controller manager services
ros2 service list | grep controller_manager
```

## Dependencies

- ros2_controllers
- omnidirectional_controllers (custom omnidirectional drive controller)
- controller_manager
- Hardware interface (leremix_control_plugin for ESP32 bridge)

## Integration

This package works with:
- **leremix_control_plugin** - Hardware interface 
- **leremix_teleop_xbox** - Xbox controller teleoperation
- **leremix_gazebo** - Simulation environment