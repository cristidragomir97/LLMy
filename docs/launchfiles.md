# LeRemix Launch Files Documentation

This document provides an overview of all launch files in the LeRemix ROS 2 workspace and their parameters.

## Main System Launch Files

### leremix_bringup/launch/bringup_robot.launch.py
**Description**: Main launch file for the physical robot with all subsystems.

**Parameters**:
- `use_camera` (bool, default: `true`) - Enable camera (leremix_camera)
- `use_imu` (bool, default: `true`) - Enable IMU (leremix_imu) 
- `use_xbox` (bool, default: `true`) - Enable Xbox controller (leremix_teleop_xbox)
- `use_base_systems` (bool, default: `true`) - Enable base systems (servo manager)
- `use_control_stack` (bool, default: `true`) - Enable control stack (controllers, robot state publisher)
- `servo_port` (string, default: `/dev/ttyTHS1`) - Serial port for servo manager
- `servo_baud` (string, default: `1000000`) - Baudrate for servo manager

**Includes**: base_systems.launch.py, control_stack.launch.py, camera.launch.py, imu.launch.py, teleop_xbox.launch.py

**Nodes**: twist_mux (cmd_vel multiplexing), rosboard (web dashboard)

### leremix_bringup/launch/bringup_desktop.launch.py
**Description**: Launch file for desktop/development environment with micro-ROS agent.

**Parameters**:
- `use_xbox` (bool, default: `false`) - Enable Xbox controller (leremix_teleop_xbox)
- `microros_transport` (string, default: `serial`) - Transport for micro-ROS agent
- `microros_device` (string, default: `/dev/ttyUSB0`) - Device for micro-ROS agent
- `microros_baudrate` (string, default: `115200`) - Baudrate for micro-ROS agent

**Includes**: teleop_xbox.launch.py

**Nodes**: robot_state_publisher, controller_manager, spawner nodes (delayed), micro_ros_agent, rosboard

## Control System Launch Files

### leremix_control/launch/base_systems.launch.py
**Description**: Launches base hardware systems including servo manager.

**Parameters**:
- `servo_port` (string, default: `/dev/ttyTHS1`) - Serial port for servo manager
- `servo_baud` (string, default: `1000000`) - Baudrate for servo manager
- `servo_config` (string, default: `''`) - Path to servo manager config file (uses package default if empty)

**Includes**: servo_manager.launch.py

### leremix_control/launch/control_stack.launch.py
**Description**: Launches the ros2_control stack with robot description and controllers.

**Parameters**: None

**Nodes**: robot_state_publisher, controller_manager, controller spawners (joint_state_broadcaster, omnidirectional_controller, arm_controller, head_controller)

### leremix_control/launch/robot_bringup.launch.py
**Description**: Hardware interface launch with micro-ROS agent.

**Parameters**:
- `microros_transport` (string, default: `serial`) - Transport for micro-ROS agent
- `microros_device` (string, default: `/dev/ttyUSB0`) - Device for micro-ROS agent
- `microros_baudrate` (string, default: `2000000`) - Baudrate for micro-ROS agent

**Nodes**: robot_state_publisher, controller_manager, controller spawners (delayed), micro_ros_agent

### leremix_control_plugin/launch/bringup.launch.py
**Description**: Simple control plugin bringup without robot description.

**Parameters**: None

**Nodes**: controller_manager, controller spawners (joint_state_broadcaster, omnidirectional_controller, arm_controller)

## Camera System Launch Files

### leremix_camera/launch/camera.launch.py
**Description**: Comprehensive camera setup with RealSense and USB wrist camera.

**Parameters**:
- `camera_name` (string, default: `head_camera`) - RealSense camera name
- `wrist_camera_device` (string, default: `/dev/video0`) - Wrist camera device path
- `enable_wrist_camera` (bool, default: `true`) - Enable wrist camera
- `device_type` (string, default: `''`) - RealSense device type (e.g., d435i, d455, l515)
- `enable_compressed` (bool, default: `true`) - Enable compressed image transport
- `enable_laser_scan` (bool, default: `true`) - Enable depth to laser scan conversion
- `laser_scan_min_height` (float, default: `-0.5`) - Minimum height for laser scan slice
- `laser_scan_max_height` (float, default: `0.5`) - Maximum height for laser scan slice
- `laser_scan_angle_min` (float, default: `-1.57`) - Minimum angle for laser scan (-90 degrees)
- `laser_scan_angle_max` (float, default: `1.57`) - Maximum angle for laser scan (+90 degrees)
- `laser_scan_angle_increment` (float, default: `0.005`) - Angular resolution of laser scan
- `laser_scan_range_min` (float, default: `0.2`) - Minimum range for laser scan
- `laser_scan_range_max` (float, default: `10.0`) - Maximum range for laser scan

**Includes**: realsense2_camera/launch/rs_launch.py

**Nodes**: compressed image transport, depthimage_to_laserscan, wrist camera (usb_cam)

### leremix_camera/launch/realsense_only.launch.py
**Description**: Minimal RealSense camera setup.

**Parameters**:
- `camera_name` (string, default: `camera`) - RealSense camera name
- `device_type` (string, default: `d435i`) - RealSense device type

**Includes**: realsense2_camera/launch/rs_launch.py

## Sensor Launch Files

### leremix_imu/launch/imu.launch.py
**Description**: IMU sensor setup with Madgwick filter.

**Parameters**:
- `publish_rate` (float, default: `200.0`) - Publishing rate for IMU data in Hz
- `frame_id` (string, default: `imu_link`) - Frame ID for IMU data
- `use_mag` (bool, default: `true`) - Use magnetometer for orientation filtering

**Nodes**: imu_node, imu_filter_madgwick

## Teleop Launch Files

### leremix_teleop_xbox/launch/teleop_xbox.launch.py
**Description**: Xbox controller teleoperation setup.

**Parameters**:
- `joy_dev` (string, default: `0`) - Joystick device ID
- `switch_controllers` (bool, default: `true`) - Switch controllers flag

**Nodes**: joy_node, teleop_xbox (with controller command topics configured)

## Simulation Launch Files

### leremix_gazebo/launch/sim.launch.py
**Description**: Gazebo simulation environment setup.

**Parameters**:
- `gui` (bool, default: `true`) - Enable Gazebo GUI
- `pause` (bool, default: `false`) - Start simulation paused
- `spawn_z` (float, default: `0.05`) - Z position for robot spawn
- `cm_timeout` (string, default: `60`) - Controller manager timeout

**Includes**: gazebo_ros/launch/gzserver.launch.py, gazebo_ros/launch/gzclient.launch.py

**Nodes**: robot_state_publisher, spawn_entity, controller spawners (with event handlers for sequencing)

## Hardware Manager Launch Files

### leremix_servo_manager/launch/servo_manager.launch.py
**Description**: Servo motor manager for hardware communication.

**Parameters**:
- `config_file` (string, default: package config file) - Path to the servo manager config file
- `port` (string, default: `/dev/ttyUSB0`) - Serial port for motor communication
- `baud` (string, default: `2000000`) - Baud rate for serial communication

**Nodes**: servo_manager_node

### leremix_servo_manager/launch/ping_test.launch.py
**Description**: Motor ping test utility.

**Parameters**:
- `config_file` (string, default: package config file) - Path to the servo manager config file  
- `port` (string, default: `/dev/ttyUSB0`) - Serial port for motor communication
- `baud` (string, default: `2000000`) - Baud rate for serial communication

**Nodes**: ping_test (motor_ping_test)

## Usage Examples

### Launch the complete robot system:
```bash
ros2 launch leremix_bringup bringup_robot.launch.py
```

### Launch with custom servo port:
```bash
ros2 launch leremix_bringup bringup_robot.launch.py servo_port:=/dev/ttyUSB1
```

### Launch desktop development environment:
```bash
ros2 launch leremix_bringup bringup_desktop.launch.py
```

### Launch simulation:
```bash
ros2 launch leremix_gazebo sim.launch.py
```

### Launch only camera system:
```bash
ros2 launch leremix_camera camera.launch.py
```