# 🚀 Getting Started Guide

## 📋 Prerequisites

**For Simulation:**
- Ubuntu 22.04 LTS
- ROS2 Humble
- Gazebo Classic

**For Hardware:**
- All simulation requirements +
- FEETECH STS servos connected via serial (USB or UART)
- Xbox controller (optional)
- RealSense camera (optional)

## 🔧 Installation

```bash
# 1. Install ROS2 Humble (if not already installed)
sudo apt install ros-humble-desktop

# 2. Clone the repository
git clone https://github.com/cristidragomir97/leremix.git leremix_ws
cd leremix_ws/ros

# 3. Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# 4. Build the workspace
colcon build

# 5. Source the workspace
source install/setup.bash
```

## 🎮 Quick Test - Simulation

```bash
# Terminal 1: Launch simulation
ros2 launch leremix_gazebo sim.launch.py

# Terminal 2: Launch Xbox controller (optional)
ros2 launch leremix_teleop_xbox teleop_xbox.launch.py
```

You should see the robot in Gazebo. Use your Xbox controller to drive around!

## 🤖 Quick Test - Hardware

```bash
# Single command launch (recommended)
ros2 launch leremix_bringup bringup_robot.launch.py

# Or with custom serial settings
ros2 launch leremix_bringup bringup_robot.launch.py servo_port:=/dev/ttyTHS1 servo_baud:=1000000
```

## 🔍 Verify Everything Works

```bash
# Check that controllers are running
ros2 control list_controllers

# Monitor motor communication
ros2 topic echo /motor_manager/joint_states --once

# Test motor connectivity
ros2 launch leremix_servo_manager ping_test.launch.py
```

## 🎯 Next Steps

**For Development:**
- Explore the [detailed package documentation](../README.md#-detailed-package-information) 
- Try the individual component launches for debugging
- Check out the [architecture diagrams](../README.md#-architecture) to understand the system

**For Applications:**
- Use MoveIt2 for motion planning: `ros2 launch leremix_moveit_config demo.launch.py`
- Enable navigation with Nav2: `ros2 launch leremix_navigation navigation.launch.py`
- Integrate your AI/ML code via standard ROS2 topics and services

## 🔧 Real Hardware Setup

### Prerequisites

```bash
# Install ROS2 Humble
sudo apt install ros-humble-desktop

# Install additional packages
sudo apt install ros-humble-realsense2-camera \
                 ros-humble-compressed-image-transport \
                 ros-humble-depthimage-to-laserscan \
                 ros-humble-imu-filter-madgwick
```

### 🚀 Launch Robot System

```bash
# Build workspace
cd leremix_ws/ros
colcon build
source install/setup.bash

# Launch complete robot system (single command)
ros2 launch leremix_bringup bringup_robot.launch.py

# Custom serial port and baud rate
ros2 launch leremix_bringup bringup_robot.launch.py servo_port:=/dev/ttyTHS1 servo_baud:=1000000

# Disable specific components
ros2 launch leremix_bringup bringup_robot.launch.py use_camera:=false use_xbox:=false

# Or launch components individually (in separate terminals)
ros2 launch leremix_control base_systems.launch.py         # Servo manager
ros2 launch leremix_control_plugin bringup.launch.py       # Hardware interface
ros2 launch leremix_camera camera.launch.py                # Camera system  
ros2 launch leremix_imu imu.launch.py                      # IMU sensor
ros2 launch leremix_teleop_xbox teleop_xbox.launch.py      # Xbox control

# Optional: Monitor system status
ros2 topic list
ros2 control list_controllers
```

### ✅ Verification

Check that all systems are running:
```bash
# Verify controllers are active
ros2 control list_controllers

# Check camera streams
ros2 topic echo /camera/color/image_raw --once

# Verify IMU data
ros2 topic echo /imu/fused --once  

# Check joint states from motor manager
ros2 topic echo /motor_manager/joint_states --once
```

The robot should now respond to Xbox controller inputs with real hardware!