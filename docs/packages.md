# LLMy ROS Packages


| Package | Purpose | Language | Key Topics | Launch Command |
|---------|---------|----------|------------|----------------|
| **llmy_description** | Robot URDF model & transforms | XML/Python | `/robot_description`, `/tf`, `/joint_states` | `ros2 launch llmy_description view_robot.launch.py` |
| **llmy_servo_manager** | Direct motor control (C++) | C++ | `/motor_manager/*_cmd`, `/motor_manager/joint_states` | `ros2 launch llmy_servo_manager servo_manager.launch.py` |
| **llmy_servo_manager_py** | Motor control with advanced braking | Python | `/motor_manager/*_cmd`, `/motor_manager/joint_states` | `ros2 launch llmy_servo_manager_py servo_manager.launch.py` |
| **llmy_control_plugin** | ros2_control hardware bridge | C++ | `/cmd_vel`, `/arm_controller/joint_trajectory` | `ros2 launch llmy_control_plugin bringup.launch.py` |
| **llmy_teleop_xbox** | Xbox controller interface | Python | `/cmd_vel`, `/joy` | `ros2 launch llmy_teleop_xbox teleop_xbox.launch.py` |
| **llmy_control** | Controller configurations | YAML | N/A (config files) | Loaded by control_plugin |
| **llmy_camera** | RGB-D camera & vision | Python | `/head_camera/*`, `/scan`, `/wrist_camera/*` | `ros2 launch llmy_camera camera.launch.py` |
| **llmy_imu** | IMU sensor & fusion | Python | `/imu/data`, `/imu/fused` | `ros2 launch llmy_imu imu.launch.py` |

### 📦 Detailed Package Information

#### **📐 llmy_description - Robot Model**

**What it does:** Provides the complete URDF/Xacro robot model with accurate physical properties, joint limits, collision meshes, and visual representations. This is the "digital twin" of your physical robot.

**Nodes launched:**
- `robot_state_publisher` - Publishes robot transforms and joint states
- `joint_state_publisher` - (Optional) For manual joint control in simulation

**How to run:**
```bash
# Standalone URDF visualization
ros2 launch llmy_description view_robot.launch.py

# Load robot model for other packages
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro $(ros2 pkg prefix llmy_description)/share/llmy_description/urdf/llmy.urdf.xacro)"
```

**Key topics:**
- `/robot_description` - URDF robot model
- `/tf` and `/tf_static` - Robot transforms
- `/joint_states` - Current joint positions


#### **🔧 llmy_servo_manager - Direct Motor Control (C++)**

**What it does:** Handles low-level communication with FEETECH STS servos via serial protocol. Converts ROS2 joint commands into servo-specific position/velocity commands and provides real-time telemetry feedback.

**Nodes launched:**
- `servo_manager_node` - Main servo communication node
- `ping_test` - (Optional) Motor connectivity testing utility

**How to run:**
```bash
# Main servo manager (requires hardware)
ros2 launch llmy_servo_manager servo_manager.launch.py

# Test motor connectivity
ros2 launch llmy_servo_manager ping_test.launch.py

# Custom serial port
ros2 launch llmy_servo_manager servo_manager.launch.py port:=/dev/ttyTHS1 baud:=1000000
```

**Key topics:**
- `/motor_manager/base_cmd` - Base motor velocity commands
- `/motor_manager/arm_cmd` - Arm motor position commands  
- `/motor_manager/head_cmd` - Head motor position commands
- `/motor_manager/joint_states` - Motor telemetry feedback

**Configuration:** Edit `config/servo_manager.yaml` to adjust motor IDs, serial settings, and control parameters.

---

#### **🐍 llmy_servo_manager_py - Python Motor Control**

**What it does:** Python-based alternative to the C++ servo manager with modular architecture, advanced braking system, and improved error handling. Provides the same functionality with enhanced safety features and easier customization.

**Features:**
- **Advanced Braking System**: Three configurable braking methods to prevent power spikes
- **Modular Architecture**: Clean separation of motor management, configuration, and command handling
- **Improved Safety**: Comprehensive connectivity testing and graceful error recovery
- **Enhanced Telemetry**: Detailed motor state monitoring and diagnostics

**Nodes launched:**
- `servo_manager_node.py` - Main Python servo manager node

**How to run:**
```bash
# Main Python servo manager (requires hardware)
ros2 launch llmy_servo_manager_py servo_manager.launch.py

# Test brake methods
ros2 launch llmy_servo_manager_py test_brake_methods.launch.py

# Custom configuration
ros2 launch llmy_servo_manager_py servo_manager.launch.py config_file:=custom_config.yaml
```

**Key topics:** (Same as C++ version)
- `/motor_manager/base_cmd` - Base motor velocity commands
- `/motor_manager/arm_cmd` - Arm motor position commands  
- `/motor_manager/head_cmd` - Head motor position commands
- `/motor_manager/joint_states` - Motor telemetry feedback

**Configuration:** Edit `config/servo_manager.yaml` to adjust motor IDs, serial settings, braking method, and control parameters.

> **💡 When to use:** Choose the Python version for enhanced safety features, easier customization, or when you need advanced braking control. Both versions are fully compatible with the rest of the system.

---

#### **🔌 llmy_control_plugin - Hardware Bridge**

**What it does:** Acts as the ros2_control hardware interface, bridging standard ROS2 controllers with the servo manager. Enables seamless integration with MoveIt2, navigation, and other ROS2 tools.

**Nodes launched:**
- `controller_manager` - ros2_control manager
- `diff_drive_controller` - Base movement controller (skid-steer)
- `arm_controller` - Arm trajectory controller
- `joint_state_broadcaster` - Joint state publisher

**How to run:**
```bash
# Launch with hardware interface
ros2 launch llmy_control_plugin bringup.launch.py

# Simulation mode (requires Gazebo)
ros2 launch llmy_control_plugin bringup.launch.py use_sim_time:=true
```

**Key topics:**
- `/cmd_vel` - Base velocity commands (input)
- `/arm_controller/joint_trajectory` - Arm trajectory commands (input)
- `/joint_states` - Combined joint states (output)
- `/controller_manager/*` - Controller status and management

---

#### **🎮 llmy_teleop_xbox - Manual Control**

**What it does:** Provides intuitive Xbox controller mapping for manual robot operation. Maps controller inputs to robot movements with safety limits and smooth control.

**Nodes launched:**
- `teleop_xbox_node` - Xbox controller interface
- `joy_node` - Joystick driver

**How to run:**
```bash
# Standard Xbox controller
ros2 launch llmy_teleop_xbox teleop_xbox.launch.py

# Custom controller device
ros2 launch llmy_teleop_xbox teleop_xbox.launch.py device:=/dev/input/js1
```

**Controller mapping:**
- **Right stick:** Base movement (forward/back + rotation)
- **RB/LB:** Arm joint 1 (+/-)
- **RT/LT:** Arm joint 2 (+/-)
- **Y/A:** Arm joint 3 (+/-)
- **B/X:** Arm joint 4 (+/-)
- **Start/Back:** Arm joint 5 (+/-)
- **Stick clicks:** Arm joint 6 (+/-)
- **D-pad up/down:** Camera pan
- **D-pad left/right:** Camera tilt

**Key topics:**
- `/cmd_vel` - Base velocity output
- `/arm_controller/joint_trajectory` - Arm movement output
- `/joy` - Raw joystick data

---

#### **⚙️ llmy_control - Controller Configuration**

**What it does:** Provides controller configurations and parameter files for the skid-steer differential drive base and 6-DOF arm using standard ros2_control patterns.

**Configuration files:**
- `config/controllers.yaml` - Controller parameters
- `config/ros2_control.yaml` - Hardware interface config
- `config/joint_limits.yaml` - Safety limits

**Loaded by:** llmy_control_plugin (no standalone launch)

**Key parameters:**
- Velocity limits for base wheels
- Position/velocity limits for arm joints
- Controller gains and dynamics






#### **📷 llmy_camera - Vision System**

**What it does:** Integrates RGB-D cameras with compressed image transport and depth-to-laser conversion. Provides both manipulation-ready RGB-D data and navigation-ready 2D laser scans.

**Nodes launched:**
- `realsense2_camera_node` - RealSense D435 driver
- `depthimage_to_laserscan` - Depth to 2D scan converter
- `image_transport` - Compressed image streaming
- `usb_cam_node` - (Optional) Wrist camera driver

**How to run:**
```bash
# Full camera system
ros2 launch llmy_camera camera.launch.py

# RealSense only
ros2 launch llmy_camera realsense.launch.py

# With custom resolution
ros2 launch llmy_camera camera.launch.py width:=1280 height:=720
```

**Key topics:**
- `/head_camera/color/image_raw` - RGB images
- `/head_camera/depth/image_rect_raw` - Depth images
- `/head_camera/rgbd` - Combined RGB-D data
- `/scan` - 2D laser scan from depth
- `/wrist_camera/image_raw` - Wrist camera feed

---

#### **📐 llmy_imu - Orientation Sensing**

**What it does:** Handles ICM20948 9-DOF sensor integration with Madgwick sensor fusion, providing calibrated orientation data crucial for navigation and balance.

**Nodes launched:**
- `imu_filter_madgwick` - Sensor fusion node
- `icm20948_driver` - Raw IMU data driver

**How to run:**
```bash
# IMU with sensor fusion
ros2 launch llmy_imu imu.launch.py

# Raw IMU data only
ros2 run llmy_imu icm20948_driver
```

**Key topics:**
- `/imu/data` - Raw IMU measurements
- `/imu/fused` - Fused orientation estimate
- `/imu/mag` - Magnetometer data

**Calibration:** Run calibration sequence on first setup:
```bash
ros2 run llmy_imu calibrate_imu
```