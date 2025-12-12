# llmy_rviz

RViz2 configuration and visualization package for the LLMy robot.

## Features

This package provides comprehensive visualization for:

### Robot Visualization
- **Robot Model**: Full 3D visualization of the LLMy robot from URDF
- **TF Tree**: Transform frame visualization showing all coordinate frames

### Sensor Data
- **LaserScan**: 2D lidar visualization (`/scan` topic)
- **Head Camera RGB**: Color camera feed from head camera
- **Head Camera Depth**: Depth camera visualization (disabled by default)
- **Wrist Camera**: RGB camera on robot wrist (disabled by default)
- **Depth PointCloud**: 3D point cloud from depth camera (disabled by default)

### SLAM & Navigation
- **Map**: Occupancy grid map from SLAM (`/map` topic)
- **Global Plan**: Planned path from navigation (`/plan` topic)
- **Local Plan**: Local planner trajectory (`/local_plan` topic)
- **Local Costmap**: Real-time obstacle avoidance costmap
- **Global Costmap**: Global planning costmap

### Tools
- **2D Pose Estimate**: Set initial robot pose for localization
- **2D Nav Goal**: Send navigation goals to the robot
- **Measure**: Measure distances in the environment
- **Publish Point**: Click to publish 3D points

## Usage

### Launch RViz with default configuration:
```bash
ros2 launch llmy_rviz rviz.launch.py
```

### Launch with custom config:
```bash
ros2 launch llmy_rviz rviz.launch.py rviz_config:=/path/to/config.rviz
```

### Launch with hardware (no sim time):
```bash
ros2 launch llmy_rviz rviz.launch.py use_sim_time:=false
```

## Configuration

The main RViz configuration file is located at:
```
rviz/llmy.rviz
```

You can customize this file or create new configurations and save them in the `rviz/` directory.

## Topics Subscribed

- `/robot_description` - Robot model URDF
- `/scan` - LaserScan data
- `/map` - Occupancy grid map
- `/plan` - Global navigation plan
- `/local_plan` - Local navigation plan
- `/local_costmap/costmap` - Local costmap
- `/global_costmap/costmap` - Global costmap
- `/head_camera/rgb` - Head camera RGB image
- `/head_camera/depth` - Head camera depth image
- `/head_camera/depth/points` - Depth point cloud
- `/wrist_camera/rgb` - Wrist camera RGB image

## Dependencies

- rviz2
- rviz_common
- rviz_default_plugins
- robot_state_publisher
- llmy_description
