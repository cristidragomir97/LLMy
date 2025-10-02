#!/usr/bin/env python3
"""
Telemetry System for LeRemix Robot
Handles motor state publishing and monitoring
"""

from rclpy.node import Node
from sensor_msgs.msg import JointState


class TelemetrySystem:
    """Manages motor telemetry publishing"""
    
    def __init__(self, node: Node, motor_manager, config):
        self.node = node
        self.motor_manager = motor_manager
        self.config = config
        
        # Create publisher
        self.state_pub = self.node.create_publisher(JointState, "/motor_manager/joint_states", 10)
        self.node.get_logger().info("  ✅ Joint state publisher: /motor_manager/joint_states")
    
    def publish_telemetry(self):
        """Publish motor telemetry as JointState message"""
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        
        # Read all enabled motors
        enabled_ids = self.config.get_enabled_motor_ids()
            
        for motor_id in enabled_ids:
            pos, speed = self.motor_manager.read_motor_state(motor_id)
            
            if pos is not None and speed is not None:
                msg.name.append(f"motor_{motor_id}")
                
                # Convert servo ticks to radians
                pos_rad = self._ticks_to_radians(pos)
                msg.position.append(pos_rad)
                
                # Convert speed to rad/s
                vel_rad_s = self._speed_to_rad_per_sec(speed)
                msg.velocity.append(vel_rad_s)
                
        if len(msg.name) > 0:
            self.state_pub.publish(msg)
    
    def _ticks_to_radians(self, pos_ticks: int) -> float:
        """Convert servo ticks to radians"""
        servo_center = 2048
        return (pos_ticks - servo_center) / self.config.ticks_per_rev * 2.0 * 3.14159
    
    def _speed_to_rad_per_sec(self, speed: int) -> float:
        """Convert motor speed to rad/s"""
        max_rad_per_sec = 10.0
        return speed / 1023.0 * max_rad_per_sec
    
    def get_motor_summary(self) -> dict:
        """Get summary of motor states for diagnostics"""
        enabled_ids = self.config.get_enabled_motor_ids()
        summary = {
            "total_enabled": len(enabled_ids),
            "responding": 0,
            "positions": {},
            "velocities": {}
        }
        
        for motor_id in enabled_ids:
            pos, speed = self.motor_manager.read_motor_state(motor_id)
            if pos is not None and speed is not None:
                summary["responding"] += 1
                summary["positions"][motor_id] = self._ticks_to_radians(pos)
                summary["velocities"][motor_id] = self._speed_to_rad_per_sec(speed)
        
        return summary