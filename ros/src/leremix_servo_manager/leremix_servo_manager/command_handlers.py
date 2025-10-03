#!/usr/bin/env python3
"""
Command Handlers for LeRemix Robot
Handles ROS message processing and motor command conversion
"""

import math
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class CommandHandlers:
    """Handles ROS command message processing and motor control"""
    
    def __init__(self, node: Node, motor_manager, config):
        self.node = node
        self.motor_manager = motor_manager
        self.config = config
        
        # Command state tracking
        self.last_base_commands = []
        self.last_arm_commands = []
        self.last_head_commands = []
    
    def handle_base_command(self, msg: Float64MultiArray):
        """Handle base/locomotion motor commands with new control logic"""
        
        if not self.config.loc_enable:
            return
            
        if len(msg.data) != len(self.config.loc_ids):
            self.node.get_logger().warn(f"Base command size mismatch: expected {len(self.config.loc_ids)}, got {len(msg.data)}")
            return
            
        # Check if commands have changed
        if (len(self.last_base_commands) == len(msg.data) and 
            all(abs(a - b) < 1e-6 for a, b in zip(msg.data, self.last_base_commands))):
            return
            
        self.last_base_commands = list(msg.data)
        
        # MAPPING: Hardware interface sends commands in order of wheel names ["wheel1_rotation", "wheel2_rotation", "wheel3_rotation"]
        # which maps to motor IDs [1,2,3] in config order
        
        # Log the received command for debugging
        wheel_names = ["wheel1_rotation", "wheel2_rotation", "wheel3_rotation"]
        self.node.get_logger().info(f"Base command received: {[f'{wheel_names[i]}={msg.data[i]:.3f}' for i in range(len(msg.data))]}")
        
        for i, motor_id in enumerate(self.config.loc_ids):
            velocity = msg.data[i]
            
            try:
                # Convert velocity to motor speed
                rad_per_sec = velocity * self.config.loc_speed_scale
                max_rad_per_sec = 10.0
                speed_float = rad_per_sec / max_rad_per_sec * 3400.0
                speed_raw = int(max(-3400, min(3400, speed_float)))
                
                # Log the motor command for debugging
                self.node.get_logger().info(f"  Motor {motor_id} ({wheel_names[i]}): velocity={velocity:.3f} → speed_raw={speed_raw}")
                
                # Send command to motor
                self.motor_manager.send_velocity_command(motor_id, speed_raw)
                    
            except Exception as e:
                self.node.get_logger().error(f"Failed to send velocity command to motor {motor_id}: {e}")
    
    def handle_arm_command(self, msg: Float64MultiArray):
        """Handle arm motor commands"""
        if not self.config.arm_enable:
            return
            
        if len(msg.data) != len(self.config.arm_ids):
            self.node.get_logger().warn(f"Arm command size mismatch: expected {len(self.config.arm_ids)}, got {len(msg.data)}")
            return
            
        # Check if commands have changed
        if (len(self.last_arm_commands) == len(msg.data) and 
            all(abs(a - b) < 1e-6 for a, b in zip(msg.data, self.last_arm_commands))):
            return
            
        self.last_arm_commands = list(msg.data)
        
        # MAPPING: Hardware interface sends commands in order of joint names ["1","2","3","4","5","6"]
        # which maps directly to motor IDs [4,5,6,7,8,9] in order (msg.data[0] -> motor_id 4, etc.)
        for i, motor_id in enumerate(self.config.arm_ids):
            try:
                # Convert from ROS radians to servo ticks
                angle_rad = msg.data[i]
                pos_ticks = self._radians_to_ticks(angle_rad)
                
                # Apply arm speed scaling
                scaled_speed = int(200 * self.config.arm_speed_scale)
                scaled_accel = int(200 * self.config.arm_speed_scale)
                
                self.motor_manager.send_position_command(motor_id, pos_ticks, scaled_speed, scaled_accel)
                
            except Exception as e:
                self.node.get_logger().error(f"Failed to send arm command to motor {motor_id}: {e}")
    
    def handle_head_command(self, msg: Float64MultiArray):
        """Handle head motor commands"""
        if not self.config.head_enable:
            return
            
        if len(msg.data) != len(self.config.head_ids):
            self.node.get_logger().warn(f"Head command size mismatch: expected {len(self.config.head_ids)}, got {len(msg.data)}")
            return
            
        # Check if commands have changed
        if (len(self.last_head_commands) == len(msg.data) and 
            all(abs(a - b) < 1e-6 for a, b in zip(msg.data, self.last_head_commands))):
            return
            
        self.last_head_commands = list(msg.data)
        
        for i, motor_id in enumerate(self.config.head_ids):
            try:
                # Convert from ROS radians to servo ticks
                angle_rad = msg.data[i]
                pos_ticks = self._radians_to_ticks(angle_rad)
                
                # Apply head speed scaling
                scaled_speed = int(100 * self.config.head_speed_scale)
                scaled_accel = int(150 * self.config.head_speed_scale)
                
                self.motor_manager.send_position_command(motor_id, pos_ticks, scaled_speed, scaled_accel)
                
            except Exception as e:
                self.node.get_logger().error(f"Failed to send head command to motor {motor_id}: {e}")
    
    def _radians_to_ticks(self, angle_rad: float) -> int:
        """Convert radians to servo ticks"""
        servo_center = 2048
        pos_ticks = int(servo_center + (angle_rad / (2.0 * math.pi)) * self.config.ticks_per_rev)
        return max(0, min(4095, pos_ticks))
    
    def _ticks_to_radians(self, pos_ticks: int) -> float:
        """Convert servo ticks to radians"""
        servo_center = 2048
        return (pos_ticks - servo_center) / self.config.ticks_per_rev * 2.0 * math.pi