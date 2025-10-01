#!/usr/bin/env python3
"""
Motor Manager for LeRemix Robot
Handles low-level motor communication and control using ST3215 library
"""

import time
from st3215 import ST3215
from rclpy.node import Node


class MotorManager:
    """Manages communication and control of FEETECH servo motors"""
    
    def __init__(self, node: Node, port: str, baud: int = 1000000):
        self.node = node
        self.port = port
        self.baud = baud
        self.motor_manager = None
        self.velocity_mode_initialized = set()
        self.running_motors = {}  # motor_id -> current_speed
        
    def connect(self):
        """Connect to the motor bus"""
        try:
            self.motor_manager = ST3215(self.port)
            self.node.get_logger().info(f"✅ Successfully connected to serial port: {self.port}")
            return True
        except Exception as e:
            self.node.get_logger().error(f"❌ Failed to open serial port: {e}")
            self.node.get_logger().error("Please check:")
            self.node.get_logger().error("  - Serial port exists and has correct permissions")
            self.node.get_logger().error("  - Device is connected and powered on")
            self.node.get_logger().error("  - No other processes are using the port")
            return False
    
    def test_connectivity(self, enabled_ids: list):
        """Test connectivity with enabled motors"""
        self.node.get_logger().info("Testing enabled motor connectivity...")
        
        if not enabled_ids:
            self.node.get_logger().warn("No motor groups enabled - skipping connectivity test")
            return 0
            
        connected_motors = 0
        for motor_id in enabled_ids:
            if self.motor_manager.PingServo(motor_id):
                connected_motors += 1
                self.node.get_logger().info(f"  Motor ID {motor_id}: ✅ ONLINE")
            else:
                self.node.get_logger().warn(f"  Motor ID {motor_id}: ❌ OFFLINE")
                
        self.node.get_logger().info(f"Motor connectivity test complete: {connected_motors}/{len(enabled_ids)} enabled motors online")
        return connected_motors
    
    def run_test_sequences(self, motor_ids: list, loc_ids: list, arm_ids: list, head_ids: list, loc_enable: bool):
        """Run short test sequences to verify motor communication"""
        self.node.get_logger().info("Running motor communication test sequences...")
        
        for motor_id in motor_ids:
            if not self.motor_manager.PingServo(motor_id):
                continue
                
            try:
                self.node.get_logger().info(f"Testing motor {motor_id}...")
                
                # Test 1: Brief spin in velocity mode for locomotion motors
                if motor_id in loc_ids and loc_enable:
                    self.motor_manager.SetMode(motor_id, 1)  # Velocity mode
                    self.motor_manager.StartServo(motor_id)
                    self.motor_manager.Rotate(motor_id, 500)  # Slow spin
                    time.sleep(1.0)
                    self.motor_manager.Rotate(motor_id, 0)
                    self.motor_manager.StopServo(motor_id)
                    self.node.get_logger().info(f"  Motor {motor_id}: Velocity test ✅")
                
                # Test 2: Small position move for arm/head motors
                elif motor_id in arm_ids + head_ids:
                    self.motor_manager.SetMode(motor_id, 0)  # Position mode
                    self.motor_manager.StartServo(motor_id)
                    
                    # Read current position and move slightly
                    current_pos = self.motor_manager.ReadPosition(motor_id)
                    if current_pos is not None:
                        test_pos = current_pos + 100  # Small movement
                        test_pos = max(0, min(4095, test_pos))
                        
                        self.motor_manager.MoveTo(motor_id, test_pos, 500, 200)
                        time.sleep(0.5)
                        self.motor_manager.MoveTo(motor_id, current_pos, 500, 200)
                        self.node.get_logger().info(f"  Motor {motor_id}: Position test ✅")
                    
                time.sleep(0.2)  # Brief pause between motors
                
            except Exception as e:
                self.node.get_logger().warn(f"Test failed for motor {motor_id}: {e}")
                
        self.node.get_logger().info("Motor test sequences complete!")
    
    def initialize_locomotion_motors(self, loc_ids: list, loc_accel: list):
        """Initialize locomotion motors for velocity mode"""
        self.node.get_logger().info("Setting locomotion motors to velocity mode...")
        
        for i, motor_id in enumerate(loc_ids):
            accel = loc_accel[i] if i < len(loc_accel) else 50
            
            try:
                self.motor_manager.SetMode(motor_id, 1)  # Velocity mode
           
                self.motor_manager.StartServo(motor_id)
                self.motor_manager.Rotate(motor_id, 0)
                
                self.velocity_mode_initialized.add(motor_id)
                self.running_motors[motor_id] = 0
                self.node.get_logger().info(f"Initialized locomotion motor {motor_id}")
            except Exception as e:
                self.node.get_logger().warn(f"Failed to initialize locomotion motor {motor_id}: {e}")
    
    def initialize_arm_motors(self, arm_ids: list, arm_accel: list, ticks_per_rev: int):
        """Initialize arm motors for position mode"""
        self.node.get_logger().info("Setting arm motors to position mode...")
        arm_home_positions = [0.0, 0.0, -1.5, 1.5, 0.0, 1.0]  # radians
        
        for i, motor_id in enumerate(arm_ids):
            accel = arm_accel[i] if i < len(arm_accel) else 20
            
            try:
                self.motor_manager.SetMode(motor_id, 0)
                self.motor_manager.SetAcceleration(motor_id, accel)
                self.motor_manager.StartServo(motor_id)
                
                if i < len(arm_home_positions):
                    angle_rad = arm_home_positions[i]
                    servo_center = 2048
                    pos_ticks = int(servo_center + (angle_rad / (2.0 * 3.14159)) * ticks_per_rev)
                    pos_ticks = max(0, min(4095, pos_ticks))
                    
                    self.motor_manager.MoveTo(motor_id, pos_ticks, 1000, 300)
                    self.node.get_logger().info(f"Arm motor {motor_id} to home ({angle_rad:.2f} rad)")
                    
            except Exception as e:
                self.node.get_logger().warn(f"Failed to initialize arm motor {motor_id}: {e}")
    
    def initialize_head_motors(self, head_ids: list, head_accel: list, ticks_per_rev: int):
        """Initialize head motors for position mode"""
        self.node.get_logger().info("Setting head motors to position mode...")
        head_home_positions = [0.0, 0.0]  # radians
        
        for i, motor_id in enumerate(head_ids):
            accel = head_accel[i] if i < len(head_accel) else 30
            
            try:
                self.motor_manager.SetMode(motor_id, 0)
                self.motor_manager.SetAcceleration(motor_id, accel)
                self.motor_manager.StartServo(motor_id)
                
                if i < len(head_home_positions):
                    angle_rad = head_home_positions[i]
                    servo_center = 2048
                    pos_ticks = int(servo_center + (angle_rad / (2.0 * 3.14159)) * ticks_per_rev)
                    pos_ticks = max(0, min(4095, pos_ticks))
                    
                    self.motor_manager.MoveTo(motor_id, pos_ticks, 500, 400)
                    self.node.get_logger().info(f"Head motor {motor_id} to home ({angle_rad:.2f} rad)")
                    
            except Exception as e:
                self.node.get_logger().warn(f"Failed to initialize head motor {motor_id}: {e}")
    
    def send_velocity_command(self, motor_id: int, speed_raw: int):
        """Send velocity command to a motor"""
        if speed_raw == 0:
            # Zero velocity: set zero velocity, then stop motor
            self.motor_manager.Rotate(motor_id, 0)
            time.sleep(0.01)
            self.motor_manager.StopServo(motor_id)
            
            # Remove from running motors
            if motor_id in self.running_motors:
                del self.running_motors[motor_id]
        else:
            # Non-zero motor speed: set velocity (motor should already be started)
            # Check if motor needs to be restarted
            if motor_id not in self.running_motors:
                self.motor_manager.StartServo(motor_id)
            
            # Set the speed
            self.running_motors[motor_id] = speed_raw
            self.motor_manager.Rotate(motor_id, speed_raw)
    
    def send_position_command(self, motor_id: int, pos_ticks: int, speed: int, accel: int):
        """Send position command to a motor"""
        self.motor_manager.MoveTo(motor_id, pos_ticks, speed, accel)
    
    def read_motor_state(self, motor_id: int):
        """Read motor position and velocity"""
        try:
            pos = self.motor_manager.ReadPosition(motor_id)
            speed_data = self.motor_manager.ReadSpeed(motor_id)
            
            if pos is not None and speed_data is not None:
                speed = speed_data[0] if isinstance(speed_data, tuple) else speed_data
                return pos, speed
        except Exception:
            pass
        
        return None, None