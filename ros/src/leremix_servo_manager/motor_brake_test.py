#!/usr/bin/env python3
"""
Motor Brake Test Script
Tests different braking methods and measures deceleration distance

Commands:
- m1, m2, m3: Start motor 1, 2, or 3 at 50% speed
- s: Stop motor using stopInPlace (hard brake)
- d: Stop motor using velocity command (gradual)
- t: Stop motor using torque disable (coast to stop)

Measures steps traveled after brake command for each method.
"""

import time
import threading
from st3215 import ST3215

class MotorBrakeTest:
    def __init__(self, port="/dev/ttyTHS1", baud=1000000):
        self.port = port
        self.baud = baud
        self.motor_manager = None
        self.motor_ids = [1, 2, 3]
        self.running_motors = {}
        self.position_tracker = {}
        self.tracking_active = False
        
        print("🔧 Motor Brake Test Initializing...")
        print(f"   Port: {port}")
        print(f"   Baud: {baud}")
        print(f"   Motor IDs: {self.motor_ids}")
        
    def connect(self):
        """Connect to the motor bus"""
        try:
            self.motor_manager = ST3215(self.port)
            print("✅ Connected to motor bus")
            
            # Test connectivity
            for motor_id in self.motor_ids:
                if self.motor_manager.PingServo(motor_id):
                    print(f"   Motor {motor_id}: ONLINE")
                    # Initialize motor
                    self.motor_manager.SetMode(motor_id, 1)  # Velocity mode
                    self.motor_manager.StartServo(motor_id)
                    self.position_tracker[motor_id] = 0
                else:
                    print(f"   Motor {motor_id}: OFFLINE")
                    
        except Exception as e:
            print(f"❌ Failed to connect: {e}")
            return False
        return True
    
    def get_position(self, motor_id):
        """Get current motor position"""
        try:
            return self.motor_manager.ReadPosition(motor_id)
        except:
            return None
    
    def start_motor(self, motor_id, speed_percent=50):
        """Start motor at specified speed percentage"""
        if motor_id not in self.motor_ids:
            print(f"❌ Invalid motor ID: {motor_id}")
            return
            
        try:
            # Calculate speed value (ST3215 uses -1023 to +1023)
            speed_value = int((speed_percent / 100.0) * 1023)
            
            # Record starting position
            start_pos = self.get_position(motor_id)
            if start_pos is not None:
                self.position_tracker[motor_id] = start_pos
                print(f"🚀 Motor {motor_id} starting at position {start_pos}")
            
            # Set velocity mode and start
            self.motor_manager.SetMode(motor_id, 1)  # Velocity mode
            self.motor_manager.Rotate(motor_id, speed_value)
            self.running_motors[motor_id] = speed_value
            
            print(f"✅ Motor {motor_id} started at {speed_percent}% speed ({speed_value} raw)")
            
        except Exception as e:
            print(f"❌ Failed to start motor {motor_id}: {e}")
    
    def measure_brake_distance(self, motor_id, brake_method):
        """Measure how far motor travels after brake command"""
        if motor_id not in self.running_motors:
            print(f"❌ Motor {motor_id} is not running")
            return
            
        # Get position before brake
        pos_before = self.get_position(motor_id)
        if pos_before is None:
            print(f"❌ Could not read position for motor {motor_id}")
            return
            
        print(f"🛑 Applying {brake_method} brake to motor {motor_id}")
        print(f"   Position before brake: {pos_before}")
        
        # Apply brake command
        brake_start_time = time.time()
        
        try:
            if brake_method == "stopInPlace":
                # ST3215 doesn't have stopInPlace, use MoveTo current position with high acceleration
                current_pos = self.get_position(motor_id)
                if current_pos is not None:
                    self.motor_manager.MoveTo(motor_id, current_pos, speed=3000, acc=200)
            elif brake_method == "velocity":
                self.motor_manager.Rotate(motor_id, 0)  # Set velocity to 0
            elif brake_method == "torque_disable":
                self.motor_manager.StopServo(motor_id)  # Coast to stop
            else:
                print(f"❌ Unknown brake method: {brake_method}")
                return
                
        except Exception as e:
            print(f"❌ Failed to apply brake: {e}")
            return
        
        # Monitor position for 3 seconds to see how far it travels
        print("   Monitoring brake distance...")
        positions = []
        
        for i in range(30):  # 3 seconds at 10Hz
            time.sleep(0.1)
            current_pos = self.get_position(motor_id)
            if current_pos is not None:
                positions.append((time.time() - brake_start_time, current_pos))
                
        # Calculate final position and distance traveled
        if positions:
            final_time, final_pos = positions[-1]
            brake_distance = abs(final_pos - pos_before)
            
            print(f"   Final position: {final_pos}")
            print(f"   Brake distance: {brake_distance} steps")
            print(f"   Brake time: {final_time:.2f} seconds")
            
            # Show position over time
            print("   Position timeline:")
            for t, pos in positions[::3]:  # Every 3rd sample
                print(f"     {t:.1f}s: {pos} steps")
                
        else:
            print("   ❌ Could not track brake distance")
        
        # Remove from running motors
        if motor_id in self.running_motors:
            del self.running_motors[motor_id]
        
        # Re-enable torque if it was disabled
        if brake_method == "torque_disable":
            time.sleep(0.5)
            self.motor_manager.StartServo(motor_id)
            self.motor_manager.SetMode(motor_id, 1)  # Back to velocity mode
    
    def stop_motor_method_s(self):
        """Stop using stopInPlace method"""
        if not self.running_motors:
            print("❌ No motors running")
            return
            
        motor_id = list(self.running_motors.keys())[0]  # Stop first running motor
        self.measure_brake_distance(motor_id, "stopInPlace")
    
    def stop_motor_method_d(self):
        """Stop using velocity command method"""
        if not self.running_motors:
            print("❌ No motors running")
            return
            
        motor_id = list(self.running_motors.keys())[0]  # Stop first running motor
        self.measure_brake_distance(motor_id, "velocity")
    
    def stop_motor_method_t(self):
        """Stop using torque disable method"""
        if not self.running_motors:
            print("❌ No motors running")
            return
            
        motor_id = list(self.running_motors.keys())[0]  # Stop first running motor
        self.measure_brake_distance(motor_id, "torque_disable")
    
    def print_status(self):
        """Print current motor status"""
        print("\n📊 Motor Status:")
        for motor_id in self.motor_ids:
            if motor_id in self.running_motors:
                speed = self.running_motors[motor_id]
                pos = self.get_position(motor_id)
                print(f"   Motor {motor_id}: RUNNING at speed {speed} (pos: {pos})")
            else:
                pos = self.get_position(motor_id)
                print(f"   Motor {motor_id}: STOPPED (pos: {pos})")
        print()
    
    def run_cli(self):
        """Run the command line interface"""
        print("\n🎮 Motor Brake Test Ready!")
        print("Commands:")
        print("  m1, m2, m3 - Start motor 1, 2, or 3 at 50% speed")
        print("  s - Stop motor using stopInPlace (hard brake)")
        print("  d - Stop motor using velocity command (gradual)")
        print("  t - Stop motor using torque disable (coast)")
        print("  status - Show motor status")
        print("  quit - Exit")
        print()
        
        while True:
            try:
                cmd = input("Command: ").strip().lower()
                
                if cmd == "quit":
                    break
                elif cmd == "m1":
                    self.start_motor(1)
                elif cmd == "m2":
                    self.start_motor(2)
                elif cmd == "m3":
                    self.start_motor(3)
                elif cmd == "s":
                    self.stop_motor_method_s()
                elif cmd == "d":
                    self.stop_motor_method_d()
                elif cmd == "t":
                    self.stop_motor_method_t()
                elif cmd == "status":
                    self.print_status()
                elif cmd == "":
                    continue
                else:
                    print(f"❌ Unknown command: {cmd}")
                    
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"❌ Error: {e}")
        
        # Stop all motors before exit
        print("\n🛑 Stopping all motors...")
        for motor_id in list(self.running_motors.keys()):
            try:
                self.motor_manager.Rotate(motor_id, 0)
            except:
                pass

def main():
    # Use same port as servo manager
    test = MotorBrakeTest("/dev/ttyTHS1", 1000000)
    
    if test.connect():
        test.run_cli()
    else:
        print("❌ Failed to initialize motor test")

if __name__ == "__main__":
    main()