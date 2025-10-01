#!/usr/bin/env python3
"""
Arm Calibration Tool for LeRemix
Calibrates arm joint zero positions and limits
"""

import argparse
import sys
import time
import os
from st3215 import ST3215
import yaml


class ArmCalibrator:
    """Manages arm calibration process"""

    def __init__(self, port: str = "/dev/ttyUSB0", baud: int = 1000000, config_path: str = None):
        self.port = port
        self.baud = baud
        self.servo = None
        self.config_path = config_path

        # Joint names for LeRemix arm (IDs 4-9)
        self.joint_names = [
            "Joint 1 (Base Rotation)",
            "Joint 2 (Shoulder)",
            "Joint 3 (Elbow)",
            "Joint 4 (Wrist Pitch)",
            "Joint 5 (Wrist Roll)",
            "Joint 6 (Gripper)"
        ]

        # Calibration data
        self.calibration = {
            'arm_ids': [4, 5, 6, 7, 8, 9],
            'arm_zero_positions': [],
            'arm_min_limits': [],
            'arm_max_limits': [],
            'ticks_per_rev': 4096
        }

    def connect(self):
        """Connect to the servo bus"""
        try:
            self.servo = ST3215(self.port)
            print(f"✅ Connected to serial port: {self.port}")
            return True
        except Exception as e:
            print(f"❌ Failed to connect to {self.port}: {e}")
            print("\nTroubleshooting:")
            print("  - Check that the device is connected and powered")
            print("  - Verify serial port permissions: sudo usermod -a -G dialout $USER")
            print("  - Ensure no other ROS nodes are using the port")
            return False

    def check_arm_servos(self):
        """Verify all arm servos are connected"""
        print("\n🔍 Checking arm servo connectivity...\n")

        missing = []
        for i, servo_id in enumerate(self.calibration['arm_ids']):
            if self.servo.PingServo(servo_id):
                print(f"  ID {servo_id} ({self.joint_names[i]}): ✅ ONLINE")
            else:
                print(f"  ID {servo_id} ({self.joint_names[i]}): ❌ OFFLINE")
                missing.append(servo_id)

        if missing:
            print(f"\n❌ Missing servos: {missing}")
            print("Please check connections and power before calibrating.")
            return False

        print("\n✅ All arm servos detected!")
        return True

    def enable_free_mode(self, servo_id):
        """Put servo in free-moving mode (torque disabled)"""
        try:
            self.servo.SetMode(servo_id, 0)  # Position mode
            self.servo.StopServo(servo_id)  # Disable torque - allows free movement
            return True
        except Exception as e:
            print(f"❌ Failed to enable free mode for servo {servo_id}: {e}")
            return False

    def read_position(self, servo_id):
        """Read current servo position"""
        try:
            pos = self.servo.ReadPosition(servo_id)
            return pos
        except Exception as e:
            print(f"❌ Failed to read position from servo {servo_id}: {e}")
            return None

    def lock_servo(self, servo_id):
        """Lock servo at current position"""
        try:
            self.servo.StartServo(servo_id)  # Enable torque
            return True
        except Exception as e:
            print(f"❌ Failed to lock servo {servo_id}: {e}")
            return False

    def calibrate_zero_position(self):
        """Calibrate zero position with arm fully erect"""
        print("\n" + "="*70)
        print("  STEP 1: Zero Position Calibration")
        print("="*70)
        print("\n📐 Move the robot arm to ZERO POSITION:")
        print("   - All joints fully erect (vertical)")
        print("   - Gripper fully closed")
        print("   - Wrist rotation centered")
        print("\nThis position will be set as the home/reference position.\n")

        # Enable free mode for all arm servos
        print("🔓 Enabling free movement mode for all arm joints...")
        for servo_id in self.calibration['arm_ids']:
            self.enable_free_mode(servo_id)

        input("\n1️⃣  Manually position the arm as described above, then press ENTER...")

        # Read and save zero positions
        print("\n📊 Reading zero positions...")
        self.calibration['arm_zero_positions'] = []

        for i, servo_id in enumerate(self.calibration['arm_ids']):
            pos = self.read_position(servo_id)
            if pos is not None:
                self.calibration['arm_zero_positions'].append(pos)
                print(f"   {self.joint_names[i]} (ID {servo_id}): {pos} ticks")
            else:
                print(f"   {self.joint_names[i]} (ID {servo_id}): ❌ Read failed!")
                return False

        # Lock servos at zero position
        print("\n🔒 Locking servos at zero position...")
        for servo_id in self.calibration['arm_ids']:
            self.lock_servo(servo_id)

        print("\n✅ Zero position calibration complete!")
        return True

    def calibrate_joint_limits(self):
        """Calibrate min/max limits for each joint"""
        print("\n" + "="*70)
        print("  STEP 2: Joint Limit Calibration")
        print("="*70)
        print("\nYou will now calibrate the movement limits for each joint.")
        print("For each joint, you'll move it to its minimum and maximum positions.\n")

        self.calibration['arm_min_limits'] = []
        self.calibration['arm_max_limits'] = []

        for i, servo_id in enumerate(self.calibration['arm_ids']):
            print("\n" + "-"*70)
            print(f"  Calibrating: {self.joint_names[i]} (ID {servo_id})")
            print("-"*70)

            # Enable free mode
            self.enable_free_mode(servo_id)

            # Calibrate minimum limit
            input(f"\n2️⃣  Move {self.joint_names[i]} to its MINIMUM position, then press ENTER...")
            min_pos = self.read_position(servo_id)
            if min_pos is None:
                print("❌ Failed to read minimum position!")
                return False
            print(f"   Minimum position: {min_pos} ticks")

            # Calibrate maximum limit
            input(f"\n3️⃣  Move {self.joint_names[i]} to its MAXIMUM position, then press ENTER...")
            max_pos = self.read_position(servo_id)
            if max_pos is None:
                print("❌ Failed to read maximum position!")
                return False
            print(f"   Maximum position: {max_pos} ticks")

            # Verify range
            range_ticks = abs(max_pos - min_pos)
            print(f"   Range: {range_ticks} ticks ({range_ticks/self.calibration['ticks_per_rev']*360:.1f}°)")

            # Save limits (ensure min < max)
            if min_pos <= max_pos:
                self.calibration['arm_min_limits'].append(min_pos)
                self.calibration['arm_max_limits'].append(max_pos)
            else:
                self.calibration['arm_min_limits'].append(max_pos)
                self.calibration['arm_max_limits'].append(min_pos)
                print("   ⚠️  Swapped min/max to ensure min < max")

            # Lock servo
            self.lock_servo(servo_id)
            print(f"   ✅ {self.joint_names[i]} calibrated")

        print("\n✅ All joint limits calibrated!")
        return True

    def return_to_zero(self):
        """Move all joints back to zero position"""
        print("\n📍 Returning arm to zero position...")

        for i, servo_id in enumerate(self.calibration['arm_ids']):
            try:
                zero_pos = self.calibration['arm_zero_positions'][i]
                self.servo.SetMode(servo_id, 0)
                self.servo.StartServo(servo_id)
                self.servo.MoveTo(servo_id, zero_pos, 1000, 200)
                print(f"   {self.joint_names[i]}: Moving to {zero_pos}")
            except Exception as e:
                print(f"   ❌ Failed to move {self.joint_names[i]}: {e}")

        time.sleep(2)
        print("✅ Return to zero complete")

    def display_calibration_summary(self):
        """Display calibration results"""
        print("\n" + "="*70)
        print("  CALIBRATION SUMMARY")
        print("="*70)
        print("\n┌─────────────────────────┬──────┬──────────┬─────────┬─────────┐")
        print("│ Joint                   │  ID  │ Zero Pos │ Min Pos │ Max Pos │")
        print("├─────────────────────────┼──────┼──────────┼─────────┼─────────┤")

        for i in range(len(self.calibration['arm_ids'])):
            joint = self.joint_names[i]
            servo_id = self.calibration['arm_ids'][i]
            zero = self.calibration['arm_zero_positions'][i]
            min_pos = self.calibration['arm_min_limits'][i]
            max_pos = self.calibration['arm_max_limits'][i]

            print(f"│ {joint:23s} │  {servo_id:2d}  │  {zero:4d}    │  {min_pos:4d}   │  {max_pos:4d}   │")

        print("└─────────────────────────┴──────┴──────────┴─────────┴─────────┘")

    def save_calibration(self):
        """Save calibration to YAML config file"""
        if not self.config_path:
            print("\n⚠️  No config path specified, skipping save.")
            return False

        # Check if config file exists
        if not os.path.exists(self.config_path):
            print(f"\n❌ Config file not found: {self.config_path}")
            return False

        try:
            # Load existing config
            with open(self.config_path, 'r') as f:
                config = yaml.safe_load(f)

            # Update with calibration data
            if 'servo_manager_node_py' not in config:
                config['servo_manager_node_py'] = {}
            if 'ros__parameters' not in config['servo_manager_node_py']:
                config['servo_manager_node_py']['ros__parameters'] = {}

            params = config['servo_manager_node_py']['ros__parameters']

            # Add calibration data
            params['arm_zero_positions'] = self.calibration['arm_zero_positions']
            params['arm_min_limits'] = self.calibration['arm_min_limits']
            params['arm_max_limits'] = self.calibration['arm_max_limits']

            # Backup original config
            backup_path = self.config_path + '.backup'
            with open(backup_path, 'w') as f:
                yaml.dump(config, f, default_flow_style=False)
            print(f"\n💾 Backup saved to: {backup_path}")

            # Save updated config
            with open(self.config_path, 'w') as f:
                yaml.dump(config, f, default_flow_style=False, sort_keys=False)

            print(f"✅ Calibration saved to: {self.config_path}")
            return True

        except Exception as e:
            print(f"\n❌ Failed to save calibration: {e}")
            return False

    def run_calibration(self):
        """Run complete calibration process"""
        print("\n" + "="*70)
        print("  LeRemix Arm Calibration Wizard")
        print("="*70)
        print("\nThis tool will calibrate your robot arm's zero position and joint limits.")
        print("Make sure:")
        print("  ✓ Robot is powered on")
        print("  ✓ No ROS nodes are running (they may interfere with servo control)")
        print("  ✓ You can safely move the arm manually")
        print("\n" + "="*70)

        # Check connectivity
        if not self.check_arm_servos():
            return False

        # Step 1: Zero position
        if not self.calibrate_zero_position():
            print("\n❌ Zero position calibration failed!")
            return False

        # Step 2: Joint limits
        if not self.calibrate_joint_limits():
            print("\n❌ Joint limit calibration failed!")
            return False

        # Return to zero
        self.return_to_zero()

        # Display summary
        self.display_calibration_summary()

        # Save to config
        if self.config_path:
            save = input("\n💾 Save calibration to config file? (yes/no): ")
            if save.lower() == 'yes':
                self.save_calibration()

        print("\n" + "="*70)
        print("  🎉 Calibration Complete!")
        print("="*70)
        print("\nNext steps:")
        print("  1. Review the calibration data above")
        print("  2. Test the arm with ROS2 controllers")
        print("  3. Fine-tune limits if needed by re-running this script")
        print("\n")

        return True


def main():
    parser = argparse.ArgumentParser(
        description='LeRemix Arm Calibration Tool',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
This tool calibrates the arm's zero position and joint limits.

IMPORTANT: Stop all ROS2 nodes before running this script!

Examples:
  # Basic calibration (will prompt for save location)
  python3 calibrate_arm.py

  # Specify config file to update
  python3 calibrate_arm.py --config ~/leremix_ws/ros/src/leremix_servo_manager/config/servo_manager.yaml

  # Use custom serial port
  python3 calibrate_arm.py --port /dev/ttyACM0
        """
    )

    parser.add_argument(
        '--port',
        default='/dev/ttyUSB0',
        help='Serial port (default: /dev/ttyUSB0)'
    )

    parser.add_argument(
        '--baud',
        type=int,
        default=1000000,
        help='Baud rate (default: 1000000)'
    )

    parser.add_argument(
        '--config',
        help='Path to servo_manager.yaml config file to update'
    )

    args = parser.parse_args()

    # Create calibrator
    calibrator = ArmCalibrator(args.port, args.baud, args.config)

    # Connect to servos
    if not calibrator.connect():
        sys.exit(1)

    # Run calibration
    try:
        if calibrator.run_calibration():
            sys.exit(0)
        else:
            sys.exit(1)
    except KeyboardInterrupt:
        print("\n\n👋 Calibration cancelled by user")
        sys.exit(0)
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
