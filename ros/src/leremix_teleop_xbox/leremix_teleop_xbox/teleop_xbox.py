#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from .joystick_driver import JoystickDriver

class XboxTeleop(Node):
    def __init__(self):
        super().__init__('leremix_teleop_xbox')

        # Topics / rates
        self.declare_parameter('cmd_vel_topic', '/omnidirectional_controller/cmd_vel_unstamped')
        self.declare_parameter('arm_cmd_topic', '/arm_controller/commands')
        self.declare_parameter('head_cmd_topic', '/head_controller/commands')
        self.declare_parameter('linear_scale', 0.5)
        self.declare_parameter('lateral_scale', 0.3)
        self.declare_parameter('arm_increment', 0.0175)  # 1.0 degree
        self.declare_parameter('stick_deadband', 0.1)
        self.declare_parameter('trigger_threshold', 0.3)
        self.declare_parameter('arm_rate', 50.0)
        self.declare_parameter('base_rate', 50.0)
        self.declare_parameter('acceleration_limit', 2.0)

        # Separate arm and head joints with limits
        self.arm_joints = ['1','2','3','4','5','6']
        self.head_joints = ['camera_pan', 'camera_tilt']
        self.jidx_arm = {name: i for i, name in enumerate(self.arm_joints)}
        self.jidx_head = {name: i for i, name in enumerate(self.head_joints)}
        
        # Joint limits (from gazebo joint_limits.xacro)
        self.arm_limits = {
            '1': (-1.57, 1.57),     # ±90° // 
            '2': (-0.22, 3.14),     # shoulder
            '3': (-2.84, 0.0),      # elbow
            '4': (0.02, 3.32),      # wrist tilt
            '5': (-1.04, 3.14),     # wrist rotation
            '6': (0.0, 2.14)        # gripper
        }
        self.head_limits = {
            'camera_pan': (-3.14, 3.14),    # full rotation
            'camera_tilt': (-1.57, 0.86)     # tilt limits
        }

        # Load params
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.arm_cmd_topic = self.get_parameter('arm_cmd_topic').get_parameter_value().string_value
        self.head_cmd_topic = self.get_parameter('head_cmd_topic').get_parameter_value().string_value
        self.linear_scale  = float(self.get_parameter('linear_scale').value)
        self.lateral_scale = float(self.get_parameter('lateral_scale').value)
        self.arm_inc       = float(self.get_parameter('arm_increment').value)
        self.stick_deadband = float(self.get_parameter('stick_deadband').value)
        self.trigger_threshold = float(self.get_parameter('trigger_threshold').value)
        self.arm_rate      = float(self.get_parameter('arm_rate').value)
        self.base_rate     = float(self.get_parameter('base_rate').value)
        self.acceleration_limit = float(self.get_parameter('acceleration_limit').value)

        # Publishers/subscriber
        self.pub_twist = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.pub_arm   = self.create_publisher(Float64MultiArray, self.arm_cmd_topic, 10)
        self.pub_head  = self.create_publisher(Float64MultiArray, self.head_cmd_topic, 10)
        self.sub       = self.create_subscription(Joy, 'joy', self.on_joy, 10)

        # State
        self.arm_targets = [0.0] * len(self.arm_joints)
        self.head_targets = [0.0] * len(self.head_joints)
        self.joystick_driver = JoystickDriver(self.trigger_threshold)
        
        # Smooth movement state
        self.current_vel_x = 0.0
        self.current_vel_y = 0.0
        self.target_vel_x = 0.0
        self.target_vel_y = 0.0
        self.dt = 1.0 / self.base_rate

        # Timer
        self.timer_arm = self.create_timer(1.0 / self.arm_rate, self.publish_arm)
        self.timer_head = self.create_timer(1.0 / self.arm_rate, self.publish_head)
        self.timer_base = self.create_timer(1.0 / self.base_rate, self.publish_base)

        self.get_logger().info("Xbox teleop node started")
        self.get_logger().info(f"Motion topic: {self.cmd_vel_topic}, Arm topic: {self.arm_cmd_topic}, Head topic: {self.head_cmd_topic}")
        self.get_logger().info(f"Arm joints: {self.arm_joints}, Head joints: {self.head_joints}, stick deadband={self.stick_deadband}")

    def add_to_arm_joint(self, name, delta, src=""):
        if name in self.jidx_arm:
            i = self.jidx_arm[name]
            # Apply joint limits
            min_pos, max_pos = self.arm_limits[name]
            new_pos = self.arm_targets[i] + delta
            self.arm_targets[i] = max(min_pos, min(max_pos, new_pos))
            self.get_logger().info(f"Arm joint {name} += {delta:.3f} from {src}, clamped={self.arm_targets[i]:.3f}")
    
    def add_to_head_joint(self, name, delta, src=""):
        if name in self.jidx_head:
            i = self.jidx_head[name]
            # Apply joint limits
            min_pos, max_pos = self.head_limits[name]
            new_pos = self.head_targets[i] + delta
            self.head_targets[i] = max(min_pos, min(max_pos, new_pos))
            self.get_logger().info(f"Head joint {name} += {delta:.3f} from {src}, clamped={self.head_targets[i]:.3f}")

    def on_joy(self, msg: Joy):
        events = self.joystick_driver.process_joy_message(msg)

        # ----- Motion: right joystick -----
        right_stick = events.get('right_stick', {})
        rs_x = right_stick.get('x', 0.0)  # Forward/backward
        rs_y = right_stick.get('y', 0.0)  # Left/right strafe
        
        # Apply deadband and scaling
        if abs(rs_x) > self.stick_deadband:
            self.target_vel_x = rs_x * self.linear_scale
        else:
            self.target_vel_x = 0.0
            
        if abs(rs_y) > self.stick_deadband:
            self.target_vel_y = -rs_y * self.lateral_scale  # Invert Y for natural movement
        else:
            self.target_vel_y = 0.0

        # ----- Arm joints: buttons and left stick -----
        inc = self.arm_inc

        # Left Stick → joints 1 & 2
        left_stick = events.get('left_stick', {})
        ls_x = left_stick.get('x', 0.0)
        ls_y = left_stick.get('y', 0.0)
        if abs(ls_x) > self.stick_deadband:
            self.add_to_arm_joint("1", ls_x * inc, "LS_x")
        if abs(ls_y) > self.stick_deadband:
            self.add_to_arm_joint("2", ls_y * inc, "LS_y")

        # Face buttons → joints 3 & 4 (single press for small increment, long press for continuous)
        if events.get('y_press', False):
            self.add_to_arm_joint("3", inc, "Y press")
        elif events.get('y_long_press_active', False):
            self.add_to_arm_joint("3", inc, "Y long press")
            
        if events.get('a_press', False):
            self.add_to_arm_joint("3", -inc, "A press")
        elif events.get('a_long_press_active', False):
            self.add_to_arm_joint("3", -inc, "A long press")
            
        if events.get('x_press', False):
            self.add_to_arm_joint("4", -inc, "X press")
        elif events.get('x_long_press_active', False):
            self.add_to_arm_joint("4", -inc, "X long press")
            
        if events.get('b_press', False):
            self.add_to_arm_joint("4", inc, "B press")
        elif events.get('b_long_press_active', False):
            self.add_to_arm_joint("4", inc, "B long press")

        # Shoulder buttons → joint 5 (single press for small increment, long press for continuous)
        if events.get('rb_press', False):
            self.add_to_arm_joint("5", inc, "RB press")
        elif events.get('rb_long_press_active', False):
            self.add_to_arm_joint("5", inc, "RB long press")
            
        if events.get('lb_press', False):
            self.add_to_arm_joint("5", -inc, "LB press")
        elif events.get('lb_long_press_active', False):
            self.add_to_arm_joint("5", -inc, "LB long press")

        # Triggers → joint 6
        if events.get('rt_press', False):
            self.add_to_arm_joint("6", inc, "RT press")
        if events.get('lt_press', False):
            self.add_to_arm_joint("6", -inc, "LT press")

        # Camera controls: START/BACK for tilt (single press for small increment, long press for continuous)
        if events.get('start_press', False):
            self.add_to_head_joint("camera_tilt", inc, "START press")
        elif events.get('start_long_press_active', False):
            self.add_to_head_joint("camera_tilt", inc, "START long press")
            
        if events.get('back_press', False):
            self.add_to_head_joint("camera_tilt", -inc, "BACK press")
        elif events.get('back_long_press_active', False):
            self.add_to_head_joint("camera_tilt", -inc, "BACK long press")
        
        # D-pad for camera pan: left = negative, right = positive (single press for small increment, long press for continuous)
        if events.get('dpad_left_press', False):
            self.add_to_head_joint("camera_pan", -inc, "D-pad left press")
        elif events.get('dpad_left_long_press_active', False):
            self.add_to_head_joint("camera_pan", -inc, "D-pad left long press")
            
        if events.get('dpad_right_press', False):
            self.add_to_head_joint("camera_pan", inc, "D-pad right press")
        elif events.get('dpad_right_long_press_active', False):
            self.add_to_head_joint("camera_pan", inc, "D-pad right long press")

    def publish_arm(self):
        msg = Float64MultiArray()
        msg.data = self.arm_targets
        self.pub_arm.publish(msg)
    
    def publish_head(self):
        msg = Float64MultiArray()
        msg.data = self.head_targets
        self.pub_head.publish(msg)

    def publish_base(self):
        # Smooth acceleration/deceleration
        max_delta = self.acceleration_limit * self.dt
        
        # X velocity
        vel_diff_x = self.target_vel_x - self.current_vel_x
        if abs(vel_diff_x) > max_delta:
            vel_diff_x = max_delta if vel_diff_x > 0 else -max_delta
        self.current_vel_x += vel_diff_x
        
        # Y velocity
        vel_diff_y = self.target_vel_y - self.current_vel_y
        if abs(vel_diff_y) > max_delta:
            vel_diff_y = max_delta if vel_diff_y > 0 else -max_delta
        self.current_vel_y += vel_diff_y
        
        # Publish smoothed twist
        tw = Twist()
        tw.linear.x = self.current_vel_x
        tw.linear.y = self.current_vel_y
        self.pub_twist.publish(tw)

def main():
    rclpy.init()
    node = XboxTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
