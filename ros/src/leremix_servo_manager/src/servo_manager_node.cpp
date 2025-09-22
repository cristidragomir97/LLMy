#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <thread>
#include <chrono>
#include <map>
#include <vector>
#include <cmath>
#include "MotorManager.h"

class ServoManagerNode : public rclcpp::Node
{
public:
    ServoManagerNode() : Node("servo_manager_node")
    {
        RCLCPP_INFO(get_logger(), "=== LeRemix Servo Manager Node Starting ===");
        
        // Parameters from config file
        declare_parameter("port", "/dev/ttyUSB0");
        declare_parameter("baud", 2000000);
        declare_parameter("loc_enable", true);
        declare_parameter("arm_enable", true);
        declare_parameter("head_enable", true);
        declare_parameter("loc_ids", std::vector<long>{1, 2, 3});
        declare_parameter("arm_ids", std::vector<long>{4, 5, 6, 7, 8, 9});
        declare_parameter("head_ids", std::vector<long>{10, 11});
        declare_parameter("ticks_per_rev", 4096);
        declare_parameter("telemetry_rate", 50.0);
        declare_parameter("loc_accel", std::vector<long>{50, 50, 50});
        declare_parameter("arm_accel", std::vector<long>{20, 20, 15, 15, 25, 25});
        declare_parameter("head_accel", std::vector<long>{30, 30});
        declare_parameter("loc_speed_scale", 1.0);
        declare_parameter("arm_speed_scale", 1.0);
        declare_parameter("head_speed_scale", 1.0);
        
        RCLCPP_INFO(get_logger(), "Loading servo manager configuration...");

        auto port = get_parameter("port").as_string();
        auto baud = get_parameter("baud").as_int();
        loc_enable_ = get_parameter("loc_enable").as_bool();
        arm_enable_ = get_parameter("arm_enable").as_bool();
        head_enable_ = get_parameter("head_enable").as_bool();
        auto loc_ids_param = get_parameter("loc_ids").as_integer_array();
        auto arm_ids_param = get_parameter("arm_ids").as_integer_array();
        auto head_ids_param = get_parameter("head_ids").as_integer_array();
        auto loc_accel_param = get_parameter("loc_accel").as_integer_array();
        auto arm_accel_param = get_parameter("arm_accel").as_integer_array();
        auto head_accel_param = get_parameter("head_accel").as_integer_array();
        ticks_per_rev_ = get_parameter("ticks_per_rev").as_int();
        auto rate = get_parameter("telemetry_rate").as_double();
        loc_speed_scale_ = get_parameter("loc_speed_scale").as_double();
        arm_speed_scale_ = get_parameter("arm_speed_scale").as_double();
        head_speed_scale_ = get_parameter("head_speed_scale").as_double();

        // Convert to uint8_t vectors
        for (auto id : loc_ids_param) loc_ids_.push_back(static_cast<uint8_t>(id));
        for (auto id : arm_ids_param) arm_ids_.push_back(static_cast<uint8_t>(id));
        for (auto id : head_ids_param) head_ids_.push_back(static_cast<uint8_t>(id));
        for (auto accel : loc_accel_param) loc_accel_.push_back(static_cast<uint8_t>(accel));
        for (auto accel : arm_accel_param) arm_accel_.push_back(static_cast<uint8_t>(accel));
        for (auto accel : head_accel_param) head_accel_.push_back(static_cast<uint8_t>(accel));
        
        // Log configuration
        RCLCPP_INFO(get_logger(), "Configuration loaded:");
        RCLCPP_INFO(get_logger(), "  Serial Port: %s", port.c_str());
        RCLCPP_INFO(get_logger(), "  Baud Rate: %ld", baud);
        RCLCPP_INFO(get_logger(), "  Ticks per Revolution: %d", ticks_per_rev_);
        RCLCPP_INFO(get_logger(), "  Telemetry Rate: %.1f Hz", rate);
        RCLCPP_INFO(get_logger(), "  Locomotion Enabled: %s (Speed: %.1f%%)", loc_enable_ ? "YES" : "NO", loc_speed_scale_ * 100.0);
        RCLCPP_INFO(get_logger(), "  Arm Enabled: %s (Speed: %.1f%%)", arm_enable_ ? "YES" : "NO", arm_speed_scale_ * 100.0);
        RCLCPP_INFO(get_logger(), "  Head Enabled: %s (Speed: %.1f%%)", head_enable_ ? "YES" : "NO", head_speed_scale_ * 100.0);
        
        // Log motor configuration
        RCLCPP_INFO(get_logger(), "Motor Configuration:");
        std::string loc_str = "  Locomotion IDs: [";
        for (size_t i = 0; i < loc_ids_.size(); ++i) {
            loc_str += std::to_string(loc_ids_[i]);
            if (i < loc_ids_.size() - 1) loc_str += ", ";
        }
        loc_str += "]";
        RCLCPP_INFO(get_logger(), "%s", loc_str.c_str());
        
        std::string arm_str = "  Arm IDs: [";
        for (size_t i = 0; i < arm_ids_.size(); ++i) {
            arm_str += std::to_string(arm_ids_[i]);
            if (i < arm_ids_.size() - 1) arm_str += ", ";
        }
        arm_str += "]";
        RCLCPP_INFO(get_logger(), "%s", arm_str.c_str());
        
        std::string head_str = "  Head IDs: [";
        for (size_t i = 0; i < head_ids_.size(); ++i) {
            head_str += std::to_string(head_ids_[i]);
            if (i < head_ids_.size() - 1) head_str += ", ";
        }
        head_str += "]";
        RCLCPP_INFO(get_logger(), "%s", head_str.c_str());

        RCLCPP_INFO(get_logger(), "Attempting to connect to FEETECH servo bus...");
        try {
            motor_manager_ = std::make_unique<MotorManager>(port, baud);
            RCLCPP_INFO(get_logger(), "✅ Successfully connected to serial port: %s at %ld baud", port.c_str(), baud);
            
            // Test communication with enabled motors
            RCLCPP_INFO(get_logger(), "Testing enabled motor connectivity...");
            std::vector<uint8_t> enabled_ids;
            if (loc_enable_) enabled_ids.insert(enabled_ids.end(), loc_ids_.begin(), loc_ids_.end());
            if (arm_enable_) enabled_ids.insert(enabled_ids.end(), arm_ids_.begin(), arm_ids_.end());
            if (head_enable_) enabled_ids.insert(enabled_ids.end(), head_ids_.begin(), head_ids_.end());
            
            if (enabled_ids.empty()) {
                RCLCPP_WARN(get_logger(), "No motor groups enabled - skipping connectivity test");
            } else {
                int connected_motors = 0;
                for (auto id : enabled_ids) {
                    if (motor_manager_->ping(id)) {
                        connected_motors++;
                        RCLCPP_INFO(get_logger(), "  Motor ID %d: ✅ ONLINE", id);
                    } else {
                        RCLCPP_WARN(get_logger(), "  Motor ID %d: ❌ OFFLINE", id);
                    }
                }
                RCLCPP_INFO(get_logger(), "Motor connectivity test complete: %d/%zu enabled motors online", 
                           connected_motors, enabled_ids.size());
            }

            // Initialize motor modes and positions
            RCLCPP_INFO(get_logger(), "Initializing motor modes and positions...");
            initializeMotors();
                       
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "❌ Failed to open serial port: %s", e.what());
            RCLCPP_ERROR(get_logger(), "Please check:");
            RCLCPP_ERROR(get_logger(), "  - Serial port exists and has correct permissions");
            RCLCPP_ERROR(get_logger(), "  - Device is connected and powered on");
            RCLCPP_ERROR(get_logger(), "  - No other processes are using the port");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(get_logger(), "Setting up ROS2 communication...");
        
        // Create subscribers for commands (using best_effort QoS to match control plugin)
        RCLCPP_INFO(get_logger(), "Creating command subscribers:");
        auto cmd_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        
        base_cmd_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/motor_manager/base_cmd", cmd_qos,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) { 
                handle_base_command(msg); 
            });
        RCLCPP_INFO(get_logger(), "  ✅ Base command subscriber: /motor_manager/base_cmd");

        arm_cmd_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/motor_manager/arm_cmd", cmd_qos,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) { 
                handle_arm_command(msg); 
            });
        RCLCPP_INFO(get_logger(), "  ✅ Arm command subscriber: /motor_manager/arm_cmd");

        head_cmd_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/motor_manager/head_cmd", cmd_qos,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) { 
                handle_head_command(msg); 
            });
        RCLCPP_INFO(get_logger(), "  ✅ Head command subscriber: /motor_manager/head_cmd");

        // Create publisher for state feedback
        state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/motor_manager/joint_states", 10);
        RCLCPP_INFO(get_logger(), "  ✅ Joint state publisher: /motor_manager/joint_states");

        // Create timer for telemetry
        timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
            [this]() { publish_telemetry(); });
        RCLCPP_INFO(get_logger(), "  ✅ Telemetry timer started at %.1f Hz", rate);
        

        RCLCPP_INFO(get_logger(), "🚀 === Servo Manager Node Successfully Started ===");
        RCLCPP_INFO(get_logger(), "📊 System Summary:");
        RCLCPP_INFO(get_logger(), "  🏃 Locomotion motors: %zu (%s)", loc_ids_.size(), loc_enable_ ? "ENABLED" : "DISABLED");
        RCLCPP_INFO(get_logger(), "  🦾 Arm motors: %zu (%s)", arm_ids_.size(), arm_enable_ ? "ENABLED" : "DISABLED");
        RCLCPP_INFO(get_logger(), "  🗣️  Head motors: %zu (%s)", head_ids_.size(), head_enable_ ? "ENABLED" : "DISABLED");
        size_t enabled_count = (loc_enable_ ? loc_ids_.size() : 0) + (arm_enable_ ? arm_ids_.size() : 0) + (head_enable_ ? head_ids_.size() : 0);
        RCLCPP_INFO(get_logger(), "  📡 Total motors enabled: %zu/%zu", enabled_count, loc_ids_.size() + arm_ids_.size() + head_ids_.size());
        RCLCPP_INFO(get_logger(), "----------------------------------");
        RCLCPP_INFO(get_logger(), "Ready to receive motor commands and publish telemetry!");
        RCLCPP_INFO(get_logger(), "Monitor topics: ros2 topic list | grep motor_manager");
    }

private:
    void handle_base_command(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (!loc_enable_) {
            return;
        }
        
        if (msg->data.size() != loc_ids_.size()) {
            RCLCPP_WARN(get_logger(), "Base command size mismatch: expected %zu, got %zu", 
                       loc_ids_.size(), msg->data.size());
            return;
        }

        // Check if commands have changed - avoid redundant motor updates
        if (last_base_commands_.size() == msg->data.size()) {
            bool commands_changed = false;
            for (size_t i = 0; i < msg->data.size(); ++i) {
                if (std::abs(msg->data[i] - last_base_commands_[i]) > 1e-6) {
                    commands_changed = true;
                    break;
                }
            }
            if (!commands_changed) {
                return;
            }
        }
        
        // Store current commands for next comparison
        last_base_commands_ = msg->data;

        // Convert velocities to motor speeds
        std::vector<int16_t> speeds;
        for (double rad_per_sec : msg->data) {
            double scaled_rad_per_sec = rad_per_sec * loc_speed_scale_;
            double max_rad_per_sec = 10.0;
            int16_t speed_raw = static_cast<int16_t>(std::clamp(scaled_rad_per_sec / max_rad_per_sec * 3400.0, -3400.0, 3400.0));
            speeds.push_back(speed_raw);
        }
        
        // Handle torque control per motor based on individual speed
        const double speed_threshold = 0.001; // Small threshold for near-zero speeds
        for (size_t i = 0; i < loc_ids_.size() && i < msg->data.size(); ++i) {
            uint8_t id = loc_ids_[i];
            bool should_have_torque = std::abs(msg->data[i]) > speed_threshold;
            
            if (should_have_torque) {
                // Enable torque for this motor
                if (!motor_manager_->setTorque(id, true)) {
                    RCLCPP_WARN(get_logger(), "Failed to enable torque for motor %d", id);
                }
            } else {
                // Set velocity to zero before disabling torque
                if (!motor_manager_->setVelocity(id, 0)) {
                    RCLCPP_WARN(get_logger(), "Failed to set zero velocity for motor %d", id);
                }
                // Disable torque for this motor
                if (!motor_manager_->setTorque(id, false)) {
                    RCLCPP_WARN(get_logger(), "Failed to disable torque for motor %d", id);
                }
            }
        }
        
        // Send velocity commands only to motors with torque enabled
        if (!motor_manager_->syncWriteVelocity(loc_ids_, speeds)) {
            RCLCPP_ERROR(get_logger(), "Failed to send velocity commands");
        }
    }

    void handle_arm_command(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (!arm_enable_) {
            return;
        }
        
        if (msg->data.size() != arm_ids_.size()) {
            RCLCPP_WARN(get_logger(), "Arm command size mismatch: expected %zu, got %zu", 
                       arm_ids_.size(), msg->data.size());
            return;
        }

        // Check if commands have changed - avoid redundant motor updates
        if (last_arm_commands_.size() == msg->data.size()) {
            bool commands_changed = false;
            for (size_t i = 0; i < msg->data.size(); ++i) {
                if (std::abs(msg->data[i] - last_arm_commands_[i]) > 1e-6) {
                    commands_changed = true;
                    break;
                }
            }
            if (!commands_changed) {
                return;
            }
        }
        
        // Store current commands for next comparison
        last_arm_commands_ = msg->data;

        std::vector<uint16_t> positions, times, speeds;
        for (size_t i = 0; i < msg->data.size(); ++i) {
            // Convert from ROS radians to servo ticks, handling negative angles and centering
            double angle_normalized = msg->data[i]; // ROS angle in radians
            double servo_center = 2048; // Center position (adjust as needed)
            int32_t pos_ticks_signed = static_cast<int32_t>(servo_center + (angle_normalized / (2.0 * M_PI)) * ticks_per_rev_);
            uint16_t pos_ticks = static_cast<uint16_t>(std::max(0, std::min(65535, pos_ticks_signed)));
            positions.push_back(pos_ticks);
            times.push_back(200);  // 200ms move time
            speeds.push_back(500); // Default speed
        }
        
        if (!motor_manager_->syncWritePositionTimeSpeed(arm_ids_, positions, times, speeds)) {
            RCLCPP_ERROR(get_logger(), "Failed to send arm position commands");
        }
    }

    void handle_head_command(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (!head_enable_) {
            return;
        }
        
        if (msg->data.size() != head_ids_.size()) {
            RCLCPP_WARN(get_logger(), "Head command size mismatch: expected %zu, got %zu", 
                       head_ids_.size(), msg->data.size());
            return;
        }

        // Check if commands have changed - avoid redundant motor updates
        if (last_head_commands_.size() == msg->data.size()) {
            bool commands_changed = false;
            for (size_t i = 0; i < msg->data.size(); ++i) {
                if (std::abs(msg->data[i] - last_head_commands_[i]) > 1e-6) {
                    commands_changed = true;
                    break;
                }
            }
            if (!commands_changed) {
                return;
            }
        }
        
        // Store current commands for next comparison
        last_head_commands_ = msg->data;

        std::vector<uint16_t> positions, times, speeds;
        for (size_t i = 0; i < msg->data.size(); ++i) {
            // Convert from ROS radians to servo ticks, handling negative angles and centering
            double angle_normalized = msg->data[i]; // ROS angle in radians
            double servo_center = 2048; // Center position (adjust as needed)
            int32_t pos_ticks_signed = static_cast<int32_t>(servo_center + (angle_normalized / (2.0 * M_PI)) * ticks_per_rev_);
            uint16_t pos_ticks = static_cast<uint16_t>(std::max(0, std::min(65535, pos_ticks_signed)));
            positions.push_back(pos_ticks);
            times.push_back(100);  // 100ms move time for head (faster)
            speeds.push_back(800); // Faster speed for head
        }
        
        if (!motor_manager_->syncWritePositionTimeSpeed(head_ids_, positions, times, speeds)) {
            RCLCPP_ERROR(get_logger(), "Failed to send head position commands");
        }
    }

    void publish_telemetry()
    {
        static int telemetry_count = 0;
        static int offline_motor_warnings = 0;
        static int telemetry_cycle = 0;
        
        auto msg = std::make_shared<sensor_msgs::msg::JointState>();
        msg->header.stamp = now();

        // Read motors in smaller batches to avoid blocking
        // Alternate between different motor groups each cycle to reduce load
        std::vector<uint8_t> motors_to_read;
        
        // Build list of enabled motor groups
        std::vector<std::vector<uint8_t>*> enabled_groups;
        if (loc_enable_) enabled_groups.push_back(&loc_ids_);
        if (arm_enable_) enabled_groups.push_back(&arm_ids_);
        if (head_enable_) enabled_groups.push_back(&head_ids_);
        
        if (enabled_groups.empty()) {
            RCLCPP_DEBUG(get_logger(), "No motor groups enabled, skipping telemetry");
            return;
        }
        
        telemetry_cycle = (telemetry_cycle + 1) % enabled_groups.size();
        motors_to_read = *enabled_groups[telemetry_cycle];

        int successful_reads = 0;
        for (auto id : motors_to_read) {
            auto pos = motor_manager_->readPresentPosition(id);
            auto speed = motor_manager_->readPresentSpeed(id);

            if (pos && speed) {
                msg->name.push_back("motor_" + std::to_string(id));
                
                // Convert servo ticks back to ROS radians (inverse of command conversion)
                double servo_center = 2048; // Same center as used in commands
                double pos_rad = (static_cast<double>(*pos) - servo_center) / ticks_per_rev_ * 2.0 * M_PI;
                msg->position.push_back(pos_rad);
                
                // Convert raw speed to rad/s - use same scaling as command conversion
                double max_rad_per_sec = 10.0;
                double vel_rad_s = static_cast<double>(*speed) / 3400.0 * max_rad_per_sec;
                msg->velocity.push_back(vel_rad_s);
                successful_reads++;
            } else {
                // Only warn occasionally about offline motors to avoid spam
                if (offline_motor_warnings % 250 == 0) { // Every ~5 seconds at 50Hz
                    RCLCPP_WARN(get_logger(), "Motor ID %d not responding to telemetry request", id);
                }
                offline_motor_warnings++;
            }
        }

        // Only publish if we have data
        if (successful_reads > 0) {
            state_pub_->publish(*msg);
        }
        
        telemetry_count++;
        
        // Periodic status logging (every 10 seconds at 50Hz)
        if (telemetry_count % 500 == 0) {
            RCLCPP_INFO(get_logger(), "📊 Telemetry Status: %d motors responding in this cycle (%d updates sent)", 
                       successful_reads, telemetry_count);
        }
        
        // Debug logging for first few telemetry messages
        if (telemetry_count <= 5) {
            RCLCPP_INFO(get_logger(), "📡 Telemetry update %d: %d motors responding in this cycle", 
                       telemetry_count, successful_reads);
        }
    }

    void initializeMotors()
    {
        // Set locomotion motors to velocity mode with acceleration settings
        if (loc_enable_) {
            RCLCPP_INFO(get_logger(), "Setting locomotion motors to velocity mode with acceleration settings...");
        } else {
            RCLCPP_INFO(get_logger(), "Locomotion motors disabled - skipping initialization");
        }
        if (loc_enable_) {
            for (size_t i = 0; i < loc_ids_.size(); ++i) {
                uint8_t id = loc_ids_[i];
                uint8_t accel = (i < loc_accel_.size()) ? loc_accel_[i] : 50; // Default to 50 if not specified
                
                if (!motor_manager_->setMode(id, 1)) { // Mode 1 = velocity mode
                    RCLCPP_WARN(get_logger(), "Failed to set velocity mode for locomotion motor %d", id);
                }
                if (!motor_manager_->setAcceleration(id, accel)) {
                    RCLCPP_WARN(get_logger(), "Failed to set acceleration (%d) for locomotion motor %d", accel, id);
                } else {
                    RCLCPP_INFO(get_logger(), "Set acceleration %d for locomotion motor %d", accel, id);
                }
                // Start with torque disabled
                if (!motor_manager_->setTorque(id, false)) {
                    RCLCPP_WARN(get_logger(), "Failed to disable torque for locomotion motor %d", id);
                }
            }
        }

        // Set arm and head motors to position mode and move to URDF-defined positions
        if (arm_enable_ || head_enable_) {
            RCLCPP_INFO(get_logger(), "Setting enabled arm and head motors to position mode and moving to home positions...");
        }
        
        // Define home positions based on URDF joint limits and typical "relaxed" positions
        std::vector<double> arm_home_positions = {0.0, 0.0, -1.5, 1.5, 0.0, 1.0}; // radians
        std::vector<double> head_home_positions = {0.0, 0.0}; // radians
        
        // Initialize arm motors with acceleration settings
        if (arm_enable_) {
            RCLCPP_INFO(get_logger(), "Initializing arm motors...");
        } else {
            RCLCPP_INFO(get_logger(), "Arm motors disabled - skipping initialization");
        }
        if (arm_enable_) {
            for (size_t i = 0; i < arm_ids_.size() && i < arm_home_positions.size(); ++i) {
            uint8_t id = arm_ids_[i];
            uint8_t accel = (i < arm_accel_.size()) ? arm_accel_[i] : 20; // Default to 20 if not specified
            
            if (!motor_manager_->setMode(id, 0)) { // Mode 0 = position mode
                RCLCPP_WARN(get_logger(), "Failed to set position mode for arm motor %d", id);
            }
            if (!motor_manager_->setAcceleration(id, accel)) {
                RCLCPP_WARN(get_logger(), "Failed to set acceleration (%d) for arm motor %d", accel, id);
            } else {
                RCLCPP_INFO(get_logger(), "Set acceleration %d for arm motor %d", accel, id);
            }
            if (!motor_manager_->setTorque(id, true)) {
                RCLCPP_WARN(get_logger(), "Failed to enable torque for arm motor %d", id);
            }
            
            // Convert radians to servo ticks and move to home position
            double angle_rad = arm_home_positions[i];
            double servo_center = 2048;
            int32_t pos_ticks_signed = static_cast<int32_t>(servo_center + (angle_rad / (2.0 * M_PI)) * ticks_per_rev_);
            uint16_t pos_ticks = static_cast<uint16_t>(std::max(0, std::min(4095, pos_ticks_signed)));
            
            if (!motor_manager_->moveTo(id, pos_ticks, 1000, 300)) { // 1 second move time, moderate speed
                RCLCPP_WARN(get_logger(), "Failed to move arm motor %d to home position", id);
            } else {
                RCLCPP_INFO(get_logger(), "Moving arm motor %d to home position (%.2f rad)", id, angle_rad);
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Small delay between commands
            }
        }
        
        // Initialize head motors with acceleration settings
        if (head_enable_) {
            RCLCPP_INFO(get_logger(), "Initializing head motors...");
        } else {
            RCLCPP_INFO(get_logger(), "Head motors disabled - skipping initialization");
        }
        if (head_enable_) {
            for (size_t i = 0; i < head_ids_.size() && i < head_home_positions.size(); ++i) {
            uint8_t id = head_ids_[i];
            uint8_t accel = (i < head_accel_.size()) ? head_accel_[i] : 30; // Default to 30 if not specified
            
            if (!motor_manager_->setMode(id, 0)) { // Mode 0 = position mode
                RCLCPP_WARN(get_logger(), "Failed to set position mode for head motor %d", id);
            }
            if (!motor_manager_->setAcceleration(id, accel)) {
                RCLCPP_WARN(get_logger(), "Failed to set acceleration (%d) for head motor %d", accel, id);
            } else {
                RCLCPP_INFO(get_logger(), "Set acceleration %d for head motor %d", accel, id);
            }
            if (!motor_manager_->setTorque(id, true)) {
                RCLCPP_WARN(get_logger(), "Failed to enable torque for head motor %d", id);
            }
            
            // Convert radians to servo ticks and move to home position
            double angle_rad = head_home_positions[i];
            double servo_center = 2048;
            int32_t pos_ticks_signed = static_cast<int32_t>(servo_center + (angle_rad / (2.0 * M_PI)) * ticks_per_rev_);
            uint16_t pos_ticks = static_cast<uint16_t>(std::max(0, std::min(4095, pos_ticks_signed)));
            
            if (!motor_manager_->moveTo(id, pos_ticks, 500, 400)) { // 0.5 second move time, faster for head
                RCLCPP_WARN(get_logger(), "Failed to move head motor %d to home position", id);
            } else {
                RCLCPP_INFO(get_logger(), "Moving head motor %d to home position (%.2f rad)", id, angle_rad);
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Small delay between commands
            }
        }
        
        RCLCPP_INFO(get_logger(), "Motor initialization complete!");
    }
    

    std::unique_ptr<MotorManager> motor_manager_;
    std::vector<uint8_t> loc_ids_, arm_ids_, head_ids_;
    std::vector<uint8_t> loc_accel_, arm_accel_, head_accel_;
    int ticks_per_rev_;
    bool loc_enable_, arm_enable_, head_enable_;
    double loc_speed_scale_, arm_speed_scale_, head_speed_scale_;
    
    // Command change detection
    std::vector<double> last_base_commands_;
    std::vector<double> last_arm_commands_;
    std::vector<double> last_head_commands_;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr base_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr arm_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr head_cmd_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServoManagerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}