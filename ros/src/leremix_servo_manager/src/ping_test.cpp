#include <iostream>
#include <thread>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "MotorManager.h"

class MotorPingTest : public rclcpp::Node
{
public:
    MotorPingTest() : Node("motor_ping_test")
    {
        declare_parameter("port", "/dev/ttyTHS0");
        declare_parameter("baud", 1000000);
        declare_parameter("loc_ids", std::vector<long>{1, 2, 3});
        declare_parameter("arm_ids", std::vector<long>{4, 5, 6, 7, 8, 9});
        declare_parameter("head_ids", std::vector<long>{10, 11});
        declare_parameter("telemetry_rate", 50.0);

        auto port = get_parameter("port").as_string();
        auto baud = get_parameter("baud").as_int();
        auto loc_ids = get_parameter("loc_ids").as_integer_array();
        auto arm_ids = get_parameter("arm_ids").as_integer_array();
        auto head_ids = get_parameter("head_ids").as_integer_array();
        auto rate = get_parameter("telemetry_rate").as_double();

        try {
            motor_manager_ = std::make_unique<MotorManager>(port, baud);
            RCLCPP_INFO(get_logger(), "Connected to serial port: %s at %ld baud", port.c_str(), baud);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to open serial port: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        // Combine all IDs for testing
        all_motor_ids_.clear();
        for (auto id : loc_ids) all_motor_ids_.push_back(static_cast<uint8_t>(id));
        for (auto id : arm_ids) all_motor_ids_.push_back(static_cast<uint8_t>(id));
        for (auto id : head_ids) all_motor_ids_.push_back(static_cast<uint8_t>(id));

        // Create timer for periodic telemetry
        timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
            [this]() { ping_and_read_telemetry(); });

        RCLCPP_INFO(get_logger(), "Starting motor ping test with %zu motors", all_motor_ids_.size());
        ping_motors();
    }

private:
    void ping_motors()
    {
        RCLCPP_INFO(get_logger(), "Pinging all motors...");
        for (auto id : all_motor_ids_) {
            bool online = motor_manager_->ping(id);
            RCLCPP_INFO(get_logger(), "Motor ID %d: %s", id, online ? "ONLINE" : "OFFLINE");
        }
        RCLCPP_INFO(get_logger(), "Motor ping completed.");
    }

    void ping_and_read_telemetry()
    {
        for (auto id : all_motor_ids_) {
            bool online = motor_manager_->ping(id);
            if (online) {
                auto pos = motor_manager_->readPresentPosition(id);
                auto speed = motor_manager_->readPresentSpeed(id);
                auto temp = motor_manager_->readTemperature(id);
                auto voltage = motor_manager_->readVoltage(id);

                RCLCPP_INFO(get_logger(), 
                    "Motor %d: pos=%s, speed=%s, temp=%s°C, voltage=%sV",
                    id,
                    pos ? std::to_string(*pos).c_str() : "N/A",
                    speed ? std::to_string(*speed).c_str() : "N/A", 
                    temp ? std::to_string(*temp).c_str() : "N/A",
                    voltage ? std::to_string(*voltage).c_str() : "N/A");
            } else {
                RCLCPP_WARN(get_logger(), "Motor %d: OFFLINE", id);
            }
        }
    }

    std::unique_ptr<MotorManager> motor_manager_;
    std::vector<uint8_t> all_motor_ids_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorPingTest>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}