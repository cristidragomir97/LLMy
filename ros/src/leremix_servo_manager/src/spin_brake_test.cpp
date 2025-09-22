#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <csignal>
#include "MotorManager.h"

// Global flag for clean shutdown
volatile bool running = true;

void signalHandler(int signal) {
    std::cout << "\n🛑 Caught signal " << signal << ", shutting down..." << std::endl;
    running = false;
}

int main(int argc, char* argv[]) {
    // Set up signal handling for clean shutdown
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    std::cout << "🚀 Spin & Brake Test - Direct MotorManager Testing" << std::endl;
    std::cout << "====================================================" << std::endl;
    
    // Configuration (matching servo_manager.yaml)
    const std::string port = "/dev/ttyTHS1";
    const int baud = 1000000;
    const std::vector<uint8_t> motor_ids = {1, 2, 3};
    
    try {
        // Initialize MotorManager
        std::cout << "🔌 Connecting to motor bus: " << port << " @ " << baud << " baud" << std::endl;
        MotorManager motor_manager(port, baud);
        std::cout << "✅ Motor bus connected successfully!" << std::endl;
        
        // Test motor connectivity
        std::cout << "\n🔍 Testing motor connectivity..." << std::endl;
        std::vector<uint8_t> responsive_motors;
        for (uint8_t id : motor_ids) {
            if (motor_manager.ping(id)) {
                responsive_motors.push_back(id);
                std::cout << "  ✅ Motor " << (int)id << ": ONLINE" << std::endl;
            } else {
                std::cout << "  ❌ Motor " << (int)id << ": OFFLINE" << std::endl;
            }
        }
        
        if (responsive_motors.empty()) {
            std::cout << "❌ No motors responding! Check power and connections." << std::endl;
            return 1;
        }
        
        std::cout << "📊 Found " << responsive_motors.size() << "/" << motor_ids.size() << " responsive motors" << std::endl;
        
        // Initialize motors to velocity mode
        std::cout << "\n🔧 Initializing motors to velocity mode..." << std::endl;
        for (uint8_t id : responsive_motors) {
            std::cout << "  🔄 Motor " << (int)id << ": ";
            
            // Enable torque
            if (!motor_manager.setTorque(id, true)) {
                std::cout << "Failed to enable torque" << std::endl;
                continue;
            }
            
            // Set to velocity mode (mode 1)
            if (!motor_manager.setMode(id, 1)) {
                std::cout << "Failed to set velocity mode" << std::endl;
                continue;
            }
            
            // Set acceleration
            if (!motor_manager.setAcceleration(id, 15)) {
                std::cout << "Failed to set acceleration" << std::endl;
                continue;
            }
            
            std::cout << "✅ Initialized successfully" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        std::cout << "\n🎯 Starting spin and brake test sequence..." << std::endl;
        std::cout << "Press Ctrl+C to stop at any time" << std::endl;
        
        int test_cycle = 1;
        
        while (running) {
            std::cout << "\n--- Test Cycle " << test_cycle << " ---" << std::endl;
            
            // PHASE 1: Spin motors
            std::cout << "🌀 PHASE 1: Spinning motors at moderate speed..." << std::endl;
            std::vector<int16_t> spin_speeds = {300, -300, 300}; // Alternating directions
            if (responsive_motors.size() == 2) {
                spin_speeds = {300, -300}; // Adjust for 2 motors
            } else if (responsive_motors.size() == 1) {
                spin_speeds = {300}; // Adjust for 1 motor
            }
            
            if (!motor_manager.syncWriteVelocity(responsive_motors, spin_speeds)) {
                std::cout << "❌ Failed to send spin velocities" << std::endl;
            } else {
                std::cout << "✅ Motors spinning..." << std::endl;
            }
            
            // Spin for 3 seconds
            for (int i = 0; i < 30 && running; ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                if (i % 10 == 0) {
                    std::cout << "  ⏱️  Spinning... " << (3 - i/10) << "s remaining" << std::endl;
                }
            }
            
            if (!running) break;
            
            // PHASE 2: Test different braking methods
            std::cout << "\n🛑 PHASE 2: Testing braking methods..." << std::endl;
            
            // Method 1: Velocity-based soft braking
            std::cout << "  Method 1: Velocity-based soft braking (zero velocity)" << std::endl;
            std::vector<int16_t> zero_speeds(responsive_motors.size(), 0);
            if (!motor_manager.syncWriteVelocity(responsive_motors, zero_speeds)) {
                std::cout << "  ❌ Failed to send zero velocities" << std::endl;
            } else {
                std::cout << "  ✅ Zero velocities sent" << std::endl;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(3000));
            
            // Method 2: Position-based hard braking using stopInPlace
            std::cout << "  Method 2: Position-based hard braking (stopInPlace)" << std::endl;
            if (!motor_manager.stopInPlace(responsive_motors, 50)) {
                std::cout << "  ❌ stopInPlace failed" << std::endl;
                
                // Fallback: Manual position braking
                std::cout << "  🔄 Attempting manual position braking..." << std::endl;
                for (uint8_t id : responsive_motors) {
                    // Switch to position mode
                    if (motor_manager.setMode(id, 0)) {
                        std::cout << "    Motor " << (int)id << ": Switched to position mode" << std::endl;
                        
                        // Read current position
                        auto pos = motor_manager.readPresentPosition(id);
                        if (pos) {
                            std::cout << "    Motor " << (int)id << ": Current position " << *pos << std::endl;
                            
                            // Lock to current position
                            if (motor_manager.moveTo(id, *pos, 50, 1000)) {
                                std::cout << "    Motor " << (int)id << ": ✅ Position locked" << std::endl;
                            } else {
                                std::cout << "    Motor " << (int)id << ": ❌ Failed to lock position" << std::endl;
                            }
                        } else {
                            std::cout << "    Motor " << (int)id << ": ❌ Failed to read position" << std::endl;
                        }
                    } else {
                        std::cout << "    Motor " << (int)id << ": ❌ Failed to switch to position mode" << std::endl;
                    }
                }
            } else {
                std::cout << "  ✅ stopInPlace succeeded" << std::endl;
            }
            
            // Hold brake for 2 seconds
            std::cout << "  ⏸️  Holding brake for 2 seconds..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            
            // PHASE 3: Release brake and return to velocity mode
            std::cout << "\n🔄 PHASE 3: Releasing brake and returning to velocity mode..." << std::endl;
            for (uint8_t id : responsive_motors) {
                if (motor_manager.setMode(id, 1)) {
                    std::cout << "  Motor " << (int)id << ": ✅ Back to velocity mode" << std::endl;
                } else {
                    std::cout << "  Motor " << (int)id << ": ❌ Failed to return to velocity mode" << std::endl;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            
            std::cout << "✅ Test cycle " << test_cycle << " completed" << std::endl;
            test_cycle++;
            
            // Wait before next cycle
            std::cout << "\n⏳ Waiting 2 seconds before next cycle..." << std::endl;
            for (int i = 0; i < 20 && running; ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
        
        // Clean shutdown
        std::cout << "\n🧹 Cleaning up - stopping all motors..." << std::endl;
        std::vector<int16_t> stop_speeds(responsive_motors.size(), 0);
        motor_manager.syncWriteVelocity(responsive_motors, stop_speeds);
        
        // Disable torque
        for (uint8_t id : responsive_motors) {
            motor_manager.setTorque(id, false);
            std::cout << "  Motor " << (int)id << ": Torque disabled" << std::endl;
        }
        
        std::cout << "✅ Test completed successfully!" << std::endl;
        
    } catch (const std::exception& e) {
        std::cout << "❌ Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}