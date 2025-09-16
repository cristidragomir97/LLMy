#include <Arduino.h>
#include "STSServoDriver.h"

STSServoDriver servos;

// Motor states
bool motor1_running = false;
bool motor2_running = false;
bool motor3_running = false;

// Motor control functions
void startMotor(int motorId) {
    int servoId = motorId; // servo IDs match motor numbers
    int speed = 500; // Forward rotation speed
    
    if (servos.ping(servoId)) {
        servos.setTargetVelocity(servoId, speed, 0); // Continuous rotation forward
        Serial.print("Motor ");
        Serial.print(motorId);
        Serial.println(" started");
        
        switch(motorId) {
            case 1: motor1_running = true; break;
            case 2: motor2_running = true; break;
            case 3: motor3_running = true; break;
        }
    } else {
        Serial.print("Motor ");
        Serial.print(motorId);
        Serial.println(" not found");
    }
}

void stopAllMotors() {
    for (int i = 1; i <= 3; i++) {
        if (servos.ping(i)) {
            servos.setTargetVelocity(i, 0, 0); // Stop rotation
        }
    }
    motor1_running = false;
    motor2_running = false;
    motor3_running = false;
    Serial.println("All motors stopped");
}

void processSerialCommand() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        command.toLowerCase();
        
        if (command == "m1") {
            startMotor(1);
        } else if (command == "m2") {
            startMotor(2);
        } else if (command == "m3") {
            startMotor(3);
        } else if (command == "s") {
            stopAllMotors();
        } else {
            Serial.println("Unknown command. Use: m1, m2, m3, s");
        }
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB
    }
    
    Serial2.begin(1000000, SERIAL_8N1, 18, 19);
    servos.init(&Serial2);
    
    delay(1000);
    Serial.println("Starting servo detection test...");
    
    // Test servo IDs from 1 to 20
    for (int id = 1; id <= 20; id++) {
        Serial.print("Testing servo ID: ");
        Serial.print(id);
        
        // Use ping method to detect servo
        if (servos.ping(id)) {
            Serial.println(" - DETECTED!");
        } else {
            Serial.println(" - No response");
        }
        
        delay(100);
    }

    servos.setMode(1, STSMode::VELOCITY);
    servos.setMode(2, STSMode::VELOCITY);
    servos.setMode(3, STSMode::VELOCITY);

    Serial.println("Servo detection test completed.");
    Serial.println("Serial motor control ready. Commands: m1, m2, m3, s");
}

void loop() {
    // Process serial commands continuously
    processSerialCommand();
    
    // Optional: periodic servo status check every 10 seconds
    static unsigned long lastScan = 0;
    if (millis() - lastScan > 10000) {
        Serial.println("\n--- Active servos ---");
        for (int id = 1; id <= 3; id++) {
            if (servos.ping(id)) {
                Serial.print("Servo ID ");
                Serial.print(id);
                Serial.print(" - Active");
                switch(id) {
                    case 1: Serial.println(motor1_running ? " (Running)" : " (Stopped)"); break;
                    case 2: Serial.println(motor2_running ? " (Running)" : " (Stopped)"); break;
                    case 3: Serial.println(motor3_running ? " (Running)" : " (Stopped)"); break;
                }
            }
        }
        lastScan = millis();
    }
    
    delay(10); // Small delay to prevent overwhelming the system
}