#include "motor_controller.h"
#include "utils.h"
#include "config.h"
#include "debug_serial.h"
#include "display.h"
#include <STSServoDriver.h>

// Hardware configuration
const float VEL_TO_SERVO_UNIT = 4096;   // Conversion factor from rad/s to servo speed units (4096 counts/rev)
const float COUNTS_PER_REV    = 4096;   // Encoder counts per revolution

// Servo ID configuration
const uint8_t BASE_SERVO_IDS[BASE_SERVO_COUNT] = { 1, 2, 3};        // back, left, right motors
const uint8_t ARM_SERVO_IDS[ARM_SERVO_COUNT]   = {4, 5, 6, 7, 8, 9}; // joints 1-6 only
const uint8_t HEAD_SERVO_IDS[HEAD_SERVO_COUNT] = {10, 11};         // camera pan, camera tilt

// Motor direction multipliers for holonomic drive
// These ensure each wheel spins in the correct direction for proper movement
const float BASE_SERVO_DIRECTIONS[BASE_SERVO_COUNT] = {-1.0, -1.0, -1.0}; // back, left, right motors

// Global servo object - single instance for all servos
STSServoDriver servo_driver;

// Track current modes to avoid unnecessary mode switches
static STSMode base_servo_modes[BASE_SERVO_COUNT] = {
    STSMode::VELOCITY,
    STSMode::VELOCITY,
    STSMode::VELOCITY
};

bool initServos(uint8_t rx_pin, uint8_t tx_pin) {
    Serial2.begin(1000000, SERIAL_8N1, rx_pin, tx_pin);

    bool init_success = servo_driver.init(&Serial2);

    return init_success;
}

bool checkServos(void* display) {
    // Arrays to track which servos are detected
    bool base_detected[BASE_SERVO_COUNT] = {false};
    bool arm_detected[ARM_SERVO_COUNT] = {false};
    bool head_detected[HEAD_SERVO_COUNT] = {false};
    bool all_ok = true;

    // PHASE 1: Detection only - ping all servos without setup
    Serial1.println("Detecting base servos...");
    displayStatus("Detecting base servos...");
    delay(50);

    for (int i = 0; i < BASE_SERVO_COUNT; i++) {
        uint8_t servo_id = BASE_SERVO_IDS[i];
        Serial.printf("Pinging base servo %d...\n", servo_id);
        
        char msg[32];
        snprintf(msg, sizeof(msg), "Ping base servo %d", servo_id);
        displayStatus(msg);

        base_detected[i] = servo_driver.ping(servo_id);
        if (!base_detected[i]) {
            snprintf(msg, sizeof(msg), "Base servo %d FAIL!", servo_id);
            displayError(msg);
            Serial.printf("Base servo %d FAIL!\n", servo_id);
            all_ok = false;
        } else {
            snprintf(msg, sizeof(msg), "Base servo %d OK", servo_id);
            displayStatus(msg);
            Serial.printf("Base servo %d detected OK\n", servo_id);
        }
        delay(100); // Consistent timing like servo_test
    }

    Serial1.println("Detecting arm servos...");
    displayStatus("Detecting arm servos...");
    delay(50);

    for (int i = 0; i < ARM_SERVO_COUNT; i++) {
        uint8_t servo_id = ARM_SERVO_IDS[i];
        
        char msg[32];
        snprintf(msg, sizeof(msg), "Ping arm servo %d", servo_id);
        displayStatus(msg);
        
        arm_detected[i] = servo_driver.ping(servo_id);
        if (!arm_detected[i]) {
            snprintf(msg, sizeof(msg), "Arm servo %d FAIL!", servo_id);
            displayError(msg);
            Serial.printf("Arm servo %d FAIL!\n", servo_id);
            all_ok = false;
        } else {
            snprintf(msg, sizeof(msg), "Arm servo %d OK", servo_id);
            displayStatus(msg);
            Serial.printf("Arm servo %d detected OK\n", servo_id);
        }
        delay(100); // Consistent timing
    }

    Serial1.println("Detecting head servos...");
    displayStatus("Detecting head servos...");
    delay(50);

    for (int i = 0; i < HEAD_SERVO_COUNT; i++) {
        uint8_t servo_id = HEAD_SERVO_IDS[i];
        
        char msg[32];
        snprintf(msg, sizeof(msg), "Ping head servo %d", servo_id);
        displayStatus(msg);
        
        head_detected[i] = servo_driver.ping(servo_id);
        if (!head_detected[i]) {
            snprintf(msg, sizeof(msg), "Head servo %d FAIL!", servo_id);
            displayError(msg);
            Serial.printf("Head servo %d FAIL!\n", servo_id);
            all_ok = false;
        } else {
            snprintf(msg, sizeof(msg), "Head servo %d OK", servo_id);
            displayStatus(msg);
            Serial.printf("Head servo %d detected OK\n", servo_id);
        }
        delay(100); // Consistent timing
    }

    if (!all_ok) {
        displayError("Some servos failed!");
        return false;
    }

    // PHASE 2: Setup detected servos
    Serial1.println("Setting up base servos...");
    displayStatus("Setting up base servos...");
    delay(100);

    for (int i = 0; i < BASE_SERVO_COUNT; i++) {
        if (base_detected[i]) {
            uint8_t servo_id = BASE_SERVO_IDS[i];
            Serial.printf("Setting up base servo %d...\n", servo_id);
            servo_driver.setMode(servo_id, STSMode::VELOCITY);
            delay(20);
            servo_driver.setTargetAcceleration(servo_id, 10);  // Set acceleration once
            delay(20);
            servo_driver.writeRegister(servo_id, STSRegisters::TORQUE_SWITCH, 1);
            delay(20);
            Serial.printf("Base servo %d setup complete\n", servo_id);
        }
    }

    Serial1.println("Setting up arm servos...");
    displayStatus("Setting up arm servos...");
    delay(100);

    for (int i = 0; i < ARM_SERVO_COUNT; i++) {
        if (arm_detected[i]) {
            uint8_t servo_id = ARM_SERVO_IDS[i];
            Serial.printf("Setting up arm servo %d...\n", servo_id);
            servo_driver.setMode(servo_id, STSMode::POSITION);
            delay(50);
            servo_driver.setTargetAcceleration(servo_id, 30);
            delay(20);
            servo_driver.setTargetVelocity(servo_id, 100);
            delay(20);
            servo_driver.writeRegister(servo_id, STSRegisters::TORQUE_SWITCH, 1);
            delay(50);
            Serial.printf("Arm servo %d setup complete\n", servo_id);
        }
    }

    Serial1.println("Setting up head servos...");
    displayStatus("Setting up head servos...");
    delay(100);

    for (int i = 0; i < HEAD_SERVO_COUNT; i++) {
        if (head_detected[i]) {
            uint8_t servo_id = HEAD_SERVO_IDS[i];
            Serial.printf("Setting up head servo %d...\n", servo_id);
            servo_driver.setMode(servo_id, STSMode::POSITION);
            delay(50);
            servo_driver.setTargetAcceleration(servo_id, 30);
            delay(20);
            servo_driver.setTargetVelocity(servo_id, 100);
            delay(20);
            servo_driver.writeRegister(servo_id, STSRegisters::TORQUE_SWITCH, 1);
            delay(50);
            Serial.printf("Head servo %d setup complete\n", servo_id);
        }
    }

    Serial1.println("All servos OK!");
    displaySuccess("All servos OK!");
    delay(500);
    return true;
}

void moveToRelaxedPositions(void* display) {
    Serial1.println("Moving to relaxed positions...");
    displayStatus("Moving to relaxed...");

    // Move arm servos to their calibrated relaxed positions
    for (int i = 0; i < ARM_CALIBRATION_COUNT; i++) {
        const ServoCalibration& cal = ARM_CALIBRATION[i];
        
        char msg[32];
        snprintf(msg, sizeof(msg), "Arm servo %d -> %d", cal.servo_id, cal.relaxed_position);
        displayStatus(msg);
        
        servo_driver.setTargetPosition(cal.servo_id, cal.relaxed_position);
        delay(50); // Small delay between commands

        Serial.printf("Arm servo %d -> %d\n", cal.servo_id, cal.relaxed_position);
        delay(300);
    }

    // Move head servos to their calibrated relaxed positions
    for (int i = 0; i < HEAD_CALIBRATION_COUNT; i++) {
        const ServoCalibration& cal = HEAD_CALIBRATION[i];
        
        char msg[32];
        snprintf(msg, sizeof(msg), "Head servo %d -> %d", cal.servo_id, cal.relaxed_position);
        displayStatus(msg);
        
        servo_driver.setTargetPosition(cal.servo_id, cal.relaxed_position);
        delay(50); // Small delay between commands

        Serial.printf("Head servo %d -> %d\n", cal.servo_id, cal.relaxed_position);
        delay(300);
    }

    Serial1.println("All relaxed positions set!");
    displaySuccess("Relaxed positions set!");
    delay(500);
}

void controlBaseServo(uint8_t servo_index, float rad_per_sec, bool servos_enabled) {
    if (servo_index >= BASE_SERVO_COUNT) return;

    uint8_t servo_id = BASE_SERVO_IDS[servo_index];
    
    // Apply direction multiplier for holonomic drive
    float corrected_rad_per_sec = rad_per_sec * BASE_SERVO_DIRECTIONS[servo_index];

#if ENABLE_DEBUG_PRINTS
    Serial1.printf("Servo %d (ID=%d): %.4f rad/s -> %.4f rad/s (dir=%.1f)", 
                   servo_index, servo_id, rad_per_sec, corrected_rad_per_sec, BASE_SERVO_DIRECTIONS[servo_index]);
#endif

#if ENABLE_SERVO_CONTROL
    if (servos_enabled) {
        // Simple velocity control - no mode switching
        int speed_int = (int)(radPerSecToServoSpeed(corrected_rad_per_sec));
        servo_driver.setTargetVelocity(servo_id, speed_int);
#if ENABLE_DEBUG_PRINTS
        Serial.printf(" -> SPEED: %d\n", speed_int);
#endif
    } else {
#if ENABLE_DEBUG_PRINTS
        Serial.printf(" -> DISABLED\n");
#endif
    }
#else
#if ENABLE_DEBUG_PRINTS
    Serial.printf(" -> SERVO_CONTROL_DISABLED\n");
#endif
#endif
}

void controlArmServo(uint8_t servo_index, float radians, bool servos_enabled) {
    if (servo_index >= ARM_SERVO_COUNT) return;

    uint8_t servo_id = ARM_SERVO_IDS[servo_index];
    DEBUG_MOTOR_COMMAND("ARM", servo_id, radians);

#if ENABLE_DEBUG_PRINTS
    Serial.printf("Servo %d (ID=%d): %.4f rad", servo_index, servo_id, radians);
#endif

#if ENABLE_SERVO_CONTROL
    if (servos_enabled) {
        // Use calibrated conversion function
        int16_t position = radiansToServoPosition(servo_id, radians);
        servo_driver.setTargetPosition(servo_id, position);
        DEBUG_MOTOR_SENT("ARM", servo_id, position);
#if ENABLE_DEBUG_PRINTS
        Serial.printf(" -> POS: %d\n", position);
#endif
    } else {
#if ENABLE_DEBUG_PRINTS
        Serial.printf(" -> DISABLED\n");
#endif
    }
#else
#if ENABLE_DEBUG_PRINTS
    Serial.printf(" -> SERVO_CONTROL_DISABLED\n");
#endif
#endif
}

void controlMultipleBaseServos(float rad_per_sec_array[BASE_SERVO_COUNT], bool servos_enabled) {
#if ENABLE_DEBUG_PRINTS
    Serial1.println("=== MULTIPLE BASE SERVO COMMAND ===");
#endif

#if ENABLE_SERVO_CONTROL
    if (servos_enabled) {
        // Process all velocity commands and send them directly
        for (int i = 0; i < BASE_SERVO_COUNT; i++) {
            uint8_t servo_id = BASE_SERVO_IDS[i];
            float corrected_rad_per_sec = rad_per_sec_array[i] * BASE_SERVO_DIRECTIONS[i];
            
#if ENABLE_DEBUG_PRINTS
            Serial.printf("Base servo %d: %.4f rad/s -> %.4f rad/s\n", servo_id, rad_per_sec_array[i], corrected_rad_per_sec);
#endif
            
            int speed_value = (int)(radPerSecToServoSpeed(corrected_rad_per_sec));
            servo_driver.setTargetVelocity(servo_id, speed_value);
        }

#if ENABLE_DEBUG_PRINTS
        Serial1.println("Multiple base commands sent!");
#endif
    } else {
#if ENABLE_DEBUG_PRINTS
        Serial1.println("BASE servos disabled - skipping multiple command");
#endif
    }
#else
#if ENABLE_DEBUG_PRINTS
    Serial1.println("SERVO_CONTROL_DISABLED - skipping multiple command");
#endif
#endif
}

void controlMultipleArmServos(float radians_array[ARM_SERVO_COUNT], bool servos_enabled) {
#if ENABLE_DEBUG_PRINTS
    Serial1.println("=== MULTIPLE ARM SERVO COMMAND ===");
#endif

#if ENABLE_SERVO_CONTROL
    if (servos_enabled) {
        // Convert all radians to servo positions first
        int16_t positions[ARM_SERVO_COUNT];

        for (int i = 0; i < ARM_SERVO_COUNT; i++) {
            uint8_t servo_id = ARM_SERVO_IDS[i];
            positions[i]     = radiansToServoPosition(servo_id, radians_array[i]);

#if ENABLE_DEBUG_PRINTS
            Serial.printf("Servo %d: %.4f rad -> %d pos\n", servo_id, radians_array[i], positions[i]);
#endif
        }

        // Send all position commands in rapid succession (batched)
        for (int i = 0; i < ARM_SERVO_COUNT; i++) {
            uint8_t servo_id = ARM_SERVO_IDS[i];
            servo_driver.setTargetPosition(servo_id, positions[i]);
            DEBUG_MOTOR_SENT("ARM", servo_id, positions[i]);
        }

#if ENABLE_DEBUG_PRINTS
        Serial1.println("Multiple arm commands sent!");
#endif
    } else {
#if ENABLE_DEBUG_PRINTS
        Serial1.println("ARM servos disabled - skipping multiple command");
#endif
    }
#else
#if ENABLE_DEBUG_PRINTS
    Serial1.println("SERVO_CONTROL_DISABLED - skipping multiple command");
#endif
#endif
}

void controlHeadServo(uint8_t servo_index, float radians, bool servos_enabled) {
    if (servo_index >= HEAD_SERVO_COUNT) return;

    uint8_t servo_id = HEAD_SERVO_IDS[servo_index];
    DEBUG_MOTOR_COMMAND("HEAD", servo_id, radians);

#if ENABLE_DEBUG_PRINTS
    Serial.printf("Head servo %d (ID=%d): %.4f rad", servo_index, servo_id, radians);
#endif

#if ENABLE_SERVO_CONTROL
    if (servos_enabled) {
        int16_t position = radiansToServoPosition(servo_id, radians);
        
#if ENABLE_DEBUG_PRINTS
        Serial.printf(" -> position %d\n", position);
#endif
        
        servo_driver.setMode(servo_id, STSMode::POSITION);
        delay(1);  // Brief delay after mode change
        servo_driver.setTargetPosition(servo_id, position);
    } else {
#if ENABLE_DEBUG_PRINTS
        Serial1.println(" (SERVOS DISABLED)");
#endif
    }
#else
#if ENABLE_DEBUG_PRINTS
    Serial1.println(" (SERVO_CONTROL_DISABLED)");
#endif
#endif
}

void controlMultipleHeadServos(float radians_array[HEAD_SERVO_COUNT], bool servos_enabled) {
#if ENABLE_DEBUG_PRINTS
    Serial1.println("=== MULTIPLE HEAD SERVO COMMAND ===");
#endif

#if ENABLE_SERVO_CONTROL
    if (servos_enabled) {
        // Convert all radians to servo positions first
        int16_t positions[HEAD_SERVO_COUNT];

        for (int i = 0; i < HEAD_SERVO_COUNT; i++) {
            uint8_t servo_id = HEAD_SERVO_IDS[i];
            positions[i] = radiansToServoPosition(servo_id, radians_array[i]);
            
#if ENABLE_DEBUG_PRINTS
            Serial.printf("Head servo %d (ID=%d): %.4f rad -> %d counts\n", i, servo_id, radians_array[i], positions[i]);
#endif
        }

        // Execute batch position commands
        for (int i = 0; i < HEAD_SERVO_COUNT; i++) {
            uint8_t servo_id = HEAD_SERVO_IDS[i];
            servo_driver.setMode(servo_id, STSMode::POSITION);
            delay(1);  // Brief delay after mode change
            servo_driver.setTargetPosition(servo_id, positions[i]);
            delay(1);  // Brief delay between commands
        }

#if ENABLE_DEBUG_PRINTS
        Serial1.println("HEAD: Multiple servo commands sent");
#endif
    } else {
#if ENABLE_DEBUG_PRINTS
        Serial1.println("HEAD: SERVOS DISABLED - skipping multiple command");
#endif
    }
#else
#if ENABLE_DEBUG_PRINTS
    Serial1.println("SERVO_CONTROL_DISABLED - skipping head multiple command");
#endif
#endif
}

void emergencyStopHead(void* display) {
    for (int i = 0; i < HEAD_SERVO_COUNT; i++) {
        servo_driver.writeRegister(HEAD_SERVO_IDS[i], STSRegisters::TORQUE_SWITCH, 0);  // Disable torque
    }
    Serial1.println("Head emergency stop activated!");
}

void emergencyStopBase(void* display) {
    for (int i = 0; i < BASE_SERVO_COUNT; i++) {
        servo_driver.setTargetVelocity(BASE_SERVO_IDS[i], 0);              // Stop velocity
        servo_driver.writeRegister(BASE_SERVO_IDS[i], STSRegisters::TORQUE_SWITCH, 0);  // Disable torque
    }
    Serial1.println("Base emergency stop activated!");
}

void emergencyStopArm(void* display) {
    for (int i = 0; i < ARM_SERVO_COUNT; i++) {
        servo_driver.writeRegister(ARM_SERVO_IDS[i], STSRegisters::TORQUE_SWITCH, 0);  // Disable torque
    }
    Serial1.println("Arm emergency stop activated!");
}
