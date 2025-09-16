#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include <STSServoDriver.h>

// Hardware configuration
extern const float VEL_TO_SERVO_UNIT; // Conversion factor from rad/s to servo speed units
extern const float COUNTS_PER_REV;     // Encoder counts per revolution

// Servo ID configuration
#define BASE_SERVO_COUNT 3
#define ARM_SERVO_COUNT 6  // 6 arm joints only
#define HEAD_SERVO_COUNT 2 // camera pan + camera tilt

extern const uint8_t BASE_SERVO_IDS[BASE_SERVO_COUNT];
extern const uint8_t ARM_SERVO_IDS[ARM_SERVO_COUNT];
extern const uint8_t HEAD_SERVO_IDS[HEAD_SERVO_COUNT];

// Motor direction multipliers for holonomic drive
extern const float BASE_SERVO_DIRECTIONS[BASE_SERVO_COUNT];

// Global servo object - single instance for all servos
extern STSServoDriver servo_driver;

// Initialize servo communication
bool initServos(uint8_t rx_pin, uint8_t tx_pin);

// Check servo connectivity and configure
bool checkServos(void* display);

// Move arm to relaxed positions based on calibration
void moveToRelaxedPositions(void* display);

// Control functions
void controlBaseServo(uint8_t servo_index, float rad_per_sec, bool servos_enabled);
void controlMultipleBaseServos(float rad_per_sec_array[BASE_SERVO_COUNT], bool servos_enabled);  // Batch base servo control
void controlArmServo(uint8_t servo_index, float radians, bool servos_enabled);
void controlMultipleArmServos(float radians_array[ARM_SERVO_COUNT], bool servos_enabled);  // Batch arm servo control
void controlHeadServo(uint8_t servo_index, float radians, bool servos_enabled);
void controlMultipleHeadServos(float radians_array[HEAD_SERVO_COUNT], bool servos_enabled);  // Batch head servo control

// Emergency stop functions
void emergencyStopBase(void* display);
void emergencyStopArm(void* display);
void emergencyStopHead(void* display);

// Breaking recipe function
void applyBreakingRecipe(uint8_t servo_id, STSServoDriver& servo_driver);

#endif