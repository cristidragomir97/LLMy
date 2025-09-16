#include <Arduino.h>
#include "config.h"
#include "microros.h"
#include "motor_controller.h"
#include "base_subscriber.h"
#include "arm_subscriber.h"
#include "head_subscriber.h"
#include "joint_publisher.h"
#include "display.h"

// System state
bool servos_enabled = true;  // Global servo enable/disable flag - DEFAULT ENABLED

void setup() {
  // Initialize serial for debug output first
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 15, 2); // Servo serial port
  Serial1.println("Simple MicroROS Servo Driver Starting...");
  
  // Initialize display
  if (!initDisplay()) {
    Serial1.println("Display init failed!");
    delay(2000);
  }
  delay(1000);
  
  // Initialize servo communication
  displayProgress("Initializing servos...", 2, 8);
  Serial1.println("Initializing servos...");
  initServos(S_RXD, S_TXD);
  delay(500);
  
  displayProgress("Checking servos...", 3, 8);
  Serial1.println("Checking servos...");
  if (!checkServos(&display)) {
    displayError("Servo check failed!");
    while(1) delay(1000);
  }
  delay(500);
  
  // Move arm to relaxed positions after servo check
  displayProgress("Moving to position...", 4, 8);
  #if ENABLE_SERVO_CONTROL
  moveToRelaxedPositions(&display);
  #endif
  delay(500);

  // Initialize micro-ROS
  displayProgress("Starting micro-ROS...", 5, 8);
  Serial1.println("Initializing micro-ROS...");
  set_microros_serial_transports(Serial);
  
  bool ros_initialized = initMicroROS(&display);

  while (!ros_initialized) {
    displayError("micro-ROS init failed!");
    Serial1.println("micro-ROS init failed, retrying...");
    delay(2000);
    displayProgress("Retrying micro-ROS...", 5, 8);
    ros_initialized = initMicroROS(&display);
  }

  Serial1.println("micro-ROS initialized!");
  delay(500);

  // Initialize subscribers and publishers
  displayProgress("Setup subscribers...", 6, 8);
  Serial1.println("Setting up subscribers and publishers...");
  
  // Set up base subscriber
  setBaseServosEnabledPtr(&servos_enabled);
  setBaseDisplayPtr(&display);
  if (!initBaseSubscriber(&node, &executor, &display)) {
    displayError("Base subscriber failed!");
    Serial1.println("Base subscriber failed!");
    while(1) delay(1000);
  }

  // Set up arm subscriber  
  setArmServosEnabledPtr(&servos_enabled);
  setArmDisplayPtr(&display);
  if (!initArmSubscriber(&node, &executor, &display)) {
    displayError("Arm subscriber failed!");
    Serial1.println("Arm subscriber failed!");
    while(1) delay(1000);
  }

  // Set up head subscriber  
  setHeadServosEnabledPtr(&servos_enabled);
  setHeadDisplayPtr(&display);
  if (!initHeadSubscriber(&node, &executor, &display)) {
    displayError("Head subscriber failed!");
    Serial1.println("Head subscriber failed!");
    while(1) delay(1000);
  }

  displayProgress("Setup publisher...", 7, 8);
  // Set up joint state publisher
  if (!initJointPublisher(&node, &display)) {
    displayError("Joint publisher failed!");
    Serial1.println("Joint publisher failed!");
    while(1) delay(1000);
  }

  displayProgress("Complete!", 8, 8);
  delay(1000);
  displaySuccess("LeRemix Ready!");
  Serial1.println("Setup complete - waiting for ROS commands");
  delay(2000);
  
  // Show running status
  displayStatus("Waiting for commands...");
}

void loop() {
  // Run micro-ROS executor to handle incoming commands
  spinMicroRos();
  
  // Publish joint states at 200Hz (every 5ms)
  if (shouldPublishJointStates()) {
    publishJointStates();
  }
}