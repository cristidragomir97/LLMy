#include "head_subscriber.h"
#include "microros.h"
#include "motor_controller.h"
#include "utils.h"
#include "config.h"
#include "debug_serial.h"
#include <micro_ros_utilities/type_utilities.h>

// Head subscriber objects
rcl_subscription_t head_cmd_sub;
std_msgs__msg__Float64MultiArray head_cmd_msg;

// Pointers to external variables
static bool* servos_enabled_ptr = nullptr;

// Message deduplication - store last received values
static float last_head_values[HEAD_SERVO_COUNT] = {0};
static bool first_head_message = true;

bool initHeadSubscriber(rcl_node_t* node, rclc_executor_t* executor, void* display) {
    // Create message memory for dynamic arrays
    micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
      &head_cmd_msg,
      (micro_ros_utilities_memory_conf_t) {
        .max_string_capacity = 0,
        .max_ros2_type_sequence_capacity = HEAD_SERVO_COUNT,
        .max_basic_type_sequence_capacity = HEAD_SERVO_COUNT
      });

    // Initialize Head Command Subscriber with best effort QoS for ros2_control compatibility
    rcl_ret_t rc = rclc_subscription_init_best_effort(
      &head_cmd_sub,
      node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
      HEAD_CMD_TOPIC
    );

    if (rc != RCL_RET_OK) {
        Serial1.println("Error creating head cmd subscription!");
        return false;
    }

    rc = rclc_executor_add_subscription(
      executor,
      &head_cmd_sub,
      &head_cmd_msg,
      &head_cmd_callback,
      ON_NEW_DATA
    );

    if (rc != RCL_RET_OK) {
        Serial1.println("Error adding head cmd subscription!");
        return false;
    }

    Serial1.println("Successfully added head cmd subscription");
    return true;
}

void setHeadServosEnabledPtr(bool* servos_enabled_ptr_arg) {
    servos_enabled_ptr = servos_enabled_ptr_arg;
}

void setHeadDisplayPtr(void* display_ptr_arg) {
    // Display functionality removed in simplified version
}

// Head command callback - expects Float64MultiArray with 2 values (radians)
void head_cmd_callback(const void *msgin) {
    //DEBUG_CALLBACK_START("HEAD");
    const std_msgs__msg__Float64MultiArray *msg = (std_msgs__msg__Float64MultiArray *)msgin;
    
    #if ENABLE_DEBUG_PRINTS
    Serial1.println("=== HEAD COMMAND CALLBACK ===");
    Serial.printf("Message data size: %d\n", msg->data.size);
    if (servos_enabled_ptr) {
        Serial.printf("Servos enabled: %s\n", *servos_enabled_ptr ? "YES" : "NO");
    }
    #endif
    
    if (msg->data.size >= HEAD_SERVO_COUNT) {
        // Check if values have changed (skip processing if identical)
        bool values_changed = first_head_message;
        if (!first_head_message) {
            for (int i = 0; i < HEAD_SERVO_COUNT; i++) {
                if (abs(msg->data.data[i] - last_head_values[i]) > 0.001) { // 1mrad tolerance
                    values_changed = true;
                    break;
                }
            }
        }
        
        if (values_changed) {
            #if ENABLE_DEBUG_PRINTS
            Serial.print("Raw values: [");
            for (int i = 0; i < HEAD_SERVO_COUNT; i++) {
                Serial.printf("%.4f", msg->data.data[i]);
                if (i < HEAD_SERVO_COUNT - 1) Serial.print(", ");
            }
            Serial1.println("]");
            #endif
            
            // Store new values and send commands using batched write
            for (int i = 0; i < HEAD_SERVO_COUNT; i++) {
                last_head_values[i] = msg->data.data[i];
            }
            
            bool enabled = servos_enabled_ptr ? *servos_enabled_ptr : false;
            controlMultipleHeadServos(last_head_values, enabled);
            
            first_head_message = false;
        }
        #if ENABLE_DEBUG_PRINTS
        else {
            Serial1.println("HEAD: Identical message, skipping servo commands");
        }
        #endif
        
        // Simplified - no display output
    } else {
        #if ENABLE_DEBUG_PRINTS
        Serial.printf("ERROR: Insufficient data size (%d < %d)\n", msg->data.size, HEAD_SERVO_COUNT);
        #endif
    }
    //DEBUG_CALLBACK_END("HEAD");
}