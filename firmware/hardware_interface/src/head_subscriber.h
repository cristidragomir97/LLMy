#ifndef HEAD_SUBSCRIBER_H
#define HEAD_SUBSCRIBER_H

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float64_multi_array.h>

// Head subscriber objects
extern rcl_subscription_t head_cmd_sub;
extern std_msgs__msg__Float64MultiArray head_cmd_msg;

// Initialize head subscriber
bool initHeadSubscriber(rcl_node_t* node, rclc_executor_t* executor, void* display);

// Head command callback
void head_cmd_callback(const void *msgin);

// Set servos enabled state pointer for callback use
void setHeadServosEnabledPtr(bool* servos_enabled_ptr);

// Set display pointer for callback use (unused in simplified version)
void setHeadDisplayPtr(void* display_ptr);

#endif