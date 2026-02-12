#include <Arduino.h>
#include <micro_ros_platformio.h>
#include "microros.h"
#include "motor_control.h"
#include "config.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32.h>

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_init_options_t init_options;
rcl_node_t node;
rcl_subscription_t subscriber;
rcl_publisher_t left_speed_pub;
rcl_publisher_t right_speed_pub;
geometry_msgs__msg__Twist twist_msg;
std_msgs__msg__Float32 left_speed_msg;
std_msgs__msg__Float32 right_speed_msg;

unsigned long last_connection_time = 0;
const unsigned long connection_timeout = 1000;
unsigned long last_speed_publish_time = 0;
const unsigned long speed_publish_interval = 100; // 100ms, 10Hz

bool connected_before = false;

void cmd_vel_callback(const void *msg_in) {
    const auto *msg = (const geometry_msgs__msg__Twist *)msg_in;    
    linear_vel = msg->linear.x;
    angular_vel = msg->angular.z;

    last_cmd_vel_time = millis();
}

void publish_wheel_speeds(float left_speed, float right_speed) {
    if (millis() - last_speed_publish_time > speed_publish_interval) {
        left_speed_msg.data = left_speed;
        right_speed_msg.data = right_speed;
        
        rcl_publish(&left_speed_pub, &left_speed_msg, NULL);
        rcl_publish(&right_speed_pub, &right_speed_msg, NULL);
    }
}

// Cleanup microros when fail to establish communication with the microros agent
void cleanup_microros() {
    if (!connected_before) {
        rclc_support_fini(&support);
        rcl_init_options_fini(&init_options);
    }
    // Not needed anymore, since the esp32 will restart when microros agent terminates.
    // else {
    //     rclc_executor_fini(&executor);
    //     rcl_subscription_fini(&subscriber, &node);
    //     rcl_publisher_fini(&left_speed_pub, &node);
    //     rcl_publisher_fini(&right_speed_pub, &node);
    //     rcl_node_fini(&node);
    //     rclc_support_fini(&support);
    //     rcl_init_options_fini(&init_options);
    // }
}

void setup_microros() {
    while (!connected_before) {
        set_microros_serial_transports(Serial); // set transport
        // Get configured allocator
        allocator = rcl_get_default_allocator();

        // Initialise and set ROS_DOMAIN_ID
        init_options = rcl_get_zero_initialized_init_options();
        rcl_init_options_init(&init_options, allocator);
        rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID);

        /* rclc_support_init_with_options will try to establish communication with the microros agent. 
        If succeed, proceed to start subscriber and publisher*/ 
        if (rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator) == RCL_RET_OK) {   
            rclc_node_init_default(&node, "microros_esp32", "", &support);
            rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel");
            rclc_publisher_init_default(&left_speed_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/left_wheel_speed");
            rclc_publisher_init_default(&right_speed_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/right_wheel_speed");
            rclc_executor_init(&executor, &support.context, 1, &allocator);
            rclc_executor_add_subscription(&executor, &subscriber, &twist_msg, &cmd_vel_callback, ON_NEW_DATA);

            connected_before = true;
            last_connection_time = millis();
        }
        else {
            cleanup_microros();
            delay(500); // limit polling rate to reduce CPU power consumption.
        }
    }
}

void microros_loop() { 
    // If microros agent disconnected, restart esp32.
    if (millis() - last_connection_time > connection_timeout && connected_before) {
        ESP.restart();
    }
    // Spin executor to receive messages
    if (connected_before) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        last_connection_time = millis();
    }
}