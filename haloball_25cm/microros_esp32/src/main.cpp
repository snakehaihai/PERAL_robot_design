#include <Arduino.h>
#include "config.h"
#include "encoder.h"
#include "motor_control.h"
#include "pid_controller.h"
#include "microros.h"

// PID controllers for left and right motors
PIDController left_pid(LEFT_PID_KP, LEFT_PID_KI, LEFT_PID_KD, MAX_PWM, MIN_PWM);
PIDController right_pid(RIGHT_PID_KP, RIGHT_PID_KI, RIGHT_PID_KD, MAX_PWM, MIN_PWM);

// Variables for target speeds
float linear_vel = 0;    // m/s
float angular_vel = 0;   // rad/s

float left_speed = 0;
float right_speed = 0;
float left_target_speed  = 0;
float right_target_speed = 0;

// Timing and encoder tracking variables
int last_left_count = 0;
int last_right_count = 0;

unsigned long last_cmd_vel_time = 0;
unsigned long last_control_time = 0;
const unsigned long control_interval = 10; // 10ms, 100Hz control

// Initialise everything
void setup() { 
    Serial.begin(115200);
    setup_motors();
    setup_encoders();
    setup_microros(); // This is blocking. microros agent must connect in order to proceed.
    
    last_control_time = 0;
    left_encoder_count = 0;
    right_encoder_count = 0;
    last_left_count = 0;
    last_right_count = 0;
}

void loop() {
    microros_loop();

    // Auto cut-off motors if no commands given
    if (millis() - last_cmd_vel_time > CMD_VEL_TIMEOUT) {
        linear_vel = 0;
        angular_vel = 0;
    }

    if (millis() - last_control_time > control_interval) {
        float delta_time = (millis() - last_control_time) / 1000.0f;
    
        // Calculate encoder deltas
        int delta_left = left_encoder_count - last_left_count;
        int delta_right = right_encoder_count - last_right_count;
        
        // Update tracking variables
        last_control_time = millis();
        last_left_count = left_encoder_count;
        last_right_count = right_encoder_count;

        // Calculate current wheel speeds in rad/s
        left_speed = calculate_speed(delta_left, delta_time, L_ENCODER_CPR);
        right_speed = calculate_speed(delta_right, delta_time, R_ENCODER_CPR);

        // Publish wheel speeds via micro-ROS
        publish_wheel_speeds(left_speed, right_speed);

        // Calculate angular velocity (rad/s) of each wheel given linear_vel and angular_vel of the robot
        left_target_speed  = (linear_vel - (angular_vel * WHEEL_SEPARATION) / 2) / WHEEL_RADIUS;
        right_target_speed = (linear_vel + (angular_vel * WHEEL_SEPARATION) / 2) / WHEEL_RADIUS;

        // Compute PID outputs
        int left_pwm = left_pid.compute(left_target_speed, left_speed);
        int right_pwm = right_pid.compute(right_target_speed, right_speed);
        
        // Apply motor control
        set_motor_speed(PWMA, AIN1, AIN2, left_pwm);
        set_motor_speed(PWMB, BIN2, BIN1, right_pwm);

        // For debugging
        // Serial.printf("Left Error: %.2f | Left PWM: %d | Left Encoder: %d | Left: %.2f | Right Error: %.2f | Right PWM: %d | Right Encoder: %d | Right: %.2f\n",
        //         left_pid.getError(), left_pwm, left_encoder_count, left_speed, right_pid.getError(), right_pwm, right_encoder_count, right_speed);
    }
}