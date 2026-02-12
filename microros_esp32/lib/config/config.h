#pragma once

/*
Make sure the pins here matches the hardware connection.
Do not hack by swapping pins number.

If A is your right motor and B is your left motor, adjust main.cpp line 83-84 by swapping pwm

If your hardware is physically soldered to the pins of esp32, but your wheel rotate in the wrong direction,
adjust main.cpp line 83-84 and motor_control.cpp line 18-19 by swapping A or B IN1 and IN2 position
*/

// Motor Driver Pins (TB6612FNG)

// Stby pin
#define STBY 4

// Left motor
#define PWMA 5
#define AIN1 7
#define AIN2 6
// Right motor
#define PWMB 20
#define BIN1 9
#define BIN2 10

// Motor PWM Range
#define MAX_PWM 255
#define MIN_PWM -255

// Encoder Pins

// Left encoder
#define L_ENCODER_PINA 2
#define L_ENCODER_PINB 3
// Right encoder
#define R_ENCODER_PINA 1
#define R_ENCODER_PINB 0

// Encoder Count Per Revolution

// 30 (gear ratio) x 11 (magnetic loops/encoder lines) x 4 (quadrature edges)
#define L_ENCODER_CPR 1320
#define R_ENCODER_CPR 1320  

// PID Control Parameters - Individual for each wheel
#define LEFT_PID_KP 0.4f
#define LEFT_PID_KI 0.3f
#define LEFT_PID_KD 0.0f

#define RIGHT_PID_KP 0.4f
#define RIGHT_PID_KI 0.3f
#define RIGHT_PID_KD 0.0f

// microros parameters

// Robot Dimensions
#define WHEEL_RADIUS 0.034  // in meters
#define WHEEL_SEPARATION 0.194  // in meters

// Control Timing
#define CMD_VEL_TIMEOUT 500  // ms before stopping motors if no cmd_vel received

// Define ROS_DOMAIN_ID
#define ROS_DOMAIN_ID 47    // Must be same as computer to communicate