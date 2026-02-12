#include <Arduino.h>
#include "pid_controller.h"

PIDController::PIDController(float kp, float ki, float kd, float max_pwm, float min_pwm):
    kp_(kp),
    ki_(ki),
    kd_(kd),
    max_pwm_(max_pwm),
    min_pwm_(min_pwm)
{}

float PIDController::compute(float setpoint, float measured_value) {
    
    float error = setpoint - measured_value;
    integral += error;
    derivative = error - prev_error;

    if(setpoint == 0 && error == 0)
    {
        integral = 0;
        derivative = 0;
    }

    float pid = (kp_ * error) + (ki_ * integral) + (kd_ * derivative);
    prev_error = error;
    
    return constrain(pid, min_pwm_, max_pwm_);
}