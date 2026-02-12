#pragma once

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float max_pwm, float min_pwm);
    float compute(float setpoint, float measured_value);
    float getError() const { return prev_error; }
    
private:
    float kp_;
    float ki_;
    float kd_;
    float max_pwm_;
    float min_pwm_;
    
    double integral;
    double derivative;
    double prev_error;
};