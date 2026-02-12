#pragma once

void setup_microros();
void microros_loop();
void publish_wheel_speeds(float left_speed, float right_speed);

extern unsigned long last_cmd_vel_time;
extern float linear_vel;
extern float angular_vel;