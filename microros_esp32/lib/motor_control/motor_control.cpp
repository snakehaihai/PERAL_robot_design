#include <Arduino.h>
#include "config.h"
#include "motor_control.h"

void setup_motors() {
  // Initialise motor control pins
  pinMode(STBY, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  digitalWrite(STBY, LOW); // put motor on standby

  // Stop motors initially
  set_motor_speed(PWMA, AIN1, AIN2, 0);
  set_motor_speed(PWMB, BIN2, BIN1, 0);
}

// Set motor speed (PWM + direction)
void set_motor_speed(int pwm_pin, int in1_pin, int in2_pin, int pwm) {
  pwm = constrain(pwm, MIN_PWM, MAX_PWM);
  digitalWrite(STBY, HIGH); // wake motor from standby
  if (pwm > 0) {
    digitalWrite(in1_pin, HIGH);
    digitalWrite(in2_pin, LOW);
  } else if (pwm < 0) {
    digitalWrite(in1_pin, LOW);
    digitalWrite(in2_pin, HIGH);
  } else {
    digitalWrite(in1_pin, LOW);
    digitalWrite(in2_pin, LOW);
  }
  analogWrite(pwm_pin, abs(pwm));
}
