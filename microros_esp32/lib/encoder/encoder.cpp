#include <Arduino.h>
#include "config.h"
#include "encoder.h"

// Define local variables
volatile int left_encoder_count = 0;
volatile int right_encoder_count = 0;
volatile uint8_t left_encoder_state = 0;
volatile uint8_t right_encoder_state = 0;

// ISR handler
void IRAM_ATTR handle_encoder_isr(volatile int& encoder_count, volatile uint8_t& encoder_state, uint8_t encoder_pina, uint8_t encoder_pinb) {
  // Read both pins simultaneously using direct GPIO access
  uint32_t gpio_values = GPIO.in.val;
  uint8_t pin_a = (gpio_values >> encoder_pina) & 1;
  uint8_t pin_b = (gpio_values >> encoder_pinb) & 1;
  uint8_t new_state = (pin_a << 1) | pin_b;
  
  // State transition table (A B: 00, 01, 10, 11)
  static const int8_t state_transition[] = {
    0,  1, -1,  0, 
    -1,  0,  0,  1,
    1,  0,  0, -1,
    0, -1,  1,  0
  };
  
  int8_t transition = state_transition[(encoder_state << 2) | new_state];
  encoder_count += transition;
  encoder_state = new_state;
}

// Wrappers to call handle_encoder_isr function
void IRAM_ATTR left_encoder_isr() {
  handle_encoder_isr(left_encoder_count, left_encoder_state, L_ENCODER_PINA, L_ENCODER_PINB);
}

void IRAM_ATTR right_encoder_isr() {
  handle_encoder_isr(right_encoder_count, right_encoder_state, R_ENCODER_PINA, R_ENCODER_PINB);
}

// Setup encoders
void setup_encoders() {
  pinMode(L_ENCODER_PINA, INPUT_PULLUP);
  pinMode(L_ENCODER_PINB, INPUT_PULLUP);
  pinMode(R_ENCODER_PINA, INPUT_PULLUP);
  pinMode(R_ENCODER_PINB, INPUT_PULLUP);

  // Read initial state
  left_encoder_state = (digitalRead(L_ENCODER_PINA) << 1) | digitalRead(L_ENCODER_PINB);
  right_encoder_state = (digitalRead(R_ENCODER_PINA) << 1) | digitalRead(R_ENCODER_PINB);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(L_ENCODER_PINA), left_encoder_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(L_ENCODER_PINB), left_encoder_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENCODER_PINA), right_encoder_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENCODER_PINB), right_encoder_isr, CHANGE);
}

// Calculate speed in rad/s from encoder counts
float calculate_speed(int encoder_count, float delta_time, int encoder_cpr) {
  float revolutions = encoder_count / (float)encoder_cpr;
  float rad_per_sec = (revolutions * 2 * PI) / delta_time;
  return rad_per_sec;
}