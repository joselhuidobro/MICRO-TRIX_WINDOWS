#pragma once
#include "driver/gpio.h"
#include "driver/ledc.h"

struct MotorConfig {
  gpio_num_t    pin_left;        // HBRIDGE_LEFT_PIN
  gpio_num_t    pin_right;       // HBRIDGE_RIGHT_PIN
  ledc_mode_t   ledc_mode;       // LEDC_HIGH_SPEED_MODE
  ledc_timer_t  ledc_timer;      // LEDC_TIMER_0
  ledc_channel_t ch_left;        // LEDC_CHANNEL_0
  ledc_channel_t ch_right;       // LEDC_CHANNEL_1
  int           pwm_freq_hz;     // 20000
  int           resolution_bits; // 10
  int           deadtime_us;     // 5
  float         deadband_u;      // 0.02f
};

void motor_init(const MotorConfig& cfg);

// Mapea u ∈ [-1..+1] a PWM y maneja sentido con dead-time.
void motor_drive_u(float u);

// Detiene ambos PWMs y deja pines en bajo
void motor_coast(void);

