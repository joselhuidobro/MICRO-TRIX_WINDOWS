#include "motor.h"
#include "esp_err.h"
#include "esp_rom_sys.h"

static MotorConfig s_cfg{};
static bool  s_inited      = false;
static bool  s_stopped     = true;
static int8_t s_last_dir   = 0;   // -1, 0, +1
static uint32_t s_pwm_max  = 0;

static inline void pwm_set(uint32_t duty_left, uint32_t duty_right) {
  ledc_set_duty(s_cfg.ledc_mode, s_cfg.ch_left,  duty_left);
  ledc_update_duty(s_cfg.ledc_mode, s_cfg.ch_left);
  ledc_set_duty(s_cfg.ledc_mode, s_cfg.ch_right, duty_right);
  ledc_update_duty(s_cfg.ledc_mode, s_cfg.ch_right);
}

static inline void ledc_start_if_needed() {
  if (s_stopped) {
    pwm_set(0, 0);
    s_stopped = false;
  }
}

void motor_coast(void) {
  // Detener canales y forzar bajo
  ledc_stop(s_cfg.ledc_mode, s_cfg.ch_left,  0);
  ledc_stop(s_cfg.ledc_mode, s_cfg.ch_right, 0);
  s_stopped = true;
  s_last_dir = 0;
}

void motor_init(const MotorConfig& cfg) {
  s_cfg = cfg;
  s_pwm_max = (1u << cfg.resolution_bits) - 1u;

  // Pines en salida y bajo
  gpio_reset_pin(s_cfg.pin_left);
  gpio_reset_pin(s_cfg.pin_right);
  gpio_set_direction(s_cfg.pin_left,  GPIO_MODE_OUTPUT);
  gpio_set_direction(s_cfg.pin_right, GPIO_MODE_OUTPUT);
  gpio_set_level(s_cfg.pin_left,  0);
  gpio_set_level(s_cfg.pin_right, 0);

  // Timer LEDC
  ledc_timer_config_t t{};
  t.speed_mode      = s_cfg.ledc_mode;
  t.duty_resolution = (ledc_timer_bit_t)s_cfg.resolution_bits;
  t.timer_num       = s_cfg.ledc_timer;
  t.freq_hz         = s_cfg.pwm_freq_hz;
  t.clk_cfg         = LEDC_AUTO_CLK;
  ESP_ERROR_CHECK(ledc_timer_config(&t));

  // Canales
  ledc_channel_config_t ch{};
  ch.gpio_num   = s_cfg.pin_left;
  ch.speed_mode = s_cfg.ledc_mode;
  ch.channel    = s_cfg.ch_left;
  ch.intr_type  = LEDC_INTR_DISABLE;
  ch.timer_sel  = s_cfg.ledc_timer;
  ch.duty       = 0;
  ch.hpoint     = 0;
  ch.flags.output_invert = 0;
  ESP_ERROR_CHECK(ledc_channel_config(&ch));

  ch.gpio_num   = s_cfg.pin_right;
  ch.channel    = s_cfg.ch_right;
  ESP_ERROR_CHECK(ledc_channel_config(&ch));

  motor_coast();
  s_inited = true;
}

void motor_drive_u(float u) {
  if (!s_inited) return;

  // Banda muerta
  if (u < s_cfg.deadband_u && u > -s_cfg.deadband_u) {
    motor_coast();
    return;
  }

  ledc_start_if_needed();

  const int8_t dir = (u > 0.f) ? +1 : -1;
  float mag = u > 0.f ? u : -u;
  if (mag > 1.f) mag = 1.f;
  uint32_t duty = (uint32_t)(mag * (float)s_pwm_max);
  if (duty > s_pwm_max) duty = s_pwm_max;

  // Cambio de sentido seguro
  if (dir != s_last_dir) {
    motor_coast();
    esp_rom_delay_us(s_cfg.deadtime_us);
    ledc_start_if_needed();
    s_last_dir = dir;
  }

  if (dir > 0) {
    pwm_set(duty, 0);     // IZQ PWM, DER 0
  } else {
    pwm_set(0, duty);     // DER PWM, IZQ 0
  }
}

