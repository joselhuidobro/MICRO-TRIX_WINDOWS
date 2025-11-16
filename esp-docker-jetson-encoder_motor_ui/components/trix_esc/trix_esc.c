#include "trix_esc.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define ESC_FREQ_HZ          50
#define ESC_PWM_RESOLUTION   LEDC_TIMER_16_BIT
#define ESC_PWM_MAX_DUTY     ((1 << 16) - 1)

#define MOTOR1_PIN 13
#define MOTOR2_PIN 12
#define MOTOR3_PIN 19
#define MOTOR4_PIN 18

void trix_esc_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = ESC_PWM_RESOLUTION,
        .timer_num       = LEDC_TIMER_1,
        .freq_hz         = ESC_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t ch[4] = {
        {MOTOR1_PIN, LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, LEDC_INTR_DISABLE, LEDC_TIMER_1, 0, 0, {}},
        {MOTOR2_PIN, LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2, LEDC_INTR_DISABLE, LEDC_TIMER_1, 0, 0, {}},
        {MOTOR3_PIN, LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_3, LEDC_INTR_DISABLE, LEDC_TIMER_1, 0, 0, {}},
        {MOTOR4_PIN, LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_4, LEDC_INTR_DISABLE, LEDC_TIMER_1, 0, 0, {}}
    };
    for (int i = 0; i < 4; ++i) ledc_channel_config(&ch[i]);
}

static inline void set_esc_pulse_internal(int m, uint16_t us)
{
    uint32_t duty = (uint32_t)(((double)us / 20000.0) * ESC_PWM_MAX_DUTY);
    ledc_channel_t ch = m == 1 ? LEDC_CHANNEL_1 :
                        m == 2 ? LEDC_CHANNEL_2 :
                        m == 3 ? LEDC_CHANNEL_3 : LEDC_CHANNEL_4;
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, ch, duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, ch);
}

void trix_esc_set(int motor, uint16_t pulse_us)
{
    set_esc_pulse_internal(motor, pulse_us);
}

void trix_esc_arm(uint16_t pulse_us, uint16_t ms)
{
    for (uint16_t i = 0; i < ms / 20; ++i) {
        for (int m = 1; m <= 4; ++m) set_esc_pulse_internal(m, pulse_us);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

