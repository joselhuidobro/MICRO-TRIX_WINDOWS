// encoder.cpp (FIX)
#include "encoder.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_intr_alloc.h"
#include "esp_rom_sys.h"
#include "esp_attr.h"            // IRAM_ATTR

// ↓↓↓ Necesarios para portMUX_TYPE y portENTER/EXIT_CRITICAL
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
// (Opcional) #include "freertos/task.h"

static const char *TAG = "ENCODER";

// Pines y estado interno
static gpio_num_t s_button_pin = GPIO_NUM_NC;
static gpio_num_t s_dir1       = GPIO_NUM_NC;
static gpio_num_t s_dir2       = GPIO_NUM_NC;

static volatile long s_pulses = 0;

// Spinlock para secciones críticas (multi-core)
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static int    s_ppr               = 1200;
static double s_pulses_per_degree = 1200.0 / 360.0;

extern "C" void IRAM_ATTR encoder_isr(void* /*arg*/) {
    const int dir1 = gpio_get_level(s_dir1);
    const int dir2 = gpio_get_level(s_dir2);
    portENTER_CRITICAL_ISR(&s_mux);
    s_pulses = s_pulses + ((dir1 == dir2) ? -1 : +1);
    portEXIT_CRITICAL_ISR(&s_mux);
}

static inline void _recalc_ppd() {
    if (s_ppr <= 0) s_ppr = 1;
    s_pulses_per_degree = (double)s_ppr / 360.0;
}

void encoder_set_ppr(int ppr) {
    if (ppr <= 0) return;
    s_ppr = ppr;
    _recalc_ppd();
}

int encoder_get_ppr(void) { return s_ppr; }

void encoder_reset(void) {
    portENTER_CRITICAL(&s_mux);
    s_pulses = 0;
    portEXIT_CRITICAL(&s_mux);
}

long encoder_get_pulses(void) {
    long v;
    portENTER_CRITICAL(&s_mux);
    v = s_pulses;
    portEXIT_CRITICAL(&s_mux);
    return v;
}

double encoder_get_degrees(void) {
    return ((double)encoder_get_pulses()) / s_pulses_per_degree;
}

void encoder_init(gpio_num_t button_pin, gpio_num_t dir_pin1, gpio_num_t dir_pin2, int ppr) {
    s_button_pin = button_pin;
    s_dir1       = dir_pin1;
    s_dir2       = dir_pin2;
    encoder_set_ppr(ppr);
    encoder_reset();

    // Config pins
    gpio_reset_pin(s_button_pin);
    gpio_set_direction(s_button_pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(s_button_pin, GPIO_PULLUP_ONLY);
    gpio_set_intr_type(s_button_pin, GPIO_INTR_NEGEDGE);

    gpio_reset_pin(s_dir1);
    gpio_reset_pin(s_dir2);
    gpio_set_direction(s_dir1, GPIO_MODE_INPUT);
    gpio_set_direction(s_dir2, GPIO_MODE_INPUT);

    // ISR service (idempotente)
    esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }

    // Asegura handler propio
    gpio_isr_handler_remove(s_button_pin);
    ESP_ERROR_CHECK(gpio_isr_handler_add(s_button_pin, encoder_isr, nullptr));

    ESP_LOGI(TAG, "Init OK | BTN=%d DIR1=%d DIR2=%d | PPR=%d",
             (int)s_button_pin, (int)s_dir1, (int)s_dir2, s_ppr);
}

