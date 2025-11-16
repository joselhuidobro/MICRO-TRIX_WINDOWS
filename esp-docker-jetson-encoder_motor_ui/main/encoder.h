#pragma once
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// Inicializa el encoder y registra la ISR en el pin de “clock” (BUTTON_PIN).
// ppr = pulsos por revolución mecánica (ej. 1200)
void  encoder_init(gpio_num_t button_pin, gpio_num_t dir_pin1, gpio_num_t dir_pin2, int ppr);

// Lee un snapshot atómico de pulsos
long  encoder_get_pulses(void);

// Convierte a grados usando el PPR configurado
double encoder_get_degrees(void);

// Reestablece el contador de pulsos a 0
void  encoder_reset(void);

// Cambiar/consultar PPR en runtime (si lo necesitas)
void  encoder_set_ppr(int ppr);
int   encoder_get_ppr(void);

#ifdef __cplusplus
}
#endif

