#pragma once
#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float pitch;
    float roll;
    float yaw_rate;
     int16_t m1;         // Valor motor 1
    int16_t m2;         // Valor motor 2
    int16_t m3;         // Valor motor 3
    int16_t m4;         // Valor motor 4
} drone_telemetry_t;

/**
 * Inicializa la telemetría UDP.
 *  - jetson_ip:  cadena nula-terminada, p.ej. "192.168.0.82"
 *  - port:       puerto destino, p.ej. 5001
 */
esp_err_t trix_udp_telemetria_init(const char *jetson_ip, uint16_t port);

/**
 * Envía un paquete de telemetría.
 */
esp_err_t trix_udp_telemetria_send(const drone_telemetry_t *pkt);

#ifdef __cplusplus
}
#endif

