#include "trix_kalman_filter.h"
#include <math.h>

/* ────── Ajustes Optimizados ────── */
#define TRIX_KALMAN_MAX_BIAS  5.0f    // ±deg/s (reducido para mayor estabilidad)
#define TRIX_KALMAN_MAX_DT    0.02f   // 20 ms (máximo tiempo entre updates)
#define TRIX_KALMAN_MIN_DT    0.001f  // 1 ms (mínimo tiempo entre updates)

// Covarianzas ajustadas para drones (menos ruido en ángulos, más en giroscopio)
static const float Q_angle = 0.0005f;  // Reducido 50%
static const float Q_gyro  = 0.005f;  // Aumentado 66%
static const float R_angle = 0.01f;   // Reducido 66%

/* ────── Helpers Inline ────── */
__attribute__((always_inline)) 
static inline float trix_clamp(float v, float mn, float mx) {
    return fminf(fmaxf(v, mn), mx); // Más rápido que ternario
}

/* ────── API Mejorada ────── */
void trix_kalman_init_axis(trix_kalman_axis_t *k) {
    if (!k) return;
    
    // Reset más agresivo para evitar inestabilidades iniciales
    k->angle = 0.0f;
    k->bias  = 0.0f;
    k->P00 = 1e-4f;  // Valores iniciales más bajos para convergencia rápida
    k->P11 = 1e-4f;
    k->P01 = k->P10 = 0.0f;
}

void trix_kalman_update(trix_kalman_axis_t *k, 
                       float newAngle, 
                       float newRate, 
                       float dt) {
    // Validación reforzada
    if (!k || dt < TRIX_KALMAN_MIN_DT) return;
    dt = fminf(dt, TRIX_KALMAN_MAX_DT);

    /* ──── Fase de Predicción ──── */
    // Paso 1: Actualizar ángulo (incluyendo bias)
    float rate = newRate - k->bias;
    k->angle += rate * dt;

    // Paso 2: Actualizar matriz de covarianza (P)
    float dtP11 = dt * k->P11;
    k->P00 += dt * (dt * k->P11 - k->P01 - k->P10 + Q_angle);
    k->P01 -= dtP11;
    k->P10 -= dtP11;
    k->P11 += Q_gyro * dt;

    /* ──── Fase de Corrección ──── */
    // Paso 1: Calcular ganancia de Kalman (K)
    float S = k->P00 + R_angle;
    float invS = 1.0f / S;  // Evitar división múltiple
    float K0 = k->P00 * invS;
    float K1 = k->P10 * invS;

    // Paso 2: Actualizar estimación
    float y = newAngle - k->angle;
    k->angle += K0 * y;
    k->bias  += K1 * y;
    k->bias   = trix_clamp(k->bias, -TRIX_KALMAN_MAX_BIAS, TRIX_KALMAN_MAX_BIAS);

    // Paso 3: Actualizar covarianza (P = (I - K*H)*P)
    float P00_old = k->P00;
    float P01_old = k->P01;
    
    k->P00 -= K0 * P00_old;
    k->P01 -= K0 * P01_old;
    k->P10 -= K1 * P00_old;
    k->P11 -= K1 * P01_old;
}
