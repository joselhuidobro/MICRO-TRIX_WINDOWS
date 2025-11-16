#pragma once
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float angle;
    float bias;
    float P00, P01, P10, P11;
} trix_kalman_axis_t;

void trix_kalman_init_axis(trix_kalman_axis_t *ax);

void trix_kalman_update(trix_kalman_axis_t *ax,
                        float newAngle,
                        float newRate,
                        float dt);

#ifdef __cplusplus
}
#endif

