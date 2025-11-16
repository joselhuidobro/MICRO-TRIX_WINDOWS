#pragma once
#include "trix_PID.h"
#include "trix_kalman_filter.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int8_t   mode;
    float    noise_band;
    float    output_step;
    uint16_t sample_time;
    uint16_t settle_time;
    uint8_t  max_cycles;
    float    setpoint_step;
} autotune_params_t;

/* 0 = en progreso, 1 = completado, -1 = error */
int8_t trix_autotune_pid(uint8_t              axis,
                         trix_pid_t          *pid,
                         trix_kalman_axis_t  *kal,
                         float               process_var,
                         float               dt,
                         float               setpoint,
                         float              *output,
                         autotune_params_t   params);

void   trix_autotune_hello(void);

#ifdef __cplusplus
}
#endif

