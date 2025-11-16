#pragma once
#include "pid_ctrl.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    pid_ctrl_block_handle_t h;
    float kp;
    float ki;
    float kd;
    float max_output;            /* ← NUEVO  límite de salida */

} trix_pid_t;

void  trix_pid_axis_init(trix_pid_t *pid,
                         float kp, float ki, float kd,
                         float output_lim);

float trix_pid_update(trix_pid_t *pid,
                      float setpoint,
                      float measurement);

void  trix_pid_reset(trix_pid_t *pid);

#ifdef __cplusplus
}
#endif

