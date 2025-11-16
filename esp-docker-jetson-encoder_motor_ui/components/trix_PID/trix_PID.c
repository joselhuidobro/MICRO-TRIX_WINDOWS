#include "trix_PID.h"
#include <string.h>

void trix_pid_axis_init(trix_pid_t *pid,
                        float kp, float ki, float kd,
                        float output_lim)
{
    pid->h = NULL;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->max_output = output_lim;

    pid_ctrl_parameter_t p = {
        .kp = kp,
        .ki = ki,
        .kd = kd,
        .cal_type     = PID_CAL_TYPE_POSITIONAL,
        .max_output   =  output_lim,
        .min_output   = -output_lim,
        .max_integral =  output_lim * 0.5f,
        .min_integral = -output_lim * 0.5f,
    };

    pid_ctrl_config_t cfg = { .init_param = p };
    pid_new_control_block(&cfg, &pid->h);
}

float trix_pid_update(trix_pid_t *pid,
                      float setpoint,
                      float measurement)
{
    if (!pid || !pid->h) return 0.0f;

    float err = setpoint - measurement;
    float out = 0.0f;

    pid_compute(pid->h, err, &out);

    /* Anti-windup */
    if (out >  pid->max_output) out =  pid->max_output;
    if (out < -pid->max_output) out = -pid->max_output;

    return out;
}

void trix_pid_reset(trix_pid_t *pid)
{
    if (pid && pid->h) {
        pid_del_control_block(pid->h);
        pid->h = NULL;
    }
}

