/*  ───────── trix_autotune.c ─────────  */
#include "trix_autotune.h"
#include "trix_esc.h"          /* para trix_esc_set()            */
#include "esp_timer.h"         /* esp_timer_get_time()           */
#include <stdio.h>

#define CALIB_DURATION_US (3 * 1000000)  /* 3 s de PWM cero      */
#define THR_MIN_PWM       1000           /* usa tu mismo valor   */

static bool  esc_cal_done   = false;
static bool  tune_done      = false;
static int64_t calib_t0_us  = 0;

/* ------------ banner opcional ------------ */
void trix_autotune_hello(void)
{
    printf("Iniciando autotunnig de dron...\n");

}

/* ------------ rutina principal ------------ */
int8_t trix_autotune_pid(uint8_t              axis,
                         trix_pid_t          *pid,
                         trix_kalman_axis_t  *kal,
                         float               pv,
                         float               dt,
                         float               sp,
                         float              *out,
                         autotune_params_t   p)
{


    /* ---------- 1. AUTOTUNE SIMPLE (una pasada) ---------- */
    if (!tune_done) {
        /* Ejemplo: ajusta las ganancias un 20 % como demo   */
        pid->kp *= 1.20f;
        pid->ki *= 1.20f;
        pid->kd *= 1.20f;

        /* Aquí escribirías los nuevos valores al bloque real
           pid_ctrl_parameter_t par;
           pid_get_parameter(pid->h, &par);
           par.kp = pid->kp; par.ki = pid->ki; par.kd = pid->kd;
           pid_set_parameter(pid->h, &par);            */

        printf("[AUTOTUNE] axis:%u  Kp:%.3f Ki:%.3f Kd:%.3f --> DONE\n",
               axis, pid->kp, pid->ki, pid->kd);

        tune_done = true;
        return 1;                                    /* autotune listo */
    }

    /* ---------- 3. Operación normal ---------- */
    /* Sin cambiar nada, solo regresa en progreso==0 */
    *out = pv;                                       /* (dummy)        */
    return 0;
}

