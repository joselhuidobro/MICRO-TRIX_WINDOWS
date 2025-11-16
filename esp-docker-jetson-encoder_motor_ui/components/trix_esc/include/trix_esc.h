#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
void trix_esc_init(void);
void trix_esc_arm(uint16_t pulse_us, uint16_t ms);
void trix_esc_set(int motor, uint16_t pulse_us);
#ifdef __cplusplus
}
#endif

