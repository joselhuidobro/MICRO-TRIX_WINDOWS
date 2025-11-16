#ifndef TRIX_IMU_H
#define TRIX_IMU_H

#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t trix_imu_init(void);
esp_err_t trix_imu_read(float *accel_x, float *accel_y, float *accel_z,
                        float *gyro_x, float *gyro_y, float *gyro_z);

#ifdef __cplusplus
}
#endif

#endif // TRIX_IMU_H
