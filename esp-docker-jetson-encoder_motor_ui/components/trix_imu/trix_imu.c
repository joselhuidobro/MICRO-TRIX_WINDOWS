#include "trix_imu.h"
#include "mpu6050.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_err.h"

#define I2C_SDA_PIN GPIO_NUM_21
#define I2C_SCL_PIN GPIO_NUM_22
#define I2C_PORT    I2C_NUM_0
#define I2C_FREQ_HZ 400000

static mpu6050_handle_t mpu = NULL;

esp_err_t trix_imu_init(void) {
    i2c_config_t cfg = {};
    cfg.mode = I2C_MODE_MASTER;
    cfg.sda_io_num = I2C_SDA_PIN;
    cfg.scl_io_num = I2C_SCL_PIN;
    cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
    cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
    cfg.master.clk_speed = I2C_FREQ_HZ;

    esp_err_t err = i2c_param_config(I2C_PORT, &cfg);
    if (err != ESP_OK) return err;

    err = i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) return err;

    mpu = mpu6050_create(I2C_PORT, MPU6050_I2C_ADDRESS);
    if (!mpu) return ESP_FAIL;

    err = mpu6050_wake_up(mpu);
    if (err != ESP_OK) return err;

    err = mpu6050_config(mpu, ACCE_FS_2G, GYRO_FS_250DPS);
    return err;
}

esp_err_t trix_imu_read(float *accel_x, float *accel_y, float *accel_z,
                        float *gyro_x, float *gyro_y, float *gyro_z) {
    if (!mpu) return ESP_FAIL;

    mpu6050_acce_value_t a;
    mpu6050_gyro_value_t g;

    esp_err_t err = mpu6050_get_acce(mpu, &a);
    if (err != ESP_OK) return err;

    err = mpu6050_get_gyro(mpu, &g);
    if (err != ESP_OK) return err;

    *accel_x = a.acce_x;
    *accel_y = a.acce_y;
    *accel_z = a.acce_z;
    *gyro_x = g.gyro_x;
    *gyro_y = g.gyro_y;
    *gyro_z = g.gyro_z;

    return ESP_OK;
}
