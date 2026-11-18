/* imu_lsm6ds3.h — LSM6DS3 backend (spec §7.11.1, D-42). */
#ifndef IMU_LSM6DS3_H
#define IMU_LSM6DS3_H

#include "imu.h"

extern const imu_backend_t IMU_LSM6DS3;
void imu_lsm6ds3_bind(SPI_HandleTypeDef *hspi);     /* init only, before probe() */

#endif
