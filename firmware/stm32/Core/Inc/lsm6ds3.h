/*
 * lsm6ds3.h
 *
 *  Created on: Apr 18, 2026
 *      Author: jehyu
 */

#ifndef LSM6DS3_H_
#define LSM6DS3_H_

/* ---------------- Includes ---------------- */
#include "stm32f4xx_hal.h"
#include <stdint.h>

/* ---------------- Private Defines ---------------- */
#define LSM6DS3_ADDR           (0x6A << 1)

#define LSM6DS3_WHO_AM_I       0x0F
#define LSM6DS3_CTRL1_XL       0x10
#define LSM6DS3_CTRL2_G        0x11
#define LSM6DS3_CTRL3_C        0x12
#define LSM6DS3_STATUS_REG     0x1E

#define LSM6DS3_OUTX_L_G       0x22
#define LSM6DS3_OUTX_L_XL      0x28

/* ---------------- Private Defines ---------------- */
typedef struct {

	int16_t x;
	int16_t y;
	int16_t z;

} lsm6ds3_vec3_raw_t;

typedef struct {

	float x;
	float y;
	float z;

} lsm6ds3_vec3_t;


/* ---------------- Function Prototypes ---------------- */

HAL_StatusTypeDef lsm6ds3_init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef lsm6ds3_read_who_am_i(I2C_HandleTypeDef *hi2c, uint8_t *id);
HAL_StatusTypeDef lsm6ds3_read_raw_acc(I2C_HandleTypeDef *hi2c, lsm6ds3_vec3_raw_t *acc);
HAL_StatusTypeDef lsm6ds3_read_raw_gyro(I2C_HandleTypeDef *hi2c,  lsm6ds3_vec3_raw_t *gyro);
HAL_StatusTypeDef lsm6ds3_read_acc_g(I2C_HandleTypeDef *hi2c, lsm6ds3_vec3_t *acc);
HAL_StatusTypeDef lsm6ds3_read_gyro_d(I2C_HandleTypeDef *hi2c, lsm6ds3_vec3_t *gyro);

#endif /* LSM6DS3_H_ */





