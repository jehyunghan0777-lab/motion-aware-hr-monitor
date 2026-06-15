/*
 * lsm6ds3.c
 *
 *  Created on: Apr 18, 2026
 *      Author: jehyu
 */
/* ---------------- Includes ---------------- */
#include "lsm6ds3.h"


/* ---------------- Defines ---------------- */
#define I2C_RUNTIME_TIMEOUT_MS 10


/* ---------------- Functions ---------------- */
static HAL_StatusTypeDef lsm6ds3_write_reg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value){
	return HAL_I2C_Mem_Write(hi2c, LSM6DS3_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, I2C_RUNTIME_TIMEOUT_MS);
}

static HAL_StatusTypeDef lsm6ds3_read_reg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *value){
	return HAL_I2C_Mem_Read(hi2c, LSM6DS3_ADDR, reg, I2C_MEMADD_SIZE_8BIT, value, 1, I2C_RUNTIME_TIMEOUT_MS);
}

static HAL_StatusTypeDef lsm6ds3_read_mult_regs(I2C_HandleTypeDef *hi2c, uint8_t start_reg, uint8_t *buf, uint16_t len){
	return HAL_I2C_Mem_Read(hi2c, LSM6DS3_ADDR, start_reg, I2C_MEMADD_SIZE_8BIT, buf, len, I2C_RUNTIME_TIMEOUT_MS);
}

HAL_StatusTypeDef lsm6ds3_read_who_am_i(I2C_HandleTypeDef *hi2c, uint8_t *id) {

	if(hi2c == NULL || id == NULL) {
		return HAL_ERROR;
	}
	return lsm6ds3_read_reg(hi2c, LSM6DS3_WHO_AM_I, id);
}

//Initialization Function
HAL_StatusTypeDef lsm6ds3_init(I2C_HandleTypeDef *hi2c){

	HAL_StatusTypeDef st;

	if(hi2c == NULL){
		return HAL_ERROR;
	}

	//Setting CTRL3_C = 0x04 + 0x40 respectively -> IF_INC = 1 for multi-byte reads & BDU = 1
	st = lsm6ds3_write_reg(hi2c, LSM6DS3_CTRL3_C, 0x44);
	if (st != HAL_OK) return st;

	//Setting CTRL1_XL = 0x40 -> Accel ODR = 104Hz & FS = +- 2g
	st = lsm6ds3_write_reg(hi2c, LSM6DS3_CTRL1_XL, 0x40);
	if (st != HAL_OK) return st;

	//Setting CTRL2_G = 0x40 -> Gyro ODR = 104Hz & FS = +- 245dps
	st = lsm6ds3_write_reg(hi2c, LSM6DS3_CTRL2_G, 0x40);
	if (st != HAL_OK) return st;

	return HAL_OK;

}

HAL_StatusTypeDef lsm6ds3_read_raw_acc(I2C_HandleTypeDef *hi2c, lsm6ds3_vec3_raw_t *acc){

	HAL_StatusTypeDef st;

	uint8_t buf[6];

	if(hi2c == NULL || acc == NULL){
		return HAL_ERROR;
	}

	st = lsm6ds3_read_mult_regs(hi2c, LSM6DS3_OUTX_L_XL, buf, 6);
	if(st != HAL_OK) return st;

	acc->x = (int16_t)((buf[1] << 8) | buf[0] );
	acc->y = (int16_t)((buf[3] << 8) | buf[2] );
	acc->z = (int16_t)((buf[5] << 8) | buf[4] );

	return HAL_OK;
}

HAL_StatusTypeDef lsm6ds3_read_raw_gyro(I2C_HandleTypeDef *hi2c, lsm6ds3_vec3_raw_t *gyro){

	HAL_StatusTypeDef st;

	uint8_t buf[6];

	if(hi2c == NULL || gyro == NULL){
		return HAL_ERROR;
	}

	st = lsm6ds3_read_mult_regs(hi2c, LSM6DS3_OUTX_L_G, buf, 6);
	if(st != HAL_OK) return st;

	gyro->x = (int16_t)((buf[1] << 8) | buf[0] );
	gyro->y = (int16_t)((buf[3] << 8) | buf[2] );
	gyro->z = (int16_t)((buf[5] << 8) | buf[4] );

	return HAL_OK;
}

HAL_StatusTypeDef lsm6ds3_read_acc_g(I2C_HandleTypeDef *hi2c, lsm6ds3_vec3_t *acc){

	if(hi2c == NULL || acc == NULL) {
		return HAL_ERROR;
	}

	lsm6ds3_vec3_raw_t raw;
	HAL_StatusTypeDef st;

	st = lsm6ds3_read_raw_acc(hi2c, &raw);
	if(st != HAL_OK) return st;

	acc->x = raw.x * 0.000061f;
	acc->y = raw.y * 0.000061f;
	acc->z = raw.z * 0.000061f;

	return HAL_OK;
}

HAL_StatusTypeDef lsm6ds3_read_gyro_d(I2C_HandleTypeDef *hi2c, lsm6ds3_vec3_t *gyro){

	if(hi2c == NULL || gyro == NULL) {
			return HAL_ERROR;
	}


	lsm6ds3_vec3_raw_t raw;
	HAL_StatusTypeDef st;

	st = lsm6ds3_read_raw_gyro(hi2c, &raw);
	if(st != HAL_OK) return st;

	gyro->x = raw.x * 0.00875f;
	gyro->y = raw.y * 0.00875f;
	gyro->z = raw.z * 0.00875f;

	return HAL_OK;
}
