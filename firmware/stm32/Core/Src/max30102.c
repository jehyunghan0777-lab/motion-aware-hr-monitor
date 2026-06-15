/*
 * max30102.c
 *
 *  Created on: May 12, 2026
 *      Author: jehyu
 */
/* ---------------- Includes ---------------- */
#include "max30102.h"


/* ---------------- Defines ---------------- */
#define I2C_RUNTIME_TIMEOUT_MS 10


/* ---------------- Functions ---------------- */
static HAL_StatusTypeDef max30102_write_reg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value){
	return HAL_I2C_Mem_Write(hi2c, MAX30102_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, I2C_RUNTIME_TIMEOUT_MS);
}

static HAL_StatusTypeDef max30102_read_reg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *value){
	return HAL_I2C_Mem_Read(hi2c, MAX30102_ADDR, reg, I2C_MEMADD_SIZE_8BIT, value, 1, I2C_RUNTIME_TIMEOUT_MS);
}

static HAL_StatusTypeDef max30102_read_mult_regs(I2C_HandleTypeDef *hi2c, uint8_t start_reg, uint8_t *buf, uint16_t len){
	return HAL_I2C_Mem_Read(hi2c, MAX30102_ADDR, start_reg, I2C_MEMADD_SIZE_8BIT, buf, len, I2C_RUNTIME_TIMEOUT_MS);
}

HAL_StatusTypeDef max30102_read_who_am_i(I2C_HandleTypeDef *hi2c, uint8_t *id){

	if(hi2c == NULL || id == NULL) {
		return HAL_ERROR;
	}

	return max30102_read_reg(hi2c, MAX30102_PART_ID, id);
}


HAL_StatusTypeDef max30102_reset(I2C_HandleTypeDef *hi2c){

	HAL_StatusTypeDef st;
	uint8_t mode_config;
	uint32_t start;

	if(hi2c == NULL) {
		return HAL_ERROR;
	}

	//MODE_CONFIG bit 6 = reset
	st = max30102_write_reg(hi2c, MAX30102_MODE_CONFIG, 0x40);
	if(st != HAL_OK) return st;

	start = HAL_GetTick();

	do {
		st = max30102_read_reg(hi2c, MAX30102_MODE_CONFIG, &mode_config);
		if(st != HAL_OK) return st;

		if((mode_config & 0x40) == 0){
			return HAL_OK;
		}

	} while (HAL_GetTick() - start < 100);

	return HAL_TIMEOUT;
}

HAL_StatusTypeDef max30102_init(I2C_HandleTypeDef *hi2c){

	HAL_StatusTypeDef st;

	if(hi2c == NULL) return HAL_ERROR;

	st = max30102_reset(hi2c);
	if(st != HAL_OK) return st;

	uint8_t dummy;
	(void)max30102_read_reg(hi2c, MAX30102_INTR_STATUS_1, &dummy);
	(void)max30102_read_reg(hi2c, MAX30102_INTR_STATUS_2, &dummy);

	st = max30102_write_reg(hi2c, MAX30102_INTR_ENABLE_1, 0x00);
	if(st != HAL_OK) return st;
	st = max30102_write_reg(hi2c, MAX30102_INTR_ENABLE_2, 0x00);
	if(st != HAL_OK) return st;

	st = max30102_write_reg(hi2c, MAX30102_FIFO_WR_PTR, 0x00);
	if(st != HAL_OK) return st;
	st = max30102_write_reg(hi2c, MAX30102_OVF_COUNTER, 0x00);
	if(st != HAL_OK) return st;
	st = max30102_write_reg(hi2c, MAX30102_FIFO_RD_PTR, 0x00);
	if(st != HAL_OK) return st;

	st = max30102_write_reg(hi2c, MAX30102_FIFO_CONFIG, 0x0F);
	if(st != HAL_OK) return st;

	st = max30102_write_reg(hi2c, MAX30102_SPO2_CONFIG, 0x27);
	if(st != HAL_OK) return st;

	st = max30102_write_reg(hi2c, MAX30102_LED1_PA, 0x1F);
	if(st != HAL_OK) return st;
	st = max30102_write_reg(hi2c, MAX30102_LED2_PA, 0x1F);
	if(st != HAL_OK) return st;

	st = max30102_write_reg(hi2c, MAX30102_MODE_CONFIG, 0x03);
	if(st != HAL_OK) return st;

	return st;

}

HAL_StatusTypeDef max30102_read_fifo(I2C_HandleTypeDef *hi2c, max30102_sample_t *sample){

	HAL_StatusTypeDef st;
	uint8_t buf[6];

	if(hi2c == NULL || sample == NULL) return HAL_ERROR;

	st = max30102_read_mult_regs(hi2c, MAX30102_FIFO_DATA, buf, 6);
	if(st != HAL_OK) return st;

	sample->red = ((uint32_t)buf[0]<<16) |
				  ((uint32_t)buf[1]<<8) |
				  ((uint32_t)buf[2]);

	sample->ir = ((uint32_t)buf[3]<<16) |
				  ((uint32_t)buf[4]<<8) |
				  ((uint32_t)buf[5]);

	sample->red &= 0x3FFFF;
	sample->ir &= 0x3FFFF;

	return HAL_OK;
}
