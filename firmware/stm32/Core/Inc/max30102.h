/*
 * max30102.h
 *
 *  Created on: May 12, 2026
 *      Author: jehyu
 */

#ifndef INC_MAX30102_H_
#define INC_MAX30102_H_

/* ---------------- Includes ---------------- */
#include "stm32f4xx_hal.h"
#include <stdint.h>

/* ---------------- Private Defines ---------------- */
#define MAX30102_ADDR               (0x57 << 1)

#define MAX30102_INTR_STATUS_1      0x00
#define MAX30102_INTR_STATUS_2      0x01
#define MAX30102_INTR_ENABLE_1      0x02
#define MAX30102_INTR_ENABLE_2      0x03

#define MAX30102_FIFO_WR_PTR        0x04
#define MAX30102_OVF_COUNTER        0x05
#define MAX30102_FIFO_RD_PTR        0x06
#define MAX30102_FIFO_DATA          0x07

#define MAX30102_FIFO_CONFIG        0x08
#define MAX30102_MODE_CONFIG        0x09
#define MAX30102_SPO2_CONFIG        0x0A

#define MAX30102_LED1_PA            0x0C
#define MAX30102_LED2_PA            0x0D

#define MAX30102_REV_ID             0xFE
#define MAX30102_PART_ID            0xFF

/* ---------------- Private Defines ---------------- */
typedef struct {

	uint32_t red;
	uint32_t ir;

} max30102_sample_t;

/* ---------------- Function Prototypes ---------------- */
HAL_StatusTypeDef max30102_read_who_am_i(I2C_HandleTypeDef *hi2c, uint8_t *id);
HAL_StatusTypeDef max30102_init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef max30102_reset(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef max30102_read_fifo(I2C_HandleTypeDef *hi2c, max30102_sample_t *sample);

#endif /* INC_MAX30102_H_ */

