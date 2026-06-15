/*
 * blockg_sqi.h
 *
 *  Created on: May 22, 2026
 *      Author: jehyu
 */

#ifndef INC_BLOCKG_SQI_H_
#define INC_BLOCKG_SQI_H_

#include <stdint.h>
#include "blockf_hr_estimation.h"

typedef struct {

	float s_motion;
	float s_time;
	float s_freq;

	uint8_t time_bad;
	uint8_t freq_bad;
} sqi_result_t;

extern sqi_result_t sqi_result;

void BlockG_SQI_Update(void);


#endif /* INC_BLOCKG_SQI_H_ */
