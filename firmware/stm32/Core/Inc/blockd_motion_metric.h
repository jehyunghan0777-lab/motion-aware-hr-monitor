/*
 * blockd_motion_metric.h
 *
 *  Created on: May 15, 2026
 *      Author: jehyu
 */

#ifndef INC_BLOCKD_MOTION_METRIC_H_
#define INC_BLOCKD_MOTION_METRIC_H_

#include <stdint.h>

void BlockD_Motion_Metric(void);

extern float latest_m;

typedef enum {
	MOTION_LOW = 0,
	MOTION_MODERATE,
	MOTION_HIGH
} motion_state_t;

extern motion_state_t motion_state;

#endif /* INC_BLOCKD_MOTION_METRIC_H_ */
