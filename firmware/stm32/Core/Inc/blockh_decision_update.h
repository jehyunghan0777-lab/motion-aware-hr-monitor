/*
 * blockh_decision_update.h
 *
 *  Created on: May 26, 2026
 *      Author: jehyu
 */

#ifndef INC_BLOCKH_DECISION_UPDATE_H_
#define INC_BLOCKH_DECISION_UPDATE_H_

#include <stdint.h>

typedef enum {
	MODE_HOLD = 0,
	MODE_FREQ
} hr_mode_t;

typedef struct {
	float hr_bpm;
	float conf;
	hr_mode_t mode;
} hr_final_result_t;

extern hr_final_result_t hr_final_result;

void BlockH_Decision_Update();

#endif /* INC_BLOCKH_DECISION_UPDATE_H_ */
