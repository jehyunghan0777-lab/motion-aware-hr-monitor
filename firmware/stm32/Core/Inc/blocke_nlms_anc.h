/*
 * blocke_nlms_anc.h
 *
 *  Created on: May 17, 2026
 *      Author: jehyu
 */

#ifndef INC_BLOCKE_NLMS_ANC_H_
#define INC_BLOCKE_NLMS_ANC_H_

#include <stdint.h>

typedef enum {
	NLMS_MODE_BYPASS = 0,
	NLMS_MODE_ADAPT,
	NLMS_MODE_FREEZE
} nlms_mode_t;

extern float ppg_clean_rb[];
extern float ppg_clean_win[];

extern nlms_mode_t nlms_mode;

void BlockE_NLMS_ANC(void);

//test
extern uint32_t blocke_last_processed_idx;

//Exposing variables for correlation
/*
extern float D_corr_ax;
extern float D_corr_az;
extern float D_corr_ay;
extern float D_rms_ax;
extern float D_rms_ay;
extern float D_rms_az;
extern float D_rms_ppg;
*/




#endif /* INC_BLOCKE_NLMS_ANC_H_ */
