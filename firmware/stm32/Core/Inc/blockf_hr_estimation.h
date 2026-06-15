/*
 * blockf_hr_estimation.h
 *
 *  Created on: May 19, 2026
 *      Author: jehyu
 */

#ifndef INC_BLOCKF_HR_ESTIMATION_H_
#define INC_BLOCKF_HR_ESTIMATION_H_

#include <stdint.h>

typedef struct {
	float hr_bpm;
	float conf;
	uint32_t n_peaks;
	float ibi_var;
} hr_time_result_t;

typedef struct {
	float hr_bpm;
	float conf;
	float peak_hz;
	float peak_ratio;
	uint32_t k_peaks;
} hr_fft_result_t;

void BlockF_HR_Estimation(void);

extern hr_time_result_t hr_time_result;

extern hr_fft_result_t hr_fft_result;


//Exposing snapshot variables
/*
extern float snapshot_fft_mag[];
extern float snapshot_hrf;
extern float snapshot_peak_ratio;
extern float snapshot_conf ;
*/


#endif /* INC_BLOCKF_HR_ESTIMATION_H_ */
