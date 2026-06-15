/*
 * blockh_decision_update.c
 *
 *  Created on: May 26, 2026
 *      Author: jehyu
 */
/* ---------------- Includes ---------------- */
#include "blockh_decision_update.h"
#include "blockf_hr_estimation.h"
#include "blockg_sqi.h"


/* ---------------- Defines ---------------- */
#define CONF_TH 0.50f


/* ---------------- BlockH Outputs ---------------- */
hr_final_result_t hr_final_result = {0};


/* ---------------- Internal States / Variables ---------------- */
static float previous_final_hr = 0.0f;


/* ---------------- Functions ---------------- */
void BlockH_Decision_Update(void){

	float final_hr = 0.0f;
	float final_conf = 0.0f;
	hr_mode_t mode = MODE_HOLD;

	uint8_t freq_ok = !sqi_result.freq_bad;

	if(freq_ok){

		final_hr = hr_fft_result.hr_bpm;
		final_conf = sqi_result.s_freq;
		mode = MODE_FREQ;

	}
	else{


		final_hr = previous_final_hr;
		final_conf = 0.0f;
		mode = MODE_HOLD;

	}

	hr_final_result.hr_bpm = final_hr;
	hr_final_result.conf = final_conf;
	hr_final_result.mode = mode;

	if(mode != MODE_HOLD && final_conf > CONF_TH){
		previous_final_hr = final_hr;
	}
}

