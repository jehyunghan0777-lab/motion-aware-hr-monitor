/*
 * blockg_sqi.c
 *
 *  Created on: May 23, 2026
 *      Author: jehyu
 */
/* ---------------- Includes ---------------- */
#include <stdint.h>
#include <math.h>

#include "blockd_motion_metric.h"
#include "blockf_hr_estimation.h"
#include "blockg_sqi.h"

/* ---------------- Defines ---------------- */
//SQI Tunable Parameters
#define SQI_M_FULL_SCALE 10000.0f

#define HT_CONF_TH 0.30f
#define HT_MIN_PEAKS 4U

#define HF_CONF_TH 0.30f
#define HF_RATIO_TH 1.60f

#define INVALID_SQI_TH 0.30f

/* ---------------- BlockG Outputs ---------------- */
sqi_result_t sqi_result = {0};

/* ---------------- Functions ---------------- */
//Utility clamp function
static float clamp_01(float x){

	if(x < 0.0f){
		x = 0.0f;
	}

	if(x > 1.0f){
		x = 1.0f;
	}

	return x;
}

void BlockG_SQI_Update(void){

	sqi_result_t r = {0};

	float M = latest_m;

	float s_motion = 1.0f - (M / SQI_M_FULL_SCALE);
	s_motion = clamp_01(s_motion);

	float s_time = 0.0f;

	if(hr_time_result.n_peaks >= HT_MIN_PEAKS && hr_time_result.hr_bpm > 0.0f){
		s_time = hr_time_result.conf;
	}
	s_time = clamp_01(s_time);

	float s_freq = 0.0f;

	if(hr_fft_result.hr_bpm > 0.0f){
		s_freq = hr_fft_result.conf;
	}
	s_freq = clamp_01(s_freq);

	uint8_t time_valid =
	    (hr_time_result.hr_bpm > 0.0f) &&
	    (hr_time_result.conf > 0.0f) &&
	    (hr_time_result.n_peaks >= HT_MIN_PEAKS);

	uint8_t freq_valid =
	    (hr_fft_result.hr_bpm > 0.0f) &&
	    (hr_fft_result.conf > 0.0f);

	if(time_valid && freq_valid) {

	float diff_tf = fabsf(hr_time_result.hr_bpm - hr_fft_result.hr_bpm);

	float agree =
			1.0f - (diff_tf / 20.0f);

	if(agree < 0.0f){
		agree = 0.0f;
	}

	if(agree > 1.0f){
		agree = 1.0f;
	}

	s_time *= agree;

	}

	//Deducting time's sqi based on s_motion
	s_time *= s_motion;

	uint8_t time_bad =
			(hr_time_result.hr_bpm <= 0.0f) ||
			(s_time < HT_CONF_TH) ||
			(hr_time_result.n_peaks < HT_MIN_PEAKS);
	uint8_t freq_bad =
			(hr_fft_result.hr_bpm <= 0.0f) ||
			(s_freq < HF_CONF_TH) ||
			(hr_fft_result.peak_ratio < HF_RATIO_TH);



	r.s_motion = s_motion;
	r.s_time = s_time;
	r.s_freq = s_freq;

	r.time_bad = time_bad;
	r.freq_bad = freq_bad;

	sqi_result = r;
}




