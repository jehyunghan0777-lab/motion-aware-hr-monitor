/*
 * blocke_nlms_anc.c
 *
 *  Created on: May 15, 2026
 *      Author: jehyu
 */
/* ---------------- Includes ---------------- */
#include <stdint.h>
#include <math.h>

#include "main.h"
#include "blockc_data_preprocessing.h"
#include "blockd_motion_metric.h"
#include "blocke_nlms_anc.h"
#include "blockg_sqi.h"

/* ---------------- Defines ---------------- */
#define RB_SIZE                1024U

#define NLMS_WIN_SIZE          100U
#define NLMS_FRAME_STEP        50U

#define NLMS_MU0	           0.005f
#define NLMS_EPS               1.0e-6f

#define NLMS_L_AXIS            4U

#define MOTION_FREEZE_TH       9000U


/* ---------------- Shared Data from BlockA ---------------- */
typedef enum{
	SAMPLE_VALID = 0,
	SAMPLE_INVALID = 1
}sample_status_t;

extern uint32_t last_written_idx;
extern uint8_t rb_has_data;

extern uint32_t idx_rb[];
extern sample_status_t sample_status_rb[];


/* ---------------- BlockE Output ---------------- */
float ppg_clean_rb[RB_SIZE];
float ppg_clean_win[NLMS_WIN_SIZE];

nlms_mode_t nlms_mode = NLMS_MODE_BYPASS;

//Index for synchronizing BlockF HR estimation with BlockE
uint32_t blocke_last_processed_idx = 0;


/* ---------------- Internal States / Variables ---------------- */
//Weights of NLMS
static float w_nlms[NLMS_L_AXIS] = {0.0f};

//X-axis reference signal tapped delay line
static float ax_x_taps[NLMS_L_AXIS] = {0.0f};

static uint32_t last_e_frame_idx = 0;

//For calculating correlation of different IMU axis
/*
float D_corr_ax = 0.0f;
float D_corr_az = 0.0f;
float D_corr_ay = 0.0f;

float D_rms_ax = 0.0f;
float D_rms_ay = 0.0f;
float D_rms_az = 0.0f;

float D_rms_ppg = 0.0f;

static float D_win_ppg[NLMS_WIN_SIZE];
static float D_win_ax[NLMS_WIN_SIZE];
static float D_win_ay[NLMS_WIN_SIZE];
static float D_win_az[NLMS_WIN_SIZE];
*/


/* ---------------- Function Prototypes ---------------- */
static void nlms_shift_reference(float new_x, float *taps);
static float nlms_dot_product(void);
static float nlms_reference_power(void);
static void nlms_update_weights(float e, float norm);

//Functions for calculating IMU axis correlation
//static float window_corr(const float *a, const float *b, uint32_t n)
//static float window_rms(const float *x, uint32_t n)


/* ---------------- Functions ---------------- */

static void nlms_shift_reference(float new_x, float *taps){

	for(int i = NLMS_L_AXIS - 1; i > 0; i--){
		taps[i] = taps[i-1];
	}

	taps[0] = new_x;
}

static float nlms_dot_product(void){

	float y = 0.0f;


	for(uint32_t i = 0; i < NLMS_L_AXIS; i++){
		y += w_nlms[i]*ax_x_taps[i];
	}

	return y;
}

static float nlms_reference_power(void){
	float p = NLMS_EPS;

	for(uint32_t i = 0; i < NLMS_L_AXIS; i++){
		p += ax_x_taps[i]*ax_x_taps[i];
	}

	return p;
}

static void nlms_update_weights(float e, float norm){
	float mu_norm = NLMS_MU0 / norm;

	for(uint32_t i = 0; i < NLMS_L_AXIS; i++){
		w_nlms[i] += mu_norm*e*ax_x_taps[i];
	}

}

//Functions for calculating correlations of different IMU axis with PPG
/*
static float window_corr(const float *a, const float *b, uint32_t n)
{
    float mean_a = 0.0f;
    float mean_b = 0.0f;

    for(uint32_t i = 0; i < n; i++){
        mean_a += a[i];
        mean_b += b[i];
    }

    mean_a /= (float)n;
    mean_b /= (float)n;

    float num = 0.0f;
    float den_a = 0.0f;
    float den_b = 0.0f;

    for(uint32_t i = 0; i < n; i++){
        float da = a[i] - mean_a;
        float db = b[i] - mean_b;

        num   += da * db;
        den_a += da * da;
        den_b += db * db;
    }

    float den = sqrtf(den_a * den_b);

    if(den < NLMS_EPS){
        return 0.0f;
    }

    return num / den;
}

static float window_rms(const float *x, uint32_t n)
{
    float acc = 0.0f;

    for(uint32_t i = 0; i < n; i++){
        acc += x[i] * x[i];
    }

    return sqrtf(acc / (float)n);
}
*/

void BlockE_NLMS_ANC(void) {

	if(!rb_has_data){
		return;
	}

	uint32_t cur = last_written_idx;

	//Underflow Guard
	if(cur < NLMS_WIN_SIZE){
		return;
	}

	//Scheduling Guard
	if((cur - last_e_frame_idx) < NLMS_FRAME_STEP){
		return;
	}

	uint32_t start_idx = last_e_frame_idx + 1U;

	if(motion_state == MOTION_LOW){
		nlms_mode = NLMS_MODE_BYPASS;
	}
	else{
		if(motion_state == MOTION_MODERATE){
			nlms_mode = NLMS_MODE_ADAPT;
		}

	else{
		nlms_mode = NLMS_MODE_FREEZE;
	}
	}

	for(uint32_t idx = start_idx; idx <= cur; idx++){

		uint32_t r = idx % RB_SIZE;

		float clean = 0.0f;

		if(idx_rb[r] != idx || sample_status_rb[r] != SAMPLE_VALID){

			ppg_clean_rb[r] = 0.0f;

			//IMU axis correlation debug windows
			/*
			D_win_ppg[i] = 0.0f;
			D_win_ax[i]  = 0.0f;
			D_win_ay[i]  = 0.0f;
			D_win_az[i]  = 0.0f;
			*/
			continue;
		}

		float d = ppg_bpf_rb[r];

		float ax = ax_bpf_rb[r];

		// Correlation debug windows
		/*
		D_win_ppg[i] = d;
		D_win_ax[i]  = ax;
		D_win_ay[i]  = ay;
		D_win_az[i]  = az;
		*/

		nlms_shift_reference(ax, ax_x_taps);

		if(nlms_mode == NLMS_MODE_BYPASS){
			clean = d;

		}
		else {

			float v_hat = nlms_dot_product();

			clean = d - v_hat;


			if(nlms_mode == NLMS_MODE_ADAPT){

				float norm = nlms_reference_power();

				nlms_update_weights(clean, norm);


			}

		}//end else

			ppg_clean_rb[r] = clean;



	}//end for loop

	uint32_t win_start_idx = cur - NLMS_WIN_SIZE + 1U;

	for(uint32_t i = 0; i < NLMS_WIN_SIZE; i++){

		uint32_t idx = win_start_idx + i;
		uint32_t r = idx % RB_SIZE;

		if(idx_rb[r] != idx || sample_status_rb[r] != SAMPLE_VALID){

			  ppg_clean_win[i] = 0.0f;

			  continue;

		}

		ppg_clean_win[i] = ppg_clean_rb[r];


	}

	//For storing calculating correlation of IMU axis
	/*
	D_corr_ax = window_corr(D_win_ax, D_win_ppg, NLMS_WIN_SIZE);
	D_corr_ay = window_corr(D_win_ay, D_win_ppg, NLMS_WIN_SIZE);
	D_corr_az = window_corr(D_win_az, D_win_ppg, NLMS_WIN_SIZE);

	D_rms_ax = window_rms(D_win_ax, NLMS_WIN_SIZE);
	D_rms_ay = window_rms(D_win_ay, NLMS_WIN_SIZE);
	D_rms_az = window_rms(D_win_az, NLMS_WIN_SIZE);
	D_rms_ppg = window_rms(D_win_ppg, NLMS_WIN_SIZE);
	 */

		blocke_last_processed_idx = cur;
		last_e_frame_idx = cur;
}
