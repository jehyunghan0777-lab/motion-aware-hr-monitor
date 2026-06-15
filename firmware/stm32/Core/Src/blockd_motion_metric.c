/*
 * blockd_motion_metric.c
 *
 *  Created on: May 15, 2026
 *      Author: jehyu
 */
/* ---------------- Includes ---------------- */
#include <math.h>
#include <stdint.h>

#include "main.h"
#include "blockc_data_preprocessing.h"
#include "blockd_motion_metric.h"

/* ---------------- Defines ---------------- */
#define RB_SIZE                      1024U

#define MOTION_WIN_SIZE 	         100U
#define MOTION_FRAME_STEP            50U

#define MOTION_THRESHOLD_LOW         700.0f
#define MOTION_THRESHOLD_HIGH        9000.0f


/* ---------------- Shared Data from BlockA ---------------- */
typedef enum {
	SAMPLE_VALID = 0,
	SAMPLE_INVALID = 1
} sample_status_t;

extern uint32_t last_written_idx;
extern uint8_t rb_has_data;

extern uint32_t idx_rb[];
extern sample_status_t sample_status_rb[];


/* ---------------- BlockD Output ---------------- */
float latest_m = 0.0f;
motion_state_t motion_state = MOTION_LOW;


/* ---------------- Internal States / Variables ---------------- */
static uint32_t last_frame_idx = 0;
static float imu_win[MOTION_WIN_SIZE];


/* ---------------- Functions ---------------- */
static float calculate_rms(const float *win, uint32_t n){
	float acc = 0.0f;

	for(uint32_t i = 0; i < n; i++){
		acc += win[i] * win[i];
	}

	return sqrtf(acc / (float) n);
}

void BlockD_Motion_Metric(void){

	if(!rb_has_data){
		return;
	}

	uint32_t cur = last_written_idx;

//UNDERFLOW GUARD: Not enough samples.
	if(cur < MOTION_WIN_SIZE){
		return;
	}

//SCHEDULING GUARD: Only compute every frame step
	if(cur - last_frame_idx < MOTION_FRAME_STEP){
		return;
	}

	uint32_t valid_count = 0;

	for(uint32_t i = 0; i < MOTION_WIN_SIZE; i++){

		uint32_t idx = cur - (MOTION_WIN_SIZE - 1U) + i;
		uint32_t w = idx % RB_SIZE;

		if(idx_rb[w] == idx && sample_status_rb[w] == SAMPLE_VALID){
			imu_win[valid_count] = imu_bpf_rb[w];
			valid_count++;
		}
	}

	if(valid_count < (MOTION_WIN_SIZE*8U)/10U){
		latest_m = 0.0f;
		motion_state = MOTION_LOW;
		last_frame_idx = cur;
	}

	latest_m = calculate_rms(imu_win, valid_count);

	//Motion state decision logic
	if(latest_m < MOTION_THRESHOLD_LOW)
	    motion_state = MOTION_LOW;
	else if(latest_m < MOTION_THRESHOLD_HIGH){
	    motion_state = MOTION_MODERATE;
	}
	else{
	    motion_state = MOTION_HIGH;
	}

	last_frame_idx = cur;
}







