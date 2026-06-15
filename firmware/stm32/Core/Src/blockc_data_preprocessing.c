/*
 * blockc_data_preprocessing.c
 *
 *  Created on: May 14, 2026
 *      Author: jehyu
 */
/* ---------------- Includes ---------------- */
#include "main.h"
#include "blockc_data_preprocessing.h"

/* ---------------- Defines ---------------- */
#define RB_SIZE              1024U

#define PPG_HPF_ALPHA        0.97f
#define PPG_BPF_LPF_ALPHA    0.20f
#define IMU_HPF_ALPHA        0.95f
#define IMU_BPF_LPF_ALPHA	 0.25f


/* ---------------- Shared Data from BlockA ---------------- */
typedef enum
{
	SAMPLE_VALID = 0,
	SAMPLE_INVALID = 1
} sample_status_t;

extern uint32_t last_written_idx;

extern uint8_t rb_has_data;

extern uint32_t idx_rb[];
extern sample_status_t sample_status_rb[];

extern int16_t imu_ax_rb[];
extern int16_t imu_ay_rb[];
extern int16_t imu_az_rb[];
extern float imu_mag_rb[];

extern uint32_t ppg_ir_rb[];


/* ---------------- BlockC Output ---------------- */
float ppg_hpf_rb[RB_SIZE];
float ppg_bpf_rb[RB_SIZE];

float imu_hpf_rb[RB_SIZE];
float imu_bpf_rb[RB_SIZE];

float ax_hpf_rb[RB_SIZE];
float ax_bpf_rb[RB_SIZE];


/*
float ax_hpf_rb[RB_SIZE];
float ax_bpf_rb[RB_SIZE];
float ax_hpf_rb[RB_SIZE];
float ax_bpf_rb[RB_SIZE];
*/

/* ---------------- Internal States / Variables ---------------- */
//Struct for coefficients of the biquad filter
typedef struct
{
	float b0, b1, b2;
	float a1, a2;
	float z1, z2;
}biquad_t;


static uint32_t proc_idx = 0;
static uint8_t blockc_started = 0;

//DF2T Biqaud filter coefficients
//hpf biqaud filter: FS=100hz, FC=0.5hz, Q=0.7071, Gain=0db
static biquad_t ppg_hpf_coef = {
		.b0 = 0.9780302754084559f,
		.b1 = -1.9560605508169118f,
		.b2 = 0.9780302754084559f,
		.a1 = -1.9555778328194147f,
		.a2 = 0.9565432688144089f,
		.z1 = 0.0f,
		.z2 = 0.0f
};

//lpf biqaud filter: FS=100hz, FC=5hz, Q=0.7071, Gain=0db
static biquad_t ppg_lpf_coef = {
		.b0 = 0.02008333102602092f,
		.b1 = 0.04016666205204184f,
		.b2 = 0.02008333102602092f,
		.a1 = -1.5610153912536877f,
		.a2 = 0.6413487153577715f,
		.z1 = 0.0f,
		.z2 = 0.0f
};

//hpf biqaud filter: FS=100hz, FC=0.5hz, Q=0.7071, Gain=0db
static biquad_t imu_hpf_coef = {
		.b0 = 0.9780302754084559f,
		.b1 = -1.9560605508169118f,
		.b2 = 0.9780302754084559f,
		.a1 = -1.9555778328194147f,
		.a2 = 0.9565432688144089f,
		.z1 = 0.0f,
		.z2 = 0.0f
};

//lpf biqaud filter: FS=100hz, FC=10hz, Q=0.7071, Gain=0db
static biquad_t imu_lpf_coef = {
		.b0 = 0.06745508395870334f,
		.b1 = 0.13491016791740668f,
		.b2 = 0.06745508395870334f,
		.a1 = -1.1429772843080923f,
		.a2 = 0.41279762014290533f,
		.z1 = 0.0f,
		.z2 = 0.0f
};


//hpf biqaud filter: FS=100hz, FC=0.5hz, Q=0.7071, Gain=0db
static biquad_t ax_hpf_coef = {
		.b0 = 0.9780302754084559f,
		.b1 = -1.9560605508169118f,
		.b2 = 0.9780302754084559f,
		.a1 = -1.9555778328194147f,
		.a2 = 0.9565432688144089f,
		.z1 = 0.0f,
		.z2 = 0.0f
};

//lpf biqaud filter: FS=100hz, FC=10hz, Q=0.7071, Gain=0db
static biquad_t ax_lpf_coef = {
		.b0 = 0.06745508395870334f,
		.b1 = 0.13491016791740668f,
		.b2 = 0.06745508395870334f,
		.a1 = -1.1429772843080923f,
		.a2 = 0.41279762014290533f,
		.z1 = 0.0f,
		.z2 = 0.0f
};

/*
//hpf biqaud filter: FS=100hz, FC=0.5hz, Q=0.7071, Gain=0db
static biquad_t ay_hpf_coef = {
		.b0 = 0.9780302754084559f,
		.b1 = -1.9560605508169118f,
		.b2 = 0.9780302754084559f,
		.a1 = -1.9555778328194147f,
		.a2 = 0.9565432688144089f,
		.z1 = 0.0f,
		.z2 = 0.0f
};

//lpf biqaud filter: FS=100hz, FC=10hz, Q=0.7071, Gain=0db
static biquad_t ay_lpf_coef = {
		.b0 = 0.06745508395870334f,
		.b1 = 0.13491016791740668f,
		.b2 = 0.06745508395870334f,
		.a1 = -1.1429772843080923f,
		.a2 = 0.41279762014290533f,
		.z1 = 0.0f,
		.z2 = 0.0f
};
//hpf biqaud filter: FS=100hz, FC=0.5hz, Q=0.7071, Gain=0db
static biquad_t az_hpf_coef = {
		.b0 = 0.9780302754084559f,
		.b1 = -1.9560605508169118f,
		.b2 = 0.9780302754084559f,
		.a1 = -1.9555778328194147f,
		.a2 = 0.9565432688144089f,
		.z1 = 0.0f,
		.z2 = 0.0f
};

//lpf biqaud filter: FS=100hz, FC=10hz, Q=0.7071, Gain=0db
static biquad_t az_lpf_coef = {
		.b0 = 0.06745508395870334f,
		.b1 = 0.13491016791740668f,
		.b2 = 0.06745508395870334f,
		.a1 = -1.1429772843080923f,
		.a2 = 0.41279762014290533f,
		.z1 = 0.0f,
		.z2 = 0.0f
};
*/

/* ---------------- Functions ---------------- */
//DF2T Biqaud filter
static float biquad_process(biquad_t *s, float x){

	float y = s->b0*x + s->z1;

	s->z1 = s->b1*x - s->a1*y + s->z2;
	s->z2 = s->b2*x - s->a2*y;

	return y;
}


//Block C Main Processor
void BlockC_Data_Preprocessing(void){

	if(!rb_has_data) {
		return;
	}

	uint32_t cur = last_written_idx;

	if(!blockc_started){
		proc_idx = cur;
		blockc_started = 1;
	}

	while(proc_idx <= cur) {

		uint32_t idx = proc_idx;
		uint32_t w = idx % RB_SIZE;

		if(idx_rb[w] != idx || sample_status_rb[w] != SAMPLE_VALID){

			ppg_hpf_rb[w] = 0.0f;
			ppg_bpf_rb[w] = 0.0f;

			imu_hpf_rb[w] = 0.0f;
			imu_bpf_rb[w] = 0.0f;


			ax_hpf_rb[w] = 0.0f;
			ax_bpf_rb[w] = 0.0f;


			//ay_hpf_rb[w] = 0.0f;
			//ay_bpf_rb[w] = 0.0f;


			//az_hpf_rb[w] = 0.0f;
			//az_bpf_rb[w] = 0.0f;

			proc_idx++;
			continue;
		}

		float ppg_raw = (float)ppg_ir_rb[w];
		float imu_raw = imu_mag_rb[w];


		float ax_raw = (float)imu_ax_rb[w];
		//float ay_raw = (float)imu_ay_rb[w];
		//float az_raw = (float)imu_az_rb[w];



		ppg_hpf_rb[w] = biquad_process(&ppg_hpf_coef, ppg_raw);
		ppg_bpf_rb[w] = biquad_process(&ppg_lpf_coef, ppg_hpf_rb[w]);

		imu_hpf_rb[w] = biquad_process(&imu_hpf_coef, imu_raw);
		imu_bpf_rb[w] = biquad_process(&imu_lpf_coef, imu_hpf_rb[w]);


		ax_hpf_rb[w] = biquad_process(&ax_hpf_coef, ax_raw);
		ax_bpf_rb[w] = biquad_process(&ax_lpf_coef, ax_hpf_rb[w]);

		/*
		ay_hpf_rb[w] = biquad_process(&ay_hpf_coef, ay_raw);
		ay_bpf_rb[w] = biquad_process(&ay_lpf_coef, ay_hpf_rb[w]);


		az_hpf_rb[w] = biquad_process(&az_hpf_coef, az_raw);
		az_bpf_rb[w] = biquad_process(&az_lpf_coef, az_hpf_rb[w]);
		*/

		proc_idx++;
	}



}




