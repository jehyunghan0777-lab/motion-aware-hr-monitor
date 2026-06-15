/*
 * blockf_hr_estimation.c
 *
 *  Created on: May 19, 2026
 *      Author: jehyu
 */
/* ---------------- Includes ---------------- */
#include <stdint.h>
#include <math.h>

#include "arm_math.h"

#include "blocke_nlms_anc.h"
#include "blockf_hr_estimation.h"


/* ---------------- Defines ---------------- */
#define RB_SIZE                 1024U
#define HR_WIN_SIZE             512U
#define HR_FRAME_STEP           50U
#define FS_HZ                   100.0f
#define HR_MIN_BPM              40.0f
#define HR_MAX_BPM              130.0f

#define MAX_PEAKS               16U
#define PEAK_NEIGHBOR           7U
#define HR_MIN_PEAKS            4U
#define PEAK_PROM_WIN           20U
#define PEAK_SLOPE_GAP          6U
#define PEAK_THR_SCALE          0.70f
#define HR_MIN_RMS              50.0f
#define PEAK_PROM_SCALE         0.35f
#define PEAK_SLOPE_SCALE        0.02f
#define IBI_OUTLIER_FRAC        0.25f
#define PEAK_MIN_DIST_SAMPLES   ((uint32_t)(FS_HZ * 60.0f / HR_MAX_BPM ))
#define PEAK_MAX_DIST_SAMPLES   ((uint32_t)(FS_HZ * 60.0f / HR_MIN_BPM ))

#define FFT_AVG_FRAMES          3U
#define FFT_HARM_TOL_BINS       1U
#define FFT_MIN_RATIO           1.3f
#define FFT_CONF_SPAN           2.0f
#define FFT_TRACK_ALPHA         0.75f

#define EPS                     1.0e-6f


/* ---------------- Shared Data from BlockA---------------- */
typedef enum{
	SAMPLE_VALID = 0,
	SAMPLE_INVALID = 1
} sample_status_t;

extern uint8_t rb_has_data;

extern uint32_t idx_rb[];
extern sample_status_t sample_status_rb[];


/* ---------------- BlockD Output ---------------- */
hr_time_result_t hr_time_result = {0};
hr_fft_result_t hr_fft_result = {0};


/* ---------------- Internal States / Variables ---------------- */
static uint32_t last_f_frame_idx = 0;

static float ppg_for_time[HR_WIN_SIZE];
static float ppg_hr_win[HR_WIN_SIZE];

static arm_rfft_fast_instance_f32 rfft_inst;
static uint8_t fft_initialized = 0;

static float fft_in[HR_WIN_SIZE];
static float fft_out[HR_WIN_SIZE];
static float fft_mag[(HR_WIN_SIZE/2U) + 1U];
static float fft_mag_avg[(HR_WIN_SIZE/2U) + 1U] = {0.0f};
static float hann[HR_WIN_SIZE];
static uint8_t fft_avg_initialized = 0U;

static float prev_hr_fft_bpm = 0.0f;
static float prev_hr_time_bpm = 0.0f;
static float prev_peak_thr = 0.0f;

static float tracked_hr_fft_bpm = 0.0f;

//TEST for taking snapshot of a single window for Spectrum Analysis
// Adding variables for storing the snapshot data
/*
float snapshot_fft_mag[((HR_WIN_SIZE / 2U) + 1U)];
float snapshot_hrf = 0.0f;
float snapshot_peak_ratio = 0.0f;
float snapshot_conf = 0.0f;
*/


/* ---------------- Function Prototypes ---------------- */
static hr_time_result_t hr_time_estimate(const float *x, uint32_t n, float fs_hz, float prev_hr_bpm);
static uint8_t is_peak_local_max(const float *x, uint32_t i, uint32_t n);
static float peak_prominence(const float *x, uint32_t i, uint32_t n);
static uint8_t peak_slope_ok(const float *x, uint32_t i, uint32_t n, float win_rms);
static float median_float(float *a, uint32_t n);

static hr_fft_result_t hr_fft_estimate(const float *x, uint32_t n, float fs_hz, float prev_hr_bpm);
static uint8_t is_fft_local_peak(const float *mag, uint32_t k, uint32_t k_min, uint32_t k_max);
static void blockf_fft_init(void);
static float fft_parabolic_delta(const float *mag, uint32_t k, uint32_t k_min, uint32_t k_max);
static float fft_harmonic_score(const float *mag, uint32_t k, uint32_t k_max);

static uint8_t extract_clean_hr_window(uint32_t cur);

/* ---------------- Functions ---------------- */
/* ---------------- Time Estimator Functions ---------------- */
static uint8_t is_peak_local_max(const float *x, uint32_t i, uint32_t n){

	if(i < PEAK_NEIGHBOR || i >= (n - PEAK_NEIGHBOR)){
		return 0;
	}

	float center = x[i];

	for(uint32_t k = 1; k <= PEAK_NEIGHBOR; k++){

		if(center <= x[i-k]){
			return 0;
		}

		if(center < x[i+k]){
			return 0;
		}
	}
	return 1;
}

static float peak_prominence(const float *x, uint32_t i, uint32_t n){

	//Finding left and right side of window and guard
	uint32_t l0 = ( i > PEAK_PROM_WIN ) ? (i - PEAK_PROM_WIN) : 0U;
	uint32_t r1 = ( (i + PEAK_PROM_WIN) < (n - 1U) ) ? (i + PEAK_PROM_WIN) : (n - 1U);

	//Initializing left and right min with peak value
	float left_min = x[i];
	float right_min = x[i];

	//Finding the minimum for left and right
	for(uint32_t j = l0; j < i; j++){
		if(x[j] < left_min){
			left_min = x[j];
		}
	}

	for(uint32_t j = i + 1U; j <= r1; j++){
		if(x[j] < right_min){
			right_min = x[j];
		}
	}

	float valley = (left_min > right_min) ? left_min : right_min;
	return x[i] - valley;

}

static uint8_t peak_slope_ok(const float *x, uint32_t i, uint32_t n, float win_rms){

	//Guard
	if( i < PEAK_SLOPE_GAP || (i + PEAK_SLOPE_GAP) >= n){
		return 0;
	}

	float rise = x[i] - x[i- PEAK_SLOPE_GAP];
	float fall = x[i] - x[i + PEAK_SLOPE_GAP];

	float slope_min = PEAK_SLOPE_SCALE * win_rms;

	return ( (rise > slope_min) && (fall > slope_min) );

}

static float median_float(float *a, uint32_t n){

	for(uint32_t i = 0; i < n; i++){
		for(uint32_t j = i + 1U; j < n; j++){
			if(a[j] < a[i]){
				float tmp = a[i];
				a[i] = a[j];
				a[j] = tmp;
			}
		}
	}

	if(n == 0U){
		return 0.0f;
	}

	if((n%2U) == 1U){
		return a[n/2U];
	}

	return 0.5f * (a[(n/2U) - 1U] + a[n/2U]);

}


static hr_time_result_t hr_time_estimate(const float *x, uint32_t n, float fs_hz, float prev_hr_bpm){

	hr_time_result_t r = {0};

	//Calculating Mean
	float mean = 0.0f;

	for(uint32_t i = 0; i < n; i++){
		mean += x[i];
	}

	mean /= (float)n;

	float acc = 0.0f;

	//Mean Removal
	for(uint32_t i = 0; i < n; i++){
		ppg_for_time[i] = x[i] - mean;
		acc += ppg_for_time[i] * ppg_for_time[i];
	}

	//Calculating RMS
	float win_rms = sqrtf(acc/(float)n);

	//Guard
	if(win_rms < EPS){
		return r;
	}

	if(win_rms < HR_MIN_RMS ){
		return r;
	}

	float current_thr = PEAK_THR_SCALE * win_rms;

	float thr;

	if(prev_peak_thr <= 0.0f){
		thr = current_thr;
	}
	else{
		thr = 0.8f * prev_peak_thr + 0.2f * current_thr;
	}

	prev_peak_thr = thr;


	uint32_t peaks[MAX_PEAKS];
	uint32_t n_peaks = 0;

	uint32_t cand_idx = 0U;
	float cand_score = 0.0f;
	uint8_t have_cand = 0U;

	for(uint32_t i = 1; i < (n - 1U); i++){

	    float b = ppg_for_time[i];

		if(!is_peak_local_max(ppg_for_time, i, n)){
			continue;
		}
		if(b <= thr){
			continue;
		}

		float prom = peak_prominence(ppg_for_time, i, n);

		if(prom < (PEAK_PROM_SCALE * win_rms)){
			continue;
		}

		if(!peak_slope_ok(ppg_for_time, i, n, win_rms)){
			continue;
		}

		float score = b + prom;

		if(!have_cand){
			cand_idx = i;
			cand_score = score;
			have_cand = 1U;
		}
		else{
			if((i - cand_idx) < PEAK_MIN_DIST_SAMPLES ){
				if(score > cand_score){
					cand_idx = i;
					cand_score = score;
				}//end if1
			}//end if2
			else{
				if(n_peaks < MAX_PEAKS){
					peaks[n_peaks] = cand_idx;
					n_peaks++;
				}//end if

				cand_idx = i;
				cand_score = score;
			}//end else1
		}//end else2
	}

	if(have_cand && n_peaks < MAX_PEAKS){
		peaks[n_peaks] = cand_idx;
		n_peaks++;
	}



	r.n_peaks = n_peaks;

	if(n_peaks < HR_MIN_PEAKS){
		return r;
	}


	float ibi[MAX_PEAKS];
	uint32_t n_ibi = 0U;

	for(uint32_t i = 1U; i < n_peaks; i++){

		float d  = (float)peaks[i] - (float)peaks[i-1U];

		if( d>= PEAK_MIN_DIST_SAMPLES && d <= PEAK_MAX_DIST_SAMPLES){
			if(n_ibi < MAX_PEAKS){
				ibi[n_ibi] = d;
				n_ibi++;
			}
		}
	}

	if(n_ibi == 0U){
		return r;
	}

	float ibi_copy[MAX_PEAKS];

	for(uint32_t i = 0; i < n_ibi; i++){
		ibi_copy[i] = ibi[i];
	}

	float median_d = median_float(ibi_copy, n_ibi);

	float sum_d = 0.0f;
	float sum_d2 = 0.0f;
	uint32_t k = 0U;

	for(uint32_t i = 0; i < n_ibi; i++){

		float d = ibi[i];

		if(fabsf(d - median_d) > (IBI_OUTLIER_FRAC * median_d)){
			continue;
		}

		sum_d += d;
		sum_d2 += d * d;
		k++;
	}

	if(k == 0U){
		return r;
	}

	float mean_d = sum_d / (float)k;
	float var_d = (sum_d2 / (float)k) - (mean_d * mean_d);

	if(var_d < 0.0f){
		var_d = 0.0f;
	}

	float hr = 60.0f * fs_hz / median_d;

	if( (hr < HR_MIN_BPM) || (hr > HR_MAX_BPM) ){

		return r;

	}


	//IBI Regularity Score
	float score_ibi = 1.0f;
	float rel_var = var_d / ( (mean_d * mean_d) + EPS);
	score_ibi -= rel_var;
	if(score_ibi < 0.0f) {
		score_ibi = 0.0f;
	}

	if(score_ibi > 1.0f) {
		score_ibi = 1.0f;
	}
	//Continuity Score
	float score_cont = 1.0f;
	if(prev_hr_bpm > 0.0f){

	    float diff = fabsf(hr - prev_hr_bpm);
        score_cont -= diff / 25.0f;
	}
	if(score_cont < 0.0f) {
		score_cont = 0.0f;
	}

	if(score_cont > 1.0f) {
		score_cont = 1.0f;
	}


	//Peak Count Score
	float expected_peaks = (hr / 60.0f) * ((float)n / fs_hz);
	float peak_error = fabsf( (float)n_peaks - expected_peaks );
	float score_peaks = 1.0f - (peak_error / 2.0f);

	if(score_peaks < 0.0f) {
		score_peaks = 0.0f;
	}

	if(score_peaks > 1.0f) {
		score_peaks = 1.0f;
	}

	float conf =
			0.5f * score_ibi +
			0.3f * score_peaks +
			0.2f * score_cont;

	if(conf < 0.0f) {
		conf = 0.0f;
	}

	if(conf > 1.0f) {
		conf = 1.0f;
	}

	r.hr_bpm = hr;
	r.conf = conf;
	r.ibi_var = var_d;


	return r;

}

/* ---------------- FFT Estimator Functions ---------------- */
static void blockf_fft_init(void){

	if(fft_initialized){
		return;
	}

	arm_status st;

	//Initializing
	st = arm_rfft_fast_init_f32(&rfft_inst, HR_WIN_SIZE);

	if(st != ARM_MATH_SUCCESS){
		return;
	}

	//Initializing Hanning Filter
	for(uint32_t i = 0; i < HR_WIN_SIZE; i++){

		hann[i] = 0.5f - 0.5f*cosf( (2.0f * 3.14159265359f * (float)i) / ( (float)HR_WIN_SIZE - 1.0f ));

	}

	fft_initialized = 1;

}

static float fft_parabolic_delta(const float *mag, uint32_t k, uint32_t k_min, uint32_t k_max){

	if(k <= k_min || k >= k_max){
		return 0.0f;
	}

	float left = mag[k-1];
	float center = mag[k];
	float right = mag[k+1];

	float denom = left - (2.0f * center) + right;

	if(fabsf(denom) < EPS){
		return 0.0f;
	}

	float delta = 0.5f * (left - right) / denom;

	if(delta > 0.5f){
		delta = 0.5f;
	}

	if(delta < -0.5f){
		delta = -0.5f;
	}

	return delta;

}

static uint8_t is_fft_local_peak(const float *mag, uint32_t k, uint32_t k_min, uint32_t k_max){

	float v = mag[k];

	if(k > k_min){
		if(v <= mag[k-1U]){
			return 0;
		}
	}

	if(k < k_max){
		if(v < mag[k+1U]){
			return 0;
		}
	}

	return 1;
}

static float fft_harmonic_score(const float *mag, uint32_t k, uint32_t k_max){

	uint32_t h = k * 2U;

	if(h > k_max){
		return 0.5f; //Neutral output as harmonic outside of scale
	}

	uint32_t h0 = (h > FFT_HARM_TOL_BINS) ? (h - FFT_HARM_TOL_BINS) : h;
	uint32_t h1 = ((h + FFT_HARM_TOL_BINS) <= k_max) ? (h + FFT_HARM_TOL_BINS) : k_max;

	float harm_mag = 0.0f;

	for(uint32_t j = h0; j <= h1; j++){
		if(mag[j] > harm_mag){
			harm_mag = mag[j];
		}
	}

	float ratio = harm_mag / (mag[k] + EPS);

	float score = ratio / 0.5f; //Harmonic at 50% fundamental which is strong

	if(score > 1.0f){
		score = 1.0f;
	}
	if(score < 0.0f){
		score = 0.0f;
	}

	return score;
}

static hr_fft_result_t hr_fft_estimate(const float *x, uint32_t n, float fs_hz, float prev_hr_bpm){

	hr_fft_result_t r = {0};

	//Scheduling Guard
	if(n != HR_WIN_SIZE){
		return r;
	}

	//Calculate mean
	float mean = 0.0f;

	for(uint32_t i = 0; i < n; i++){
		mean += x[i];
	}

	mean /= (float)n;

	//Removing DC Offset and applying Hanning filter
	for(uint32_t i = 0; i < n; i++){
		fft_in[i] = (x[i] - mean) * hann[i];
	}

	//Calculating Window RMS with guard check for minimum RMS
	float acc = 0.0f;

	for(uint32_t i = 0; i < n; i++){
		float v = x[i] - mean;
		acc += v*v;
	}

	float win_rms = sqrtf(acc/(float)n);

	if(win_rms < 50.0f) {
		return r;
	}

	//Processing FFT
	arm_rfft_fast_f32(&rfft_inst, fft_in, fft_out, 0);

	//Get DC magnitude
	fft_mag[0] = fabsf(fft_out[0]);

	//Get magnitude for rest of the bins
	for(uint32_t k = 1; k < (n/2U); k++){

		float re = fft_out[2U * k];
		float im = fft_out[(2U*k) + 1U];

		fft_mag[k] = sqrtf( (re*re) + (im*im) );
	}

	//Get magnitude at Nyquist bin
	fft_mag[n/2U] = fabsf(fft_out[1]);


	//Multi-frame spectral averaging


	if(!fft_avg_initialized){

		for(uint32_t k = 0U; k <= (n/2U); k++){
			fft_mag_avg[k] = fft_mag[k];
		}

		fft_avg_initialized = 1U;
	}
	else {

		for(uint32_t k = 0U; k <= (n/2U); k++){
			fft_mag_avg[k] =
			((float)(FFT_AVG_FRAMES - 1U) / (float)FFT_AVG_FRAMES) * fft_mag_avg[k] +
			(1.0f / (float)FFT_AVG_FRAMES) * fft_mag[k];

		}

	}


	float hr_min_hz = HR_MIN_BPM / 60.0f;
	float hr_max_hz = HR_MAX_BPM / 60.0f;

	//Get Max and Min bins
	uint32_t k_min = (uint32_t)ceilf((hr_min_hz * (float)n) / fs_hz);
	uint32_t k_max = (uint32_t)floorf((hr_max_hz * (float)n) / fs_hz);

	//Clamp bins
	if(k_min < 1U) {
		k_min = 1U;
	}

	if(k_max > n/2U){
		k_max = n/2U;
	}

	//Invalid guard for bins
	if(k_min > k_max) {
		return r;
	}

	//Convert previous HR BPM to frequency Hz
	float prev_hz = (prev_hr_bpm > 0.0f) ? (prev_hr_bpm / 60.0f) : 0.0f;
	uint32_t prev_k = 0U;

	if(prev_hz > 0.0f){
		prev_k = (uint32_t)roundf((prev_hz * (float)n) / fs_hz);
	}

	float best_score = 0.0f;
	float best_mag = 0.0f;
	uint32_t best_k = 0U;

	float band_sum = 0.0f;
	uint32_t band_cnt = 0U;

	for(uint32_t k = k_min; k <= k_max; k++){

		float mag = fft_mag_avg[k];
		float score = mag;

		if(prev_k != 0U){
			uint32_t dk = (k > prev_k) ? (k - prev_k) : (prev_k - k);

			//Mild Continuity Bias
			float bias = 1.0f / (1.0f + 0.2f * (float)dk);
			score *= bias;
		}

		if(score > best_score){
			best_score = score;
			best_mag = mag;
			best_k = k;
		}

		band_sum += mag;
		band_cnt ++;
	}

	if(best_k == 0U || band_cnt == 0U){
		return r;
	}


	//Harmonic Checking
	float score_harm = fft_harmonic_score(fft_mag_avg, best_k, n/2U);

	if(score_harm < 0.0f) {
		score_harm = 0.0f;
	}

	if(score_harm > 1.0f){
		score_harm = 1.0f;
	}


	uint8_t fft_local_ok = is_fft_local_peak(fft_mag_avg, best_k, k_min, k_max);

/*
	//Get frequency of the best bin
	float peak_f = ((float)best_k * fs_hz)/(float)n;
	//Get BPM from peak frequency
	float hr = peak_f * 60.0f;
*/

	float delta = fft_parabolic_delta(fft_mag_avg, best_k, k_min, k_max);

	float peak_bin = (float)best_k + delta;

	float peak_f = (peak_bin * fs_hz) / (float)n;
	float hr = peak_f * 60.0f;

	//Clamp guard
	if(hr > HR_MAX_BPM || hr < HR_MIN_BPM){
		return r;
	}

	//Temporal Tracking


	float hr_tracked = 0.0f;



		hr_tracked = hr;

		if(tracked_hr_fft_bpm > 0.0f){
				hr_tracked =
						FFT_TRACK_ALPHA * tracked_hr_fft_bpm +
						(1.0f - FFT_TRACK_ALPHA) * hr;
			}


	float band_mean = band_sum / (float)band_cnt;
	float ratio = (band_mean > EPS) ? (best_mag / band_mean) : 0.0f;

	if(ratio < FFT_MIN_RATIO){
		return r;
	}

	float score_ratio  = (ratio - FFT_MIN_RATIO) / FFT_CONF_SPAN;

	if(score_ratio < 0.0f) {
		score_ratio = 0.0f;
	}

	if(score_ratio > 1.0f){
		score_ratio = 1.0f;
	}


	float score_local = fft_local_ok ? 1.0f : 0.0f;

	float conf =
			0.60f * score_ratio +
			0.25f * score_harm +
			0.15f * score_local;

	//Clamp conf
	if(conf < 0.0f) {
		conf = 0.0f;
	}

	if(conf > 1.0f){
		conf = 1.0f;
	}


	r.hr_bpm = hr_tracked;
	r.conf = conf;
	r.k_peaks = best_k;
	r.peak_hz = peak_f;
	r.peak_ratio = ratio;


	return r;
}


static uint8_t extract_clean_hr_window(uint32_t cur){

	if(cur < HR_WIN_SIZE){
		return 0;
	}

	uint32_t start_idx = cur - HR_WIN_SIZE + 1U;

	for(uint32_t i = 0; i < HR_WIN_SIZE; i++){

		uint32_t idx = start_idx + i;
		uint32_t r = idx % RB_SIZE;

		if(idx_rb[r] != idx || sample_status_rb[r] != SAMPLE_VALID){
			return 0;
		}

		ppg_hr_win[i] = ppg_clean_rb[r];

	}

	return 1;
}


void BlockF_HR_Estimation(void){

	if(!rb_has_data){
		return;
	}

	uint32_t cur = blocke_last_processed_idx;

	if(cur == 0U){
		return;
	}

	if(cur < HR_WIN_SIZE ){
		return;
	}

	if((cur - last_f_frame_idx) < HR_FRAME_STEP){
		return;
	}

	hr_time_result = (hr_time_result_t){0};
	hr_fft_result = (hr_fft_result_t){0};



	if(!extract_clean_hr_window(cur)){
		return;
	}


	blockf_fft_init();
	if(!fft_initialized){
		return;
	}

	hr_time_result = hr_time_estimate(ppg_hr_win, HR_WIN_SIZE, FS_HZ, prev_hr_time_bpm);

	hr_fft_result = hr_fft_estimate(ppg_hr_win, HR_WIN_SIZE, FS_HZ, prev_hr_fft_bpm);

		//For taking a snapshot of frequency magnitudes for analysis
		/*
	    for(uint32_t i = 0; i < ((HR_WIN_SIZE / 2U) + 1U); i++){
	        snapshot_fft_mag[i] = fft_mag[i];
	    }

	    snapshot_hrf = hr_fft_result.hr_bpm;
	    snapshot_peak_ratio = hr_fft_result.peak_ratio;
	    snapshot_conf = hr_fft_result.conf;
		*/



	if(hr_time_result.conf > 0.5f){
				prev_hr_time_bpm = hr_time_result.hr_bpm;
			}


	if(hr_fft_result.conf > 0.5f){
			prev_hr_fft_bpm = hr_fft_result.hr_bpm;
			tracked_hr_fft_bpm = hr_fft_result.hr_bpm;
		}


	last_f_frame_idx = cur;


}





