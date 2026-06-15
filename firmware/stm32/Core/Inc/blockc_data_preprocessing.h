/*
 * blockc_data_preprocessing.h
 *
 *  Created on: May 14, 2026
 *      Author: jehyu
 */

#ifndef INC_BLOCKC_DATA_PREPROCESSING_H_
#define INC_BLOCKC_DATA_PREPROCESSING_H_

#include <stdint.h>

void BlockC_Data_Preprocessing(void);

extern float ppg_hpf_rb[];
extern float ppg_bpf_rb[];

extern float imu_hpf_rb[];
extern float imu_bpf_rb[];

extern float ax_hpf_rb[];
extern float ax_bpf_rb[];

#endif /* INC_BLOCKC_DATA_PREPROCESSING_H_ */
