/*
 * hr_calc.h
 *
 *  Created on: 2021Äê12ÔÂ15ÈÕ
 *      Author: jingye_lab_win
 *      revised from https://github.com/MaximIntegratedRefDesTeam/RD117_ARDUINO/blob/master/algorithm.cpp
 */

#ifndef HR_CALC_H_
#define HR_CALC_H_

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

#define true 1
#define false 0
#define FS 12.5                  // sampling frequency 12.5,25,50,100
#define MIN_DISTANCE  (int)(FS/12.5*2)             // 12.5: 2,25: 4, 50:8, 100:16
#define BUFFER_SIZE  (int)(FS*4)
#define MA4_SIZE  4                         // DONOT CHANGE
#define min(x,y) ((x) < (y) ? (x) : (y))

static  int32_t an_x[ BUFFER_SIZE]; //ir
static  int32_t an_y[ BUFFER_SIZE]; //red

void maxim_heart_rate_and_oxygen_saturation(uint32_t *pun_ir_buffer, int32_t n_ir_buffer_length, uint32_t *pun_red_buffer, int32_t *pn_heart_rate, int8_t *pch_hr_valid);
void maxim_find_peaks(int32_t *pn_locs, int32_t *n_npks,  int32_t  *pn_x, int32_t n_size, int32_t n_min_height, int32_t n_min_distance, int32_t n_max_num);
void maxim_peaks_above_min_height(int32_t *pn_locs, int32_t *n_npks,  int32_t  *pn_x, int32_t n_size, int32_t n_min_height);
void maxim_remove_close_peaks(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, int32_t n_min_distance);
void maxim_sort_ascend(int32_t  *pn_x, int32_t n_size);
void maxim_sort_indices_descend(int32_t  *pn_x, int32_t *pn_indx, int32_t n_size);

#endif /* HR_CALC_H_ */
