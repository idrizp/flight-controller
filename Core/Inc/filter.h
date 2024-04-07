/*
 * filter.h
 *
 *  Created on: Apr 7, 2024
 *      Author: idrizpelaj
 */

#ifndef INC_FILTER_H_
#define INC_FILTER_H_

typedef struct lpf_filter {
	float prev_out;
	float beta;
} lpf_filter;

// Low-pass filters any input values
void lpf(lpf_filter *filter, float *value);


#endif /* INC_FILTER_H_ */
