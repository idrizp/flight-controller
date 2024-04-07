/*
 * filter.c
 *
 *  Created on: Apr 7, 2024
 *      Author: idrizpelaj
 */

#include "filter.h"

// First order low pass filter
void lpf(lpf_filter* filter, float *value) {
	*value = (filter->beta) * (*value) + (1-filter->beta) * filter->prev_out;
	filter->prev_out = *value;
}
