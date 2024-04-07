/*
 * sensor.h
 *
 *  Created on: Apr 6, 2024
 *      Author: idrizpelaj
 */

#ifndef INC_SENSOR_H_
#define INC_SENSOR_H_

#include "stdint.h"

typedef struct accelerometer {
	float accel[3]; // x y z [ms^-2]
	float gyr[3]; // x y z [rad*s^-1]

	float yaw; // [rad]
	float pitch; // [rad]
	float roll; // [rad]

	uint8_t falling; // bool
} accelerometer;

typedef struct barometer {
	float pressure; // [Pa]
	float temperature; // [C]

	float altitude; // [m]
} barometer;

void Accelerometer_Update(accelerometer* accelerometer);
void Barometer_Update(barometer* barometer);

#endif /* INC_SENSOR_H_ */
