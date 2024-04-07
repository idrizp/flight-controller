/*
 * sensor.c
 *
 *  Created on: Apr 6, 2024
 *      Author: idrizpelaj
 */

#include "sensor.h"
#include "icm42688.h"
#include "icp10100.h"
#include "filter.h"

#define SAMPLE_PERIOD 1/200 // The sampling rate is 200Hz
#define G 9.80665 // gravitational acceleration constant

// START lpfs
lpf_filter lpf_acc[3] = {
		{.beta=0.5, .prev_out=0},
		{.beta=0.5, .prev_out=0},
		{.beta=0.5, .prev_out=0}
};

lpf_filter lpf_gyr[3] = {
		{.beta=0.5, .prev_out=0},
		{.beta=0.5, .prev_out=0},
		{.beta=0.5, .prev_out=0}
};
// END lpfs


// Updates the accelerometer
void Accelerometer_Update(accelerometer* accel) {
	float accelX, accelY, accelZ, gyroX, gyroY, gyroZ;
	ICM42688_ReadAccelGyro(accelX, accelY, accelZ, gyroX, gyroY, gyroZ);

	// Low pass filter the accelerometer values
	lpf(&lpf_acc[0], &accelX);
	lpf(&lpf_acc[1], &accelY);
	lpf(&lpf_acc[2], &accelZ);

	// Update accelerometer values with the low pass filtered ones
	accel->accel[0] = accelX;
	accel->accel[1] = accelY;
	accel->accel[2] = accelZ;

	// Low pass filter the gyro readings
	lpf(&lpf_gyr[0], &gyroX);
	lpf(&lpf_gyr[1], &gyroY);
	lpf(&lpf_gyr[2], &gyroZ);

	// Update the gryo readings into our struct
	accel->gyr[0] = gyroX;
	accel->gyr[1] = gyroY;
	accel->gyr[2] = gyroZ;

	float yaw, pitch, roll;

	float phiHat_acc_rad = atanf(accelY / accelZ);
	float thetaHat_acc_rad = asinf(accelX / G);

}

// Updates the barometer
void Barometer_Update(barometer* bar) {
	float data[2];
	ICP10100_ReadData(ICP10100_MEAS_NORMAL_P_FIRST, &data);

	bar->pressure = data[0];
	bar->temperature = data[1];
	bar->altitude = ICP10100_ComputeAltitude(data[1], data[0]);
}

