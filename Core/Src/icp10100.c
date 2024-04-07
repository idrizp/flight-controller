/*
 * icp10100.c
 *
 *  Created on: Apr 1, 2024
 *      Author: idrizpelaj
 */

#include "icp10100.h"
#include "math.h"
#include "stdint.h"
#include "stm32l0xx.h"

#define ICP10100_I2C_ADDRESS_HAL ICP10100_I2C_ADDRESS << 1 // required to write using HAL
#define ICP10100_HAL_TIMEOUT 100 // in milliseconds
#define P_0 101325 // In Pascals
#define R 8.3144598 // Universal Gas Constant in J/(mol K)
#define G_0 9.80665 // Earth grav. acceleration in ms^-2
#define M 0.0289644 // Molar mass of earth's air

const float z_coeff = R / (G_0 * M);

// The I2C handle for the ICP10100
extern I2C_HandleTypeDef hi2c1;

// Sensor constants.
float sensor_constants[4]; // OTP values
float p_Pa_calib[3];
float LUT_lower;
float LUT_upper;
float quadr_factor;
float offst_factor;

int write_to_i2c(uint8_t data[], int size) {
	return HAL_I2C_Master_Transmit(&hi2c1, ICP10100_I2C_ADDRESS_HAL, data, size, ICP10100_HAL_TIMEOUT);
}

int read_from_i2c(uint8_t data[], int size) {
	return HAL_I2C_Master_Receive(&hi2c1, ICP10100_I2C_ADDRESS_HAL, data, size, ICP10100_HAL_TIMEOUT);
}


// START HELPER FUNCTIONS FROM DATASHEET
// Reads calibration parameters from I2C
int read_otp_from_i2c(short *out)
{
	unsigned char data_write[10];
	unsigned char data_read[10] = {0};
	int status;
	int i;
	// OTP Read mode
	data_write[0] = 0xC5;
	data_write[1] = 0x95;
	data_write[2] = 0x00;
	data_write[3] = 0x66;
	data_write[4] = 0x9C;
	status = write_to_i2c(data_write, 5);
	if (status) return status;
	// Read OTP values
	for (i = 0; i < 4; i++) {
		data_write[0] = 0xC7;
		data_write[1] = 0xF7;
		status = write_to_i2c(data_write, 2);
		if (status) return status;
		status = read_from_i2c(data_read, 3);
		if (status) return status;
		out[i] = data_read[0]<<8 | data_read[1];
	}
return 0;
}
void init_base(short *otp)
{
	int i;
	for(i = 0; i < 4; i++) sensor_constants[i] = (float)otp[i];
	p_Pa_calib[0] = 45000.0;
	p_Pa_calib[1] = 80000.0;
	p_Pa_calib[2] = 105000.0;
	LUT_lower = 3.5 * (1<<20);
	LUT_upper = 11.5 * (1<<20);
	quadr_factor = 1 / 16777216.0;
	offst_factor = 2048.0;
}

// p_Pa -- List of 3 values corresponding to applied pressure in Pa
// p_LUT -- List of 3 values corresponding to the measured p_LUT values at the applied pressures.
void calculate_conversion_constants(float *p_Pa,
float *p_LUT, float *out)
{
	float A,B,C;
	C = (p_LUT[0] * p_LUT[1] * (p_Pa[0] - p_Pa[1]) +
	p_LUT[1] * p_LUT[2] * (p_Pa[1] - p_Pa[2]) +
	p_LUT[2] * p_LUT[0] * (p_Pa[2] - p_Pa[0])) /
	(p_LUT[2] * (p_Pa[0] - p_Pa[1]) +
	p_LUT[0] * (p_Pa[1] - p_Pa[2]) +
	p_LUT[1] * (p_Pa[2] - p_Pa[0]));
	A = (p_Pa[0] * p_LUT[0] - p_Pa[1] * p_LUT[1] - (p_Pa[1] - p_Pa[0]) * C) / (p_LUT[0] - p_LUT[1]);
	B = (p_Pa[0] - A) * (p_LUT[0] + C);
	out[0] = A;
	out[1] = B;
	out[2] = C;
}

// p_LSB -- Raw pressure data from sensor
// T_LSB -- Raw temperature data from sensor
int inv_invpres_process_data(int p_LSB, int T_LSB,
float * pressure, float * temperature)
{
	float t;
	float s1,s2,s3;
	float in[3];
	float out[3];
	float A,B,C;
	t = (float)(T_LSB - 32768);
	s1 = LUT_lower + (float)(sensor_constants[0] * t * t) * quadr_factor;
	s2 = offst_factor * sensor_constants[3] + (float)(sensor_constants[1] * t * t) * quadr_factor;
	s3 = LUT_upper + (float)(sensor_constants[2] * t * t) * quadr_factor;
	in[0] = s1;
	in[1] = s2;
	in[2] = s3;
	calculate_conversion_constants(p_Pa_calib, in, out);
	A = out[0];
	B = out[1];
	C = out[2];
	*pressure = A + B / (C + p_LSB);
	*temperature = -45.f + 175.f/65536.f * T_LSB;
	return 0;
}
// Gets everything up and running
int inv_invpres_init()
{
	short otp[4];
	read_otp_from_i2c(otp);
	init_base(otp);
	return 0;
}

// END HELPER FUNCTIONS FROM DATASHEET


// Reads the data from the ICP10100
void ICP10100_ReadData(int measure_command, float data[2]) {
	uint8_t meas_data[1] = {measure_command};
	// Write the measurement command to the I2C
	write_to_i2c(meas_data, 1);

	uint8_t result_data[6];
	// Read the response back from the I2C.
	// 4 bytes for pressure.
	// 2 bytes for temperature.
	read_from_i2c(result_data, 6);

	uint8_t p_LSB = result_data[3];
	uint8_t T_LSB = result_data[5];
	// Perform ops here according to helper functions from the datasheet.
	inv_invpres_process_data(p_LSB, T_LSB, data, data + 1);
}

// Computes the height of the ICP10100
float ICP10100_ComputeAltitude(float temperature, float pressure) {
	float rel = log(pressure / P_0) * (temperature + 273.15);
	return z_coeff * rel;
}

// Readies the ICP10100
int ICP10100_Init() {
	int errors = 0;
	errors += inv_invpres_init();
	return 0;
}
