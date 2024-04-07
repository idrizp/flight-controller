/*
 * icp10100.h
 *
 *  Created on: Apr 1, 2024
 *      Author: idrizpelaj
 */

#ifndef INC_ICP10100_H_
#define INC_ICP10100_H_

// Commands
#define ICP10100_I2C_ADDRESS 0x63

// For now, we don't define the commands where the T is measured first. I just don't want to needlessly complicate this driver.
//#define ICP10100_MEAS_LOWPOWER_T_FIRST 0x609C
#define ICP10100_MEAS_LOWPOWER_P_FIRST 0x401A
//#define ICP10100_MEAS_NORMAL_T_FIRST 0x6825
#define ICP10100_MEAS_NORMAL_P_FIRST 0x48A3
//#define ICP10100_MEAS_LOWNOISE_T_FIRST 0x70DF
#define ICP10100_MEAS_LOWNOISE_P_FIRST 0x5059
//#define ICP10100_MEAS_ULTRALOWNOISE_T_FIRST 0x7866
#define ICP10100_MEAS_ULTRALOWNOISE_P_FIRST 0x58E0

// Reads the temperature by the measure command provided: [K] and pressure: [Pa] from the ICP10100
void ICP10100_ReadData(int measure_command, float data[2]);

// From temperature[K] and pressure [Pa], the height(where the pressure sensor is) relative to sea level in [m]
float ICP10100_ComputeAltitude(float temperature, float pressure);

// Sets up the ICP10100 and returns the number of errors.
int ICP10100_Init();

#endif /* INC_ICP10100_H_ */
