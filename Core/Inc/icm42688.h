/*
 * icm42688.h
 *
 *  Created on: Apr 3, 2024
 *      Author: idrizpelaj
 */

#ifndef INC_ICM42688_H_
#define INC_ICM42688_H_

#include "stm32l0xx.h"

#define ICM42688_SPI_SS_PIN GPIO_PIN_9

#define ICM42688_RREG(REGISTER) 0 << 7 | REGISTER // Converts a register to a READ register header
#define ICM42688_WREG(REGISTER) 1 << 7 | REGISTER // Converts a register to a WRITE register header

// START USER BASE 0 REGISTERS
#define ICM42688_REG_ACC_DATA_X1 0x1F
#define ICM42688_REG_ACC_DATA_X0 0x20

#define ICM42688_REG_ACC_DATA_Y1 0x21
#define ICM42688_REG_ACC_DATA_Y0 0x22

#define ICM42688_REG_ACC_DATA_Z1 0x23
#define ICM42688_REG_ACC_DATA_Z0 0x24

#define ICM42688_REG_GYR_DATA_X1 0x25
#define ICM42688_REG_GYR_DATA_X0 0x26

#define ICM42688_REG_GYR_DATA_Y1 0x27
#define ICM42688_REG_GYR_DATA_Y0 0x28

#define ICM42688_REG_GYR_DATA_Z1 0x29
#define ICM42688_REG_GYR_DATA_Z0 0x2A
// END USER BASE 0 REGISTERS

// Writes a register to the ICM42688. Returns the number of errors as an int.
HAL_StatusTypeDef ICM42688_WriteReg(uint8_t address, uint8_t value);

// Reads a register from the ICM42688. Returns the number of errors as an int.
HAL_StatusTypeDef ICM42688_ReadReg(uint8_t address, uint8_t *data, int size);

int ICM42688_ReadAccelGyro(
		float *accelX, // The acceleration on X
		float *accelY, // The acceleration on Y
		float *accelZ, // The acceleration on Z
		float *gyroX, // The gyro measurement on X direction
		float *gyroY, // The gyro measurement on Y direction
		float *gyroZ // The gyro measurement on Z direction
);

// Readies the ICM42688
int ICM42688_Setup();

#endif /* INC_ICM42688_H_ */
