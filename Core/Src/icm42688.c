/*
 * icm42688.c
 *
 *  Created on: Apr 3, 2024
 *      Author: idrizpelaj
 */

#include <icm42688.h>

extern SPI_HandleTypeDef hspi1;

// Writes a register to the ICM42688
HAL_StatusTypeDef ICM42688_WriteReg(uint8_t address, uint8_t value) {
	HAL_GPIO_WritePin(GPIOB, ICM42688_SPI_SS_PIN, GPIO_PIN_RESET); // Set to LOW when starting write. CS ICM42688.
	uint8_t data[2] = {ICM42688_WREG(address), value};
	HAL_StatusTypeDef def = HAL_SPI_Transmit(&hspi1, data, 2, 100);
	HAL_GPIO_WritePin(GPIOB, ICM42688_SPI_SS_PIN, GPIO_PIN_SET); // Set to HIGH when ending write.
	return def;
}

// Reads a register from the ICM42688
HAL_StatusTypeDef ICM42688_ReadReg(uint8_t address, uint8_t *data, int size) {
	HAL_GPIO_WritePin(GPIOB, ICM42688_SPI_SS_PIN, GPIO_PIN_RESET); // Set to LOW when starting write. CS ICM42688.
	uint8_t recv[1] = {ICM42688_RREG(address)};
	HAL_SPI_Transmit(&hspi1, recv, 2, 100);
	HAL_StatusTypeDef def = HAL_SPI_Receive(&hspi1, data, size, 100);
	HAL_GPIO_WritePin(GPIOB, ICM42688_SPI_SS_PIN, GPIO_PIN_SET); // Set to HIGH when ending write.
	return def;
}

int ICM42688_ReadAccelGyro(
		float *accelX, // The acceleration on X
		float *accelY, // The acceleration on Y
		float *accelZ, // The acceleration on Z
		float *gyroX, // The gyro measurement on X direction
		float *gyroY, // The gyro measurement on Y direction
		float *gyroZ // The gyro measurement on Z direction
) {
	uint8_t accX1, accX0, accY1, accY0, accZ1, accZ0; // Acceleration
	uint8_t gyrX1, gyrX0, gyrY1, gyrY0, gyrZ1, gyrZ0; // Gyro

	int errors = 0;

	// There doesn't seem to be a better way of doing this.. for now.
	errors += ICM42688_ReadReg(ICM42688_REG_ACC_DATA_X1, &accX1, 1);
	errors += ICM42688_ReadReg(ICM42688_REG_ACC_DATA_X0, &accX0, 1);

	errors += ICM42688_ReadReg(ICM42688_REG_ACC_DATA_Y1, &accY1, 1);
	errors += ICM42688_ReadReg(ICM42688_REG_ACC_DATA_Y0, &accY0, 1);

	errors += ICM42688_ReadReg(ICM42688_REG_ACC_DATA_Z1, &accZ1, 1);
	errors += ICM42688_ReadReg(ICM42688_REG_ACC_DATA_Z0, &accZ0, 1);

	errors += ICM42688_ReadReg(ICM42688_REG_GYR_DATA_X1, &gyrX1, 1);
	errors += ICM42688_ReadReg(ICM42688_REG_GYR_DATA_X1, &gyrX0, 1);

	errors += ICM42688_ReadReg(ICM42688_REG_GYR_DATA_Y1, &gyrY1, 1);
	errors += ICM42688_ReadReg(ICM42688_REG_GYR_DATA_Y0, &gyrY0, 1);

	errors += ICM42688_ReadReg(ICM42688_REG_GYR_DATA_Z1, &gyrZ1, 1);
	errors += ICM42688_ReadReg(ICM42688_REG_GYR_DATA_Z0, &gyrZ0, 1);

	// Now that we've read the data, we can convert it to floats appropriately.

	uint16_t accX = accX1 | accX0;
	uint16_t accY = accY1 | accY0;
	uint16_t accZ = accZ1 | accZ0;

	uint16_t gyrX = gyrX1 | gyrX0;
	uint16_t gyrY = gyrY1 | gyrY0;
	uint16_t gyrZ = gyrZ1 | gyrZ0;

	// TODO: Perhaps there's a better way to convert these values in order to make them a little more realistis?
	// Floating point isn't really that great either way for computations, this should be OK?
	*accelX = (float) accX;
	*accelY = (float) accY;
	*accelZ = (float) accZ;

	*gyroX = (float) gyrX;
	*gyroY = (float) gyrY;
	*gyroZ = (float) gyrZ;

	return errors;
}

// Readies the ICM42688
int ICM42688_Setup() {
	// TODO: Perhaps include some extra things such as settings setup here
	return 0;
}
