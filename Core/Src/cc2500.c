/*
 * cc2500.c
 *
 *  Created on: Mar 29, 2024
 *      Author: idrizpelaj
 */

#include <cc2500.h>
#include <stm32l051xx.h>

#define CC2500_FIFOTHR_OPT 15 // pg. 62 - 15 (1111) - For **now** we want 64B for RX, 1B for TX.
#define CC2500_TIMEOUT 100
#define CC2500_READ_STATE_MASK 3 // 0b00000111 - mask for READ of CC2500 STATE
#define CC2500_READ_BURST 192 // 0b11000000 - Read AND burst
#define CC2500_READ_NO_BURST 128 // 0b10000000 - Read WITHOUT burst
#define CC2500_WRITE_BURST 64 // 0b01000000 - Write AND burst
#define CC2500_WRITE_NO_BURST 0 // 0b0000000 - Write WITHOUT burst
#define CC2500_GPIO_SS_PIN GPIO_PIN_5

extern SPI_HandleTypeDef hspi1;

// Writes to a register of the CC2500.
HAL_StatusTypeDef CC2500_WriteReg(uint8_t addr, uint8_t value) {
	HAL_GPIO_WritePin(GPIOB, CC2500_GPIO_SS_PIN, GPIO_PIN_RESET); // Set CLK to LOW when writing
	uint8_t header = CC2500_WRITE_NO_BURST | addr;
	uint8_t data[2] = {header, value};
	HAL_StatusTypeDef def = HAL_SPI_Transmit(&hspi1, data, 2, CC2500_TIMEOUT);
	HAL_GPIO_WritePin(GPIOB, CC2500_GPIO_SS_PIN, GPIO_PIN_SET); // Set CLK to HIGH after done writing
	return def;
}
// Reads from a register of the CC2500.
HAL_StatusTypeDef CC2500_ReadReg(uint8_t addr, uint8_t* value) {
	HAL_GPIO_WritePin(GPIOB, CC2500_GPIO_SS_PIN, GPIO_PIN_RESET); // Set CLK to LOW when reading
	uint8_t header = CC2500_READ_NO_BURST | addr; // READ and BURST mode activated
	HAL_StatusTypeDef def = HAL_SPI_TransmitReceive(&hspi1, &header, value, 1, CC2500_TIMEOUT);
	HAL_GPIO_WritePin(GPIOB, CC2500_GPIO_SS_PIN, GPIO_PIN_SET); // Set CLK to HIGH after done writing
	return def;
}

// Sends a command probe to the CC2500
enum CC2500_Status CC2500_SendCmdStrobe(uint8_t probe_id) {
	HAL_GPIO_WritePin(GPIOB, CC2500_GPIO_SS_PIN, GPIO_PIN_RESET); // Set CLK to LOW when writing
	uint8_t header = CC2500_WRITE_NO_BURST | probe_id; // Xor with 1(00000001) so that we are in READ mode(one bit less)
	uint8_t statusByte;
	HAL_StatusTypeDef def = HAL_SPI_TransmitReceive(&hspi1, &header, &statusByte, 1, CC2500_TIMEOUT);
	if (def != 0) {
		return CC2500_STATUS_ERROR;
	}
	uint8_t state = statusByte & CC2500_READ_STATE_MASK; // Automatically maps to our enum.. or it should, at least.
	HAL_GPIO_WritePin(GPIOB, CC2500_GPIO_SS_PIN, GPIO_PIN_SET); // Set CLK to HIGH after done writing
	return state;
}


// Reads the current status of the CC2500.
enum CC2500_Status CC2500_ReadStatus() {
	return CC2500_SendCmdStrobe(0x00); // No command probe.
}

// Enters RX mode for the CC2500
void CC2500_EnterRXMode() {
	CC2500_SendCmdStrobe(CC2500_CMDSTROBE_SRX); // Enters the RX mode
}

// Flushes the RX FIFO of the CC2500
void CC2500_FlushRXFIFO() {
	CC2500_SendCmdStrobe(CC2500_CMDSTROBE_SFRX); // Flushes out the CC2500's FIFO
}

void CC2500_ReadRXFIFOBytesInfo(int* num_bytes, int* rx_overflow) {
	uint8_t value;
	CC2500_ReadReg(CC2500_REG_RXBYTES, &value);
	*rx_overflow = value & (1 << 7); // First bit is a 0 for NO OVERFLOW, 1 for OVERFLOW
	// 127 - 0b01111111
	*num_bytes = value & 127; // Last seven bits are the actual value.
}

// Receives a packet from the CC2500.
void CC2500_ReceivePacket(CC2500_Packet *packet) {
	// Handle the FIFO buffer, read the inc packet
	int num_bytes;
	int rx_overflow;

	// Read the RX FIFO bytes
	CC2500_ReadRXFIFOBytesInfo(&num_bytes, &rx_overflow);
	if (rx_overflow) {
		CC2500_FlushRXFIFO();
		CC2500_EnterRXMode();
		return;
	}

	// Take the RX FIFO bytes from the FIFO buffer all in at once.
	uint8_t addr = CC2500_READ_BURST | CC2500_REG_RX_FIFO;

	uint8_t data[6]; // byte 0 is address byte, byte 1 for packet id, 4 bytes for payload
	HAL_SPI_TransmitReceive(&hspi1, &addr, data, 6, 100);

	// The packet is then updated appropriately.
	packet->id = data[1];

	// The payload is handled by the data array.
	packet->payload[0] = data[2];
	packet->payload[1] = data[3];
	packet->payload[2] = data[4];
	packet->payload[3] = data[5];

	CC2500_EnterRXMode(); // Enter RX mode after reading the packet
}

// Initializes the CC2500
uint8_t CC2500_Init() {
	uint8_t part_num_read; // The part number that we read
	CC2500_ReadReg(CC2500_REG_PARTNUM, &part_num_read);
	if (part_num_read != CC2500_RESET_PARTNUM) {
		// We have the wrong part ID. Maximum error.
		return 255;
	}

	// Send a reset command.
	CC2500_SendCmdStrobe(CC2500_CMDSTROBE_SRES);

	// For now, we don't want to use our TX for anything.. it's sufficient to get the maximum amount of bytes in our RX FIFO
	CC2500_WriteReg(CC2500_REG_FIFOTHR, CC2500_FIFOTHR_OPT);

	CC2500_WriteReg(CC2500_REG_PKTLEN, sizeof(CC2500_Packet) + 1); // Packet LENGTH: CC2500_Packet, but an extra one for the address byte.
	CC2500_WriteReg(CC2500_REG_PKTCTRL1, 0b00001101); // Packet CTRL: CRC autoflush, append status bytes, strict address check
	CC2500_WriteReg(CC2500_REG_PKTCTRL0, 0b01000100);	// Packet CTRL: data whitening, CRC enabled, fixed packet length mode
	CC2500_WriteReg(CC2500_REG_MDMCFG2,	0x73);	// Modem configuration: MSK, 30/32 sync bits detected
	CC2500_WriteReg(CC2500_REG_MDMCFG1,	0xC2);	// Modem configuration: FEC, 8 byte minimum preamble
	CC2500_WriteReg(CC2500_REG_ADDR, 32); // Channel address: 32 because I like that number.
	CC2500_WriteReg(CC2500_REG_CHANNR, 127); // Channel number: 127 because I also like that number.
	CC2500_WriteReg(CC2500_REG_PATABLE,	0xFF);	// Output power: +1dBm (the maximum possible)
	return 0;
}
