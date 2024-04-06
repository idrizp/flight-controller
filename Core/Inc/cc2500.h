/*
 * cc2500.h
 *
 *  Created on: Mar 29, 2024
 *      Author: idrizpelaj
 */

#ifndef INC_CC2500_H_
#define INC_CC2500_H_

#include <stm32l0xx.h>
#include <sys/_stdint.h>


// Command Strobes as found on page 57.
#define CC2500_CMDSTROBE_SRES 0x30 // Reset chip
#define CC2500_CMDSTROBE_SFSTXON 0x31 // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1). If in RX (with CCA): Go to a wait state where only the synthesizer is running (for quick RX / TX turnaround)
#define CC2500_CMDSTROBE_SXOFF 0x32 // Turn off crystal oscillator
#define CC2500_CMDSTROBE_SCAL 0x33 // Calibrate frequency synthesizer and turn it off. SCAL can be strobed from IDLE mode without setting manual calibration mode (MCSM0.FS_AUTOCAL=0)
#define CC2500_CMDSTROBE_SRX 0x34 // Enable RX. Perform calibration first if coming from IDLE and MCSM0.FS_AUTOCAL=1
#define CC2500_CMDSTROBE_STX 0x35 // Enable TX. Perform calibration first if MCSM0.FS_AUTOCAL=1. If in RX state and CCA is enabled: Only go to TX if channel is clear
#define CC2500_CMDSTROBE_SIDLE 0x36 // Exit RX / TX, turn off frequency synthesizer and exit Wake-On-Radio mode if applicable. Start automatic RX polling sequence (Wake-on-Radio) as described in Section 19.5 if WORCTRL.RC_PD=0
#define CC2500_CMDSTROBE_SWOR 0x38 // Enter power down mode when CSn goes high
#define CC2500_CMDSTROBE_SPWD 0x39 // No operation. May be used to get access to the chip status byte
#define CC2500_CMDSTROBE_SFRX 0x3A // Flush the RX FIFO buffer. Only issue SFRX in IDLE or RXFIFO_OVERFLOW states
#define CC2500_CMDSTROBE_SFTX 0x3B // Flush the TX FIFO buffer. Only issue SFTX in IDLE or TXFIFO_UNDERFLOW states
#define CC2500_CMDSTROBE_SWORRST 0x3C // Reset real time clock to Event1 value
#define CC2500_CMDSTROBE_SNOP 0x3D // No operation. May be used to get access to the chip status byte

// Registers as set from pg. 58.
#define CC2500_REG_IOCFG2 0x00 // GDO2 output pin configuration. Preserved in SLEEP State
#define CC2500_REG_IOCFG1 0x01 // GDO1 output pin configuration. Preserved in SLEEP State
#define CC2500_REG_IOCFG0 0x02 // GDO0 output pin configuration. Preserved in SLEEP State
#define CC2500_REG_FIFOTHR 0x03 // RX FIFO and TX FIFO thresholds. Preserved in SLEEP State
#define CC2500_REG_SYNC1 0x04 // Sync word, high byte. Preserved in SLEEP State
#define CC2500_REG_SYNC0 0x05 // Sync word, low byte. Preserved in SLEEP State
#define CC2500_REG_PKTLEN 0x06 // Packet length. Preserved in SLEEP State
#define CC2500_REG_PKTCTRL1 0x07 // Packet automation control. Preserved in SLEEP State
#define CC2500_REG_PKTCTRL0 0x08 // Packet automation control. Preserved in SLEEP State
#define CC2500_REG_ADDR 0x09 // Device address. Preserved in SLEEP State
#define CC2500_REG_CHANNR 0x0A // Channel number. Preserved in SLEEP State
#define CC2500_REG_FSCTRL1 0x0B // Frequency synthesizer control. Preserved in SLEEP State
#define CC2500_REG_FSCTRL0 0x0C // Frequency synthesizer control. Preserved in SLEEP State
#define CC2500_REG_FREQ2 0x0D // Frequency control word, high byte. Preserved in SLEEP State
#define CC2500_REG_FREQ1 0x0E // Frequency control word, middle byte. Preserved in SLEEP State
#define CC2500_REG_FREQ0 0x0F // Frequency control word, low byte. Preserved in SLEEP State
#define CC2500_REG_MDMCFG4 0x10 // Modem configuration. Preserved in SLEEP State
#define CC2500_REG_MDMCFG3 0x11 // Modem configuration. Preserved in SLEEP State
#define CC2500_REG_MDMCFG2 0x12 // Modem configuration. Preserved in SLEEP State
#define CC2500_REG_MDMCFG1 0x13 // Modem configuration. Preserved in SLEEP State
#define CC2500_REG_MDMCFG0 0x14 // Modem configuration. Preserved in SLEEP State
#define CC2500_REG_DEVIATN 0x15 // Modem deviation setting. Preserved in SLEEP State
#define CC2500_REG_MCSM2 0x16 // Main Radio Control State Machine configuration. Preserved in SLEEP State
#define CC2500_REG_MCSM1 0x17 // Main Radio Control State Machine configuration. Preserved in SLEEP State
#define CC2500_REG_MCSM0 0x18 // Main Radio Control State Machine configuration. Preserved in SLEEP State
#define CC2500_REG_FOCCFG 0x19 // Frequency Offset Compensation configuration. Preserved in SLEEP State
#define CC2500_REG_BSCFG 0x1A // Bit Synchronization configuration. Preserved in SLEEP State
#define CC2500_REG_AGCTRL2 0x1B // AGC control. Preserved in SLEEP State
#define CC2500_REG_AGCTRL1 0x1C // AGC control. Preserved in SLEEP State
#define CC2500_REG_AGCTRL0 0x1D // AGC control. Preserved in SLEEP State
#define CC2500_REG_WOREVT1 0x1E // High byte Event 0 timeout. Preserved in SLEEP State
#define CC2500_REG_WOREVT0 0x1F // Low byte Event 0 timeout. Preserved in SLEEP State
#define CC2500_REG_WORCTRL 0x20 // Wake On Radio control. Preserved in SLEEP State
#define CC2500_REG_FREND1 0x21 // Front end RX configuration. Preserved in SLEEP State
#define CC2500_REG_FREND0 0x22 // Front end TX configuration. Preserved in SLEEP State
#define CC2500_REG_FSCAL3 0x23 // Frequency synthesizer calibration. Preserved in SLEEP State
#define CC2500_REG_FSCAL2 0x24 // Frequency synthesizer calibration. Preserved in SLEEP State
#define CC2500_REG_FSCAL1 0x25 // Frequency synthesizer calibration. Preserved in SLEEP State
#define CC2500_REG_FSCAL0 0x26 // Frequency synthesizer calibration. Preserved in SLEEP State
#define CC2500_REG_RCCTRL1 0x27 // RC oscillator configuration. Preserved in SLEEP State
#define CC2500_REG_RCCTRL0 0x28 // RC oscillator configuration. Preserved in SLEEP State
#define CC2500_REG_FSTEST 0x29 // Frequency synthesizer calibration Production test
#define CC2500_REG_PTEST 0x2A // AGC test
#define CC2500_REG_AGCTEST 0x2B // Various test settings
#define CC2500_REG_TEST2 0x2C // Various test settings
#define CC2500_REG_TEST1 0x2D // Various test settings
#define CC2500_REG_TEST0 0x2E // Various test settings

// READ BUFF
#define CC2500_REG_PARTNUM 0x30 // CC2500 part number
#define CC2500_REG_VERSION 0x31 // Current version number
#define CC2500_REG_FREQEST 0x32 // Frequency offset estimate
#define CC2500_REG_LQI 0x33 // Demodulator estimate for Link Quality
#define CC2500_REG_RSSI 0x34 // Received signal strength indication
#define CC2500_REG_MARCSTATE 0x35 // Control state machine state
#define CC2500_REG_WORTIME1 0x36 // High byte of WOR timer
#define CC2500_REG_WORTIME0 0x37 // Low byte of WOR timer
#define CC2500_REG_PKTSTATUS 0x38 // Current GDOx status and packet status
#define CC2500_REG_VCO_VC_DAC 0x39 // Current setting from PLL calibration module
#define CC2500_REG_TXBYTES 0x3A // Underflow and number of bytes in the TX FIFO
#define CC2500_REG_RXBYTES 0x3B // Overflow and number of bytes in the RX FIFO
#define CC2500_REG_RCCTRL1_STATUS 0x3C // Last RC oscillator calibration result
#define CC2500_REG_RCCTRL0_STATUS 0x3D // Last RC oscillator calibration result
#define CC2500_REG_PATABLE 0x3E // PATABLE

#define CC2500_REG_TX_FIFO 0x3F // The TX FIFO - Single Byte Access
#define CC2500_REG_RX_FIFO 0x3F // The RX FIFO - Single Byte Access

// Registers end.

// RESET values START
#define CC2500_RESET_PARTNUM 0x80
// RESET values END

// The architectural choice for my _own_ project requires that the CC2500 packets are at most 5 bytes.
// This is efficient enough for us, and we can send plenty in our FIFO buffer of 64 bytes.
typedef struct CC2500_Packet {
	uint8_t id; // 1 byte for the packet id.
	uint8_t payload[4]; // 4 bytes for the payload. -- (!) IMPORTANT: THE PAYLOAD MUST BE SENT IN BIG ENDIAN BY THE TRANSMITTER SIDE.
} CC2500_Packet;

// STATUS values START
enum CC2500_Status { // ordered as per pg. 23
	CC2500_STATUS_IDLE, // Idle state
	CC2500_STATUS_RX, // Receive mode
	CC2500_STATUS_TX, // Transmit mode
	CC2500_STATUS_FSTXON, // Frequency synth. is on, ready to start transmitting
	CC2500_STATUS_CALIBRATE, // Frequency synth. calibration is running.
	CC2500_STATUS_SETTLING, // PLL is settling
	CC2500_STATUS_RX_FIFO_OVERFLOW, // RX FIFO has OVERFLOWED, read out useful data and flush with SFRX,
	CC2500_STATUS_TX_FIFO_UNDERFLOW, // TX FIFO has UNDERFLOWED, ACK with SFTX
	CC2500_STATUS_ERROR // ERROR in reading the STATUS from the MICROCONTROLLER
};
// STATUS values END

// Sends a command strobe to the CC2500 and returns the status
enum CC2500_Status CC2500_SendCmdStrobe(uint8_t probe_id);

// Writes to a register of the CC2500.
HAL_StatusTypeDef CC2500_WriteReg(uint8_t addr, uint8_t value);
// Reads from a register of the CC2500.
HAL_StatusTypeDef CC2500_ReadReg(uint8_t addr, uint8_t* value);

// Reads the status of the CC2500.
enum CC2500_Status CC2500_ReadStatus();

// Enter RX mode.
void CC2500_EnterRXMode();

// Flushes the RX FIFO.
void CC2500_FlushRXFIFO();

// Reads the number of bytes in the RX FIFO, and whether it has overflown
void CC2500_ReadRXFIFOBytesInfo(int* num_bytes, int* rx_overflow);

// Reads the inc. packet from the CC2500
void CC2500_ReceivePacket(CC2500_Packet *packet);

// Initializes the CC2500 and returns the number of errors
uint8_t CC2500_Init();

#endif /* INC_CC2500_H_ */
