/*
 * rc522.c
 *
 *  Created on: Jan 9, 2026
 *      Author: pongn
 */

#include "rc522.h"

/* Registers (Internal Memory Addresses of the RC522 Chip) */
#define CommandReg      0x01
#define ComIrqReg       0x04  // Interrupt Request (Flags for "Done", "Error", "Timer")
#define DivIrqReg       0x05
#define ErrorReg        0x06
#define FIFODataReg     0x09  // The buffer where we put data to send / read data received
#define FIFOLevelReg    0x0A  // How many bytes are currently in the FIFO
#define BitFramingReg   0x0D  // Configures how bits are framed (Start/Stop bits)
#define CollReg         0x0E  // Collision detection register
#define ModeReg         0x11
#define TxModeReg       0x12  // Transmit modulation settings
#define RxModeReg       0x13  // Receive modulation settings
#define TxControlReg    0x14  // Controls the Antenna Driver pins (TX1, TX2)
#define TxASKReg        0x15
#define RFCfgReg        0x26
#define TModeReg        0x2A  // Internal Timer settings
#define TPrescalerReg   0x2B
#define TReloadRegH     0x2C
#define TReloadRegL     0x2D
#define ModWidthReg     0x24

/* RFID Commands (Instructions sent to the Card) */
#define PCD_IDLE        0x00
#define PCD_TRANSCEIVE  0x0C // "Send data in FIFO to card, then wait for response"
#define PCD_SOFTRESET   0x0F
#define PICC_REQA       0x26 // "Request Type A" (Wake up call)
#define PICC_WUPA       0x52 // "Wake Up Protocol A"
#define PICC_SEL_CL1    0x93 // "Select Cascade Level 1" (Ask for UID)
#define PICC_HLTA       0x50

/* Timeouts (ms) */
#define TIMEOUT_SOFT_RESET  50
#define TIMEOUT_TRANSCEIVE  36
#define TIMEOUT_HW_RESET_LOW 10
#define TIMEOUT_HW_RESET_HIGH 50

/* -------------------------------------------------------------------------- */
/* GPIO & SPI LOW-LEVEL FUNCTIONS                                            */
/* -------------------------------------------------------------------------- */

/*
 * Purpose: Selects the RC522 chip so it listens to the SPI bus.
 * [SIGNAL] SPI Chip Select (CS) is Active Low.
 * When this pin goes 0V, the RC522 resets its SPI logic and waits for the first bit.
 */
static inline void CS_LOW(RC522_Handle *d) {
	HAL_GPIO_WritePin(d->cs_port, d->cs_pin, GPIO_PIN_RESET);
}

/*
 * Purpose: Deselects the RC522.
 * [SIGNAL] When High (3.3V), the RC522 ignores all signals on MOSI/SCK.
 */
static inline void CS_HIGH(RC522_Handle *d) {
	HAL_GPIO_WritePin(d->cs_port, d->cs_pin, GPIO_PIN_SET);
}

/*
 * Purpose: Writes a value to a specific register inside the RC522.
 *
 * [SIGNAL EXPLANATION] SPI Address Format
 * The RC522 has a specific address byte format:
 * MSB (Bit 7): 0 = Write, 1 = Read
 * Bits 6-1:    The Register Address
 * LSB (Bit 0): Always 0
 *
 * Logic: 'reg << 1' shifts the address into bits 6-1.
 * Example: To write to Register 0x01:
 * Binary 0000 0001 -> Shift Left -> 0000 0010 (Sends byte 0x02)
 */
void RC522_WriteReg(RC522_Handle *d, uint8_t reg, uint8_t val) {
	uint8_t tx[2] = { reg << 1, val }; // [Address Byte, Data Byte]
	CS_LOW(d);
	HAL_SPI_Transmit(d->hspi, tx, 2, 10);
	CS_HIGH(d);
}

/*
 * Purpose: Reads a value from a register.
 *
 * [SIGNAL EXPLANATION] SPI Read
 * We send the Address Byte with Bit 7 set to 1 (0x80).
 * While we send the second byte (Dummy 0x00), the RC522 sends back the data on the MISO line.
 */
uint8_t RC522_ReadReg(RC522_Handle *d, uint8_t reg) {
	uint8_t tx[2] = { (reg << 1) | 0x80, 0 }; // Set Read Bit (0x80)
	uint8_t rx[2];
	CS_LOW(d);
	HAL_SPI_TransmitReceive(d->hspi, tx, rx, 2, 10); // Full Duplex
	CS_HIGH(d);
	return rx[1]; // The data comes in during the second byte
}

/* Helper: Sets specific bits in a register without changing others */
void RC522_SetBitMask(RC522_Handle *d, uint8_t reg, uint8_t mask) {
	uint8_t tmp = RC522_ReadReg(d, reg);
	RC522_WriteReg(d, reg, tmp | mask);
}

/* Helper: Clears specific bits */
void RC522_ClearBitMask(RC522_Handle *d, uint8_t reg, uint8_t mask) {
	uint8_t tmp = RC522_ReadReg(d, reg);
	RC522_WriteReg(d, reg, tmp & (~mask));
}

/*
 * Purpose: Turns on the 13.56 MHz Radio Field.
 * [SIGNAL] Without this, the antenna pins (TX1, TX2) are effectively disconnected.
 * The coil will generate no magnetic field, so no power is sent to the card.
 */
static void RC522_AntennaOn(RC522_Handle *d) {
	uint8_t temp = RC522_ReadReg(d, TxControlReg);
	if ((temp & 0x03) != 0x03) {
		RC522_SetBitMask(d, TxControlReg, 0x03); // Bit 0 & 1 enable TX1/TX2
	}
}

/* -------------------------------------------------------------------------- */
/* STATE MACHINE FUNCTIONS                                                   */
/* -------------------------------------------------------------------------- */

void RC522_InitHandle(RC522_Handle *d, SPI_HandleTypeDef *hspi,
		GPIO_TypeDef *cs_port, uint16_t cs_pin, GPIO_TypeDef *rst_port,
		uint16_t rst_pin) {
	d->hspi = hspi;
	d->cs_port = cs_port;
	d->cs_pin = cs_pin;
	d->rst_port = rst_port;
	d->rst_pin = rst_pin;
	d->state = RC522_STATE_IDLE;
	d->uid.size = 0;
	d->retries = 0;
}

void RC522_Reset(RC522_Handle *d) {
	d->state = RC522_STATE_IDLE;
	d->retries = 0;
}

/*
 * Purpose: Handles the startup sequence (Reset -> Config Registers).
 * Logic:
 * 1. Physical Reset: Pull RST pin Low, then High.
 * 2. Soft Reset: Send command 0x0F to internal command register.
 * 3. Config: Set timer frequencies and gain for 13.56MHz operation.
 */
RC522_Status RC522_Init(RC522_Handle *d) {
	switch (d->state) {
	case RC522_STATE_IDLE:
		CS_HIGH(d);
		if (d->rst_port) {
			HAL_GPIO_WritePin(d->rst_port, d->rst_pin, GPIO_PIN_RESET);
			d->timestamp = HAL_GetTick();
			d->state = RC522_STATE_INIT_HW_RESET;
		} else {
			d->state = RC522_STATE_INIT_SOFT_RESET;
		}
		return RC522_PENDING;

	case RC522_STATE_INIT_HW_RESET:
		// Hold RST Low for at least 10ms to discharge capacitors
		if ((HAL_GetTick() - d->timestamp) >= TIMEOUT_HW_RESET_LOW) {
			HAL_GPIO_WritePin(d->rst_port, d->rst_pin, GPIO_PIN_SET);
			d->timestamp = HAL_GetTick();
			d->state = RC522_STATE_INIT_WAIT_HW;
		}
		return RC522_PENDING;

	case RC522_STATE_INIT_WAIT_HW:
		// Wait for chip to boot up after RST goes High
		if ((HAL_GetTick() - d->timestamp) >= TIMEOUT_HW_RESET_HIGH) {
			d->state = RC522_STATE_INIT_SOFT_RESET;
		}
		return RC522_PENDING;

	case RC522_STATE_INIT_SOFT_RESET:
		RC522_WriteReg(d, CommandReg, PCD_SOFTRESET); // Internal digital reset
		d->timestamp = HAL_GetTick();
		d->state = RC522_STATE_INIT_WAIT_SOFT;
		return RC522_PENDING;

	case RC522_STATE_INIT_WAIT_SOFT:
		// Check if the 'PowerDown' bit has cleared, indicating reset is done
		if (!(RC522_ReadReg(d, CommandReg) & (1 << 4))) {
			d->state = RC522_STATE_INIT_CONFIG;
		} else if ((HAL_GetTick() - d->timestamp) >= TIMEOUT_SOFT_RESET) {
			d->state = RC522_STATE_INIT_CONFIG; // Timeout safety
		}
		return RC522_PENDING;

	case RC522_STATE_INIT_CONFIG:
		// Magic numbers to setup the 13.56MHz Timer and Modulation
		RC522_WriteReg(d, TxModeReg, 0x00);
		RC522_WriteReg(d, RxModeReg, 0x00);
		RC522_WriteReg(d, ModWidthReg, 0x26); // Set Modulation Width
		RC522_WriteReg(d, TModeReg, 0x80);    // Timer: Auto start
		RC522_WriteReg(d, TPrescalerReg, 0xA9); // Timer Freq
		RC522_WriteReg(d, TReloadRegH, 0x03);
		RC522_WriteReg(d, TReloadRegL, 0xE8);
		RC522_WriteReg(d, TxASKReg, 0x40);    // Force 100% ASK modulation
		RC522_WriteReg(d, ModeReg, 0x3D);     // CRC Preset value 0x6363

		RC522_AntennaOn(d); // Turn on the Radio Field
		d->state = RC522_STATE_READY;
		return RC522_OK;

	default:
		return RC522_ERROR;
	}
}

/*
 * Purpose: Polls for a card in the RF field.
 * Logic: Sends "REQA" (Request A). If a card is nearby, it disrupts the field
 * and sends back an ACK.
 */
RC522_Status RC522_IsNewCardPresent(RC522_Handle *d) {
	uint8_t n, fifoLen;

	switch (d->state) {
	case RC522_STATE_IDLE:
	case RC522_STATE_READY:
		// Clear interrupts and FIFO
		RC522_WriteReg(d, TxModeReg, 0x00);
		RC522_WriteReg(d, RxModeReg, 0x00);
		RC522_WriteReg(d, ModWidthReg, 0x26);
		RC522_ClearBitMask(d, CollReg, 0x80); // Clear collision bits

		// Prepare command: REQA (0x26)
		RC522_WriteReg(d, CommandReg, PCD_IDLE); // Stop current command
		RC522_WriteReg(d, ComIrqReg, 0x7F);      // Clear all interrupts
		RC522_WriteReg(d, FIFOLevelReg, 0x80);   // Flush internal FIFO buffer
		RC522_WriteReg(d, FIFODataReg, PICC_REQA); // Load REQA command into FIFO
		RC522_WriteReg(d, BitFramingReg, 0x07);  // Prepare for transmission

		// Execute: Transceive (Transmit + Receive)
		RC522_WriteReg(d, CommandReg, PCD_TRANSCEIVE);
		RC522_SetBitMask(d, BitFramingReg, 0x80); // Start sending

		d->timestamp = HAL_GetTick();
		d->state = RC522_STATE_DETECT_WAIT;
		return RC522_PENDING;

	case RC522_STATE_DETECT_WAIT:
		n = RC522_ReadReg(d, ComIrqReg);
		// Wait for "Done" (0x20) or "Timer" (0x10) interrupts
		if (n & 0x30) {
			d->state = RC522_STATE_DETECT_CHECK;
		} else if ((HAL_GetTick() - d->timestamp) >= TIMEOUT_TRANSCEIVE) {
			d->state = RC522_STATE_READY;
			return RC522_TIMEOUT;
		} else {
			return RC522_PENDING;
		}
		return RC522_PENDING;

	case RC522_STATE_DETECT_CHECK:
		// Check if we received valid data back from a card
		fifoLen = RC522_ReadReg(d, FIFOLevelReg);
		d->state = RC522_STATE_READY;
		if (fifoLen == 2) {
			return RC522_OK;  // ATQA received (2 bytes) = Card Found!
		}
		return RC522_NO_CARD;

	default:
		d->state = RC522_STATE_READY;
		return RC522_ERROR;
	}
}

/*
 * Purpose: Asks the detected card for its unique ID (UID).
 * Logic: Uses the "Anti-Collision" protocol.
 * The RC522 asks "Who are you?" (PICC_SEL_CL1).
 * The Card responds with 4 bytes of ID + 1 byte Checksum (BCC).
 */
RC522_Status RC522_ReadCardUID(RC522_Handle *d) {
	uint8_t fifoLen, bcc;

	switch (d->state) {
	case RC522_STATE_READY:
		// Prepare Anti-Collision Command
		RC522_WriteReg(d, CommandReg, PCD_IDLE);
		RC522_WriteReg(d, ComIrqReg, 0x7F);
		RC522_WriteReg(d, FIFOLevelReg, 0x80);
		RC522_WriteReg(d, FIFODataReg, PICC_SEL_CL1); // "Select Level 1"
		RC522_WriteReg(d, FIFODataReg, 0x20); // "20" means "Tell me your whole ID"
		RC522_WriteReg(d, BitFramingReg, 0x00);
		RC522_WriteReg(d, CommandReg, PCD_TRANSCEIVE);
		RC522_SetBitMask(d, BitFramingReg, 0x80);

		d->timestamp = HAL_GetTick();
		d->state = RC522_STATE_READ_WAIT;
		return RC522_PENDING;

	case RC522_STATE_READ_WAIT:
		if (RC522_ReadReg(d, ComIrqReg) & 0x30) {
			d->state = RC522_STATE_READ_CHECK;
		} else if ((HAL_GetTick() - d->timestamp) >= TIMEOUT_TRANSCEIVE) {
			d->state = RC522_STATE_READY;
			return RC522_TIMEOUT;
		}
		return RC522_PENDING;

	case RC522_STATE_READ_CHECK:
		fifoLen = RC522_ReadReg(d, FIFOLevelReg);
		// We expect 5 bytes: 4 bytes of UID + 1 byte BCC (Checksum)
		if (fifoLen < 5) {
			d->state = RC522_STATE_READY;
			return RC522_ERROR;
		}

		// Read Data from Internal FIFO
		for (uint8_t i = 0; i < 5; i++) {
			d->buffer[i] = RC522_ReadReg(d, FIFODataReg);
		}

		// Verify Checksum (BCC)
		// Logic: UID0 ^ UID1 ^ UID2 ^ UID3 must equal BCC
		bcc = d->buffer[0] ^ d->buffer[1] ^ d->buffer[2] ^ d->buffer[3];
		if (bcc != d->buffer[4]) {
			d->state = RC522_STATE_READY;
			return RC522_CRC_ERROR; // Transmission corrupted
		}

		// Save the Clean UID
		for (uint8_t i = 0; i < 4; i++) {
			d->uid.uidByte[i] = d->buffer[i];
		}
		d->uid.size = 4;
		d->state = RC522_STATE_READY;
		return RC522_OK;

	default:
		d->state = RC522_STATE_READY;
		return RC522_ERROR;
	}
}
