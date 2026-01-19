/*
 * eeprom.c
 *
 *  Created on: Jan 11, 2026
 *      Author: pongn
 */

#include "eeprom.h"
#include <string.h>

static I2C_HandleTypeDef *db_i2c;

/* -------------------------------------------------------------------------- */
/* PRIVATE HELPER FUNCTIONS                          */
/* -------------------------------------------------------------------------- */

/*
 * [SIGNAL EXPLANATION] AT24C16B Addressing Logic
 * The AT24C16B is 2048 bytes, but I2C usually only sends an 8-bit address (0-255).
 * To access memory above 255, this specific chip uses the "Device Address" byte.
 *
 * Standard I2C Addr: [1 0 1 0  A2 A1 A0  R/W]
 * AT24C16B Logic:    [1 0 1 0  P2 P1 P0  R/W]
 *
 * - P2, P1, P0 become the "Page" or "Block" bits (High part of memory address).
 * - This function calculates which block (0-7) our target address belongs to.
 * - It shifts those bits left by 1 to align with bits 1, 2, and 3 of the I2C byte.
 */
static uint8_t get_dev_addr(uint16_t mem_addr) {
	// Extract the top 3 bits of the memory address (bits 8, 9, 10)
	uint8_t block_bits = (mem_addr >> 8) & 0x07;

	// Combine base address (0xA0) with the block bits shifted into position
	return EEPROM_BASE_ADDR | (block_bits << 1);
}

/*
 * Purpose: Writes 4 bytes (one UID) to a calculated memory location.
 *
 * [SIGNAL EXPLANATION] Write Cycle
 * 1. HAL_I2C_Mem_Write sends: [DevAddr] [ACK] [MemAddr] [ACK] [Data1] [ACK]...
 * 2. Crucial Step: HAL_Delay(5).
 * EEPROMs are slow. After the I2C signal stops, the chip physically burns
 * data into silicon. During this ~5ms, it will ignore all I2C signals (NACK).
 * Without this delay, the next command would fail immediately.
 */
static void write_uid_at(uint16_t index, uint8_t *data) {
	// Calculate physical address: Start at byte 4, each entry is 4 bytes
	uint16_t mem_addr = 4 + (index * 4);
	uint8_t dev_addr = get_dev_addr(mem_addr);

	HAL_I2C_Mem_Write(db_i2c, dev_addr, mem_addr & 0xFF, I2C_MEMADD_SIZE_8BIT,
			data, 4, 100);

	// Wait for internal write cycle to finish
	HAL_Delay(5);
}

/*
 * Purpose: Reads 4 bytes from a specific index into a buffer.
 *
 * [SIGNAL EXPLANATION] Random Read
 * 1. Writes a "Dummy Write" signal first to set the internal address pointer.
 * 2. Sends a Repeated Start condition.
 * 3. Sends Device Address with Read Bit (1).
 * 4. Reads 4 bytes, sending ACK for the first 3 and NACK for the last one.
 */
static void read_uid_at(uint16_t index, uint8_t *buffer) {
	uint16_t mem_addr = 4 + (index * 4);
	uint8_t dev_addr = get_dev_addr(mem_addr);

	HAL_I2C_Mem_Read(db_i2c, dev_addr, mem_addr & 0xFF, I2C_MEMADD_SIZE_8BIT,
			buffer, 4, 100);
}

/* -------------------------------------------------------------------------- */
/* PUBLIC FUNCTIONS                              */
/* -------------------------------------------------------------------------- */

/*
 * Purpose: Checks if the EEPROM is connected and responding.
 * Returns: true if the device acknowledges (ACK) the address.
 */
bool EEPROM_Init(I2C_HandleTypeDef *hi2c) {
	db_i2c = hi2c;
	// Pings address 0xA0. If chip pulls SDA low (ACK), it is ready.
	if (HAL_I2C_IsDeviceReady(db_i2c, EEPROM_BASE_ADDR, 2, 100) == HAL_OK) {
		return true;
	}
	return false;
}

/*
 * Purpose: Resets the "User Count" at address 0x00 to zero.
 * Note: Does not physically erase the whole chip, just resets the counter
 * so the system thinks it is empty.
 */
void EEPROM_Wipe(void) {
	uint8_t zero = 0;
	HAL_I2C_Mem_Write(db_i2c, EEPROM_BASE_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT,
			&zero, 1, 100);
	HAL_Delay(5); // Wait for write cycle
}

/*
 * Purpose: Reads the very first byte (Addr 0x00) to see how many users are stored.
 * Returns: Number of users (0-250). If 0xFF (fresh chip), returns 0.
 */
uint8_t EEPROM_GetCount(void) {
	uint8_t count = 0;
	HAL_I2C_Mem_Read(db_i2c, EEPROM_BASE_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT,
			&count, 1, 100);
	if (count == 0xFF)
		return 0;
	return count;
}

/*
 * Purpose: Scans the entire database to see if a specific UID exists.
 * Returns: true if found, false if not.
 */
bool EEPROM_CheckUID(uint8_t *uid_bytes) {
	uint8_t count = EEPROM_GetCount();
	uint8_t temp[4];

	// Linear Search: Read every saved UID one by one
	for (uint8_t i = 0; i < count; i++) {
		read_uid_at(i, temp);
		if (memcmp(temp, uid_bytes, 4) == 0) {
			return true;
		}
	}
	return false;
}

/*
 * Purpose: Appends a new UID to the end of the list.
 * Logic:
 * 1. Checks for duplicates first (returns 2).
 * 2. Checks if full (returns 1).
 * 3. Writes data to the next available slot.
 * 4. Updates the total count at address 0x00.
 */
uint8_t EEPROM_SaveUID(uint8_t *uid_bytes) {
	if (EEPROM_CheckUID(uid_bytes)) {
		return 2; // Already Exists
	}

	uint8_t count = EEPROM_GetCount();
	if (count >= 250)
		return 1; // Database Full

	// Write the 4-byte UID
	write_uid_at(count, uid_bytes);

	// Update the counter
	count++;
	HAL_I2C_Mem_Write(db_i2c, EEPROM_BASE_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT,
			&count, 1, 100);
	HAL_Delay(5); // Critical write delay

	return 0; // Success
}

/*
 * Purpose: Deletes a specific UID from the database.
 * Algorithm: "Swap and Pop" (Fast Delete)
 * 1. Find the index of the UID we want to delete.
 * 2. Read the UID of the very LAST user in the database.
 * 3. Overwrite the "to-be-deleted" UID with that LAST UID.
 * 4. Decrease the total count.
 * This avoids shifting all data, saving thousands of write cycles.
 */
bool EEPROM_DeleteUID(uint8_t *uid_bytes) {
	uint8_t count = EEPROM_GetCount();
	uint8_t temp[4];
	uint8_t target_index = 0xFF; // Sentinal value for "Not Found"

	// 1. Find target index
	for (uint8_t i = 0; i < count; i++) {
		read_uid_at(i, temp);
		if (memcmp(temp, uid_bytes, 4) == 0) {
			target_index = i;
			break;
		}
	}

	if (target_index == 0xFF)
		return false; // Not found

	// 2. Perform Swap
	uint8_t last_index = count - 1;

	// Only swap if we aren't already deleting the last one
	if (target_index != last_index) {
		read_uid_at(last_index, temp);   // Read Last
		write_uid_at(target_index, temp); // Overwrite Target
	}

	// 3. Update Count
	count--;
	HAL_I2C_Mem_Write(db_i2c, EEPROM_BASE_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT,
			&count, 1, 100);
	HAL_Delay(5);

	return true;
}
