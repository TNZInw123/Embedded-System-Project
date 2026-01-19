/*
 * eeprom.h
 *
 *  Created on: Jan 11, 2026
 *      Author: pongn
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#include "main.h"
#include <stdbool.h>
#include <string.h>

#define EEPROM_BASE_ADDR 0xA0

typedef struct {
	uint8_t bytes[4];
} DB_UID;

bool EEPROM_Init(I2C_HandleTypeDef *hi2c);
void EEPROM_Wipe(void);
uint8_t EEPROM_SaveUID(uint8_t *uid_bytes);
bool EEPROM_CheckUID(uint8_t *uid_bytes);
uint8_t EEPROM_GetCount(void);
bool EEPROM_DeleteUID(uint8_t *uid_bytes);

#endif /* INC_EEPROM_H_ */
