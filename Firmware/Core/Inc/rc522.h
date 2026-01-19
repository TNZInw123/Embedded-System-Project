/*
 * rc522.h
 *
 *  Created on: Jan 9, 2026
 *      Author: pongn
 */

#ifndef INC_RC522_H_
#define INC_RC522_H_

#include "main.h"
#include <stdbool.h>

/* UID Structure */
typedef struct {
	uint8_t size;
	uint8_t uidByte[4];
} RC522_Uid;

/* State Machine States */
typedef enum {
	RC522_STATE_IDLE = 0,
	RC522_STATE_INIT_HW_RESET,
	RC522_STATE_INIT_WAIT_HW,
	RC522_STATE_INIT_SOFT_RESET,
	RC522_STATE_INIT_WAIT_SOFT,
	RC522_STATE_INIT_CONFIG,
	RC522_STATE_READY,
	RC522_STATE_DETECT_SEND,
	RC522_STATE_DETECT_WAIT,
	RC522_STATE_DETECT_CHECK,
	RC522_STATE_READ_SEND,
	RC522_STATE_READ_WAIT,
	RC522_STATE_READ_CHECK,
	RC522_STATE_ERROR
} RC522_State;

typedef enum {
	RC522_OK = 0,
	RC522_PENDING,
	RC522_ERROR,
	RC522_TIMEOUT,
	RC522_NO_CARD,
	RC522_CRC_ERROR
} RC522_Status;

typedef struct {
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *cs_port;
	GPIO_TypeDef *rst_port;

	uint32_t timestamp;
	RC522_State state;

	uint16_t cs_pin;
	uint16_t rst_pin;
	RC522_Uid uid;
	uint8_t buffer[10];
	uint8_t retries;

} RC522_Handle;

void RC522_InitHandle(RC522_Handle *d, SPI_HandleTypeDef *hspi,
		GPIO_TypeDef *cs_port, uint16_t cs_pin, GPIO_TypeDef *rst_port,
		uint16_t rst_pin);
RC522_Status RC522_Init(RC522_Handle *d);
RC522_Status RC522_IsNewCardPresent(RC522_Handle *d);
RC522_Status RC522_ReadCardUID(RC522_Handle *d);
void RC522_Reset(RC522_Handle *d);
void RC522_WriteReg(RC522_Handle *d, uint8_t reg, uint8_t val);
uint8_t RC522_ReadReg(RC522_Handle *d, uint8_t reg);
void RC522_SetBitMask(RC522_Handle *d, uint8_t reg, uint8_t mask);
void RC522_ClearBitMask(RC522_Handle *d, uint8_t reg, uint8_t mask);

#endif /* INC_RC522_H_ */
