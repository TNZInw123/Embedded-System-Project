/*
 * lcd1602_i2c.h
 *
 *  Created on: Jan 10, 2026
 *      Author: pongn
 */

#ifndef INC_LCD1602_I2C_H_
#define INC_LCD1602_I2C_H_

#include "main.h"
typedef struct {
	I2C_HandleTypeDef *hi2c;
	uint8_t addr;
	uint8_t cols;
	uint8_t rows;
	uint8_t backlight;
} LCD1602_I2C;

void LCD_Init(LCD1602_I2C *lcd, I2C_HandleTypeDef *hi2c, uint8_t addr,
		uint8_t cols, uint8_t rows);
void LCD_SendCommand(LCD1602_I2C *lcd, uint8_t cmd);
void LCD_Print(LCD1602_I2C *lcd, const char *str);
void LCD_Clear(LCD1602_I2C *lcd);
void LCD_SetCursor(LCD1602_I2C *lcd, uint8_t col, uint8_t row);
void LCD_SendData(LCD1602_I2C *lcd, uint8_t data);
void LCD_CreateCustomChar(LCD1602_I2C *lcd, uint8_t location, uint8_t *data);

#endif /* INC_LCD1602_I2C_H_ */
