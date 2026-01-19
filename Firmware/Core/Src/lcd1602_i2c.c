/*
 * lcd1602_i2c.c
 *
 *  Created on: Jan 10, 2026
 *      Author: pongn
 */

#include "lcd1602_i2c.h"

// --- Pin Mapping on the I2C Expander (PCF8574) ---
// The PCF8574 chip takes the serial I2C byte and outputs it to 8 parallel pins (P0-P7).
// These definitions map which bit controls which pin on the LCD.
#define En 0x04  // Enable bit (P2)
#define Rw 0x02  // Read/Write bit (P1)
#define Rs 0x01  // Register select bit (P0)
#define LCD_BACKLIGHT 0x08 // Backlight Control (P3)

/* -------------------------------------------------------------------------- */
/* PRIVATE HELPER FUNCTIONS (The "Physical Layer")                            */
/* -------------------------------------------------------------------------- */

/*
 * Purpose: Sends a raw byte over the I2C bus to the IO Expander.
 *
 * [SIGNAL EXPLANATION] I2C Transport
 * - This function creates the actual electrical signals on SDA/SCL lines.
 * - We always OR (|) the data with 'lcd->backlight'.
 * If we didn't do this, sending data would accidentally turn off the backlight pin.
 * - The PCF8574 acts as a "Shift Register" here: what we send becomes the voltage
 * state of its output pins immediately.
 */
static void i2c_write_byte(LCD1602_I2C *lcd, uint8_t val) {
	uint8_t d = val | lcd->backlight;
	HAL_I2C_Master_Transmit(lcd->hi2c, lcd->addr << 1, &d, 1, 100);
}

/*
 * Purpose: Toggles the 'Enable' (En) pin to tell the LCD to read the data.
 *
 * [SIGNAL EXPLANATION] The "Latch" Pulse
 * The LCD is parallel hardware. It ignores the data pins (D4-D7) until
 * it sees a falling edge (High -> Low) on the En pin.
 * 1. Set En HIGH: Prepare the LCD to listen.
 * 2. Delay: Wait for voltage to stabilize.
 * 3. Set En LOW: This "Click" forces the LCD to read the D4-D7 pins NOW.
 */
static void lcd_pulse_enable(LCD1602_I2C *lcd, uint8_t val) {
	i2c_write_byte(lcd, val | En);   // Pin High
	HAL_Delay(1);
	i2c_write_byte(lcd, val & ~En);  // Pin Low (Latch Data)
	HAL_Delay(1);
}

/*
 * Purpose: Sends 4 bits of data (a nibble) to the LCD.
 *
 * [SIGNAL EXPLANATION] 4-Bit Mode
 * Since the I2C expander only has 8 pins total (and we use 4 for control: RS, RW, EN, Backlight),
 * we only have 4 pins left for data. We cannot send a full 8-bit byte at once.
 * - This function places the data bits in the upper 4 bits of the byte (P4-P7).
 * - It keeps the control bits (RS/RW) in the lower part.
 */
static void lcd_write_nibble(LCD1602_I2C *lcd, uint8_t nibble, uint8_t mode) {
	uint8_t val = (nibble & 0xF0) | mode; // Combine Data + Control (RS/RW)
	i2c_write_byte(lcd, val);             // Set Pins
	lcd_pulse_enable(lcd, val);           // Latch it
}

/*
 * Purpose: Splits an 8-bit byte into two 4-bit nibbles and sends them.
 *
 * [SIGNAL EXPLANATION] High Order First
 * The HD44780 LCD controller expects the "High Nibble" (bits 4-7) first,
 * followed immediately by the "Low Nibble" (bits 0-3).
 * If this order is swapped, the LCD will interpret garbage characters.
 */
static void lcd_send(LCD1602_I2C *lcd, uint8_t value, uint8_t mode) {
	lcd_write_nibble(lcd, value & 0xF0, mode);       // Send Top Half
	lcd_write_nibble(lcd, (value << 4) & 0xF0, mode); // Send Bottom Half
}

/* -------------------------------------------------------------------------- */
/* PUBLIC API (The "Application Layer")                                       */
/* -------------------------------------------------------------------------- */

/*
 * Purpose: Initializes the LCD hardware and forces it into 4-bit mode.
 *
 * [SIGNAL EXPLANATION] The "Magic Sequence"
 * When the LCD powers up, it starts in 8-bit mode. Because we only connected 4 data wires,
 * the LCD is confused. We send a specific sequence of "0x30" commands blindly.
 * This forces the internal state machine of the LCD to reset and accept 4-bit mode.
 * Without this specific timing (5ms, 1ms delays), the initialization will fail.
 */
void LCD_Init(LCD1602_I2C *lcd, I2C_HandleTypeDef *hi2c, uint8_t addr,
		uint8_t cols, uint8_t rows) {
	lcd->hi2c = hi2c;
	lcd->addr = addr;
	lcd->cols = cols;
	lcd->rows = rows;
	lcd->backlight = LCD_BACKLIGHT;

	// 1. Wait for voltage to rise after power-on
	HAL_Delay(50);

	// 2. Reset Sequence (Syncing with the internal LCD clock)
	lcd_write_nibble(lcd, 0x30, 0);
	HAL_Delay(5);
	lcd_write_nibble(lcd, 0x30, 0);
	HAL_Delay(1);
	lcd_write_nibble(lcd, 0x30, 0);
	HAL_Delay(1);
	lcd_write_nibble(lcd, 0x20, 0); // Command to switch to 4-bit mode
	HAL_Delay(1);

	// 3. Configure Settings
	LCD_SendCommand(lcd, 0x28); // Function: 4-bit bus, 2 display lines, 5x8 font
	LCD_SendCommand(lcd, 0x08); // Display: OFF (during setup)
	LCD_Clear(lcd);             // Clear RAM
	LCD_SendCommand(lcd, 0x06); // Entry Mode: Auto-increment cursor right
	LCD_SendCommand(lcd, 0x0C); // Display: ON, Cursor: OFF, Blink: OFF
}

/*
 * Purpose: Sends a configuration command (like Clear Screen, Move Cursor).
 * Mode: RS = 0 (Command Register)
 */
void LCD_SendCommand(LCD1602_I2C *lcd, uint8_t cmd) {
	lcd_send(lcd, cmd, 0);
	HAL_Delay(2); // Commands take longer to process than data
}

/*
 * Purpose: Sends ASCII data to be drawn on the screen.
 * Mode: RS = 1 (Data Register)
 */
void LCD_SendData(LCD1602_I2C *lcd, uint8_t data) {
	lcd_send(lcd, data, Rs);
}

/*
 * Purpose: Prints a full string of text.
 * Logic: Loops through the C-string pointer until it hits the Null Terminator (\0).
 */
void LCD_Print(LCD1602_I2C *lcd, const char *str) {
	while (*str) {
		lcd_send(lcd, (uint8_t) (*str), Rs);
		str++;
	}
}

/*
 * Purpose: Wipes the screen and resets cursor to 0,0.
 * Note: This command is slow (requires ~2ms delay).
 */
void LCD_Clear(LCD1602_I2C *lcd) {
	LCD_SendCommand(lcd, 0x01);
	HAL_Delay(2);
}

/*
 * Purpose: Moves the write position to a specific X,Y coordinate.
 * Signal: The LCD memory is linear (0x00-0x27 is Line 1, 0x40-0x67 is Line 2).
 * This function calculates the correct memory address based on Row/Col.
 */
void LCD_SetCursor(LCD1602_I2C *lcd, uint8_t col, uint8_t row) {
	int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
	if (row >= lcd->rows)
		row = lcd->rows - 1;
	LCD_SendCommand(lcd, 0x80 | (col + row_offsets[row]));
}

/*
 * Purpose: Uploads a custom pixel pattern (icon) to the LCD's CGRAM.
 *
 * [SIGNAL EXPLANATION] CGRAM Addressing
 * The LCD has a small RAM area called CGRAM (Character Generator RAM).
 * - Command 0x40 tells the LCD "I am about to write pixel data".
 * - We shift 'location' by 3 (multiply by 8) because each character takes 8 bytes.
 * - We then send 8 bytes of data, which represent the 8 pixel rows of the icon.
 */
void LCD_CreateCustomChar(LCD1602_I2C *lcd, uint8_t location, uint8_t *data) {
	location &= 0x07; // Limit to 8 locations (0-7)

	// 1. Send Command to set CGRAM Address pointer
	LCD_SendCommand(lcd, 0x40 | (location << 3));

	// 2. Write the 8 bytes of pixel data
	for (int i = 0; i < 8; i++) {
		LCD_SendData(lcd, data[i]);
	}
}
