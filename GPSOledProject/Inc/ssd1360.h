/*
 * ssd1360.h
 *
 *  Created on: 4 Nov 2017
 *      Author: Kjetil
 */

#include "stm32l4xx_hal.h"

#ifndef SSD1360_H_
#define SSD1360_H_

struct SSD1360_d {
	int width;
	int height;
	I2C_HandleTypeDef oledI2c;
	uint8_t addr;
	uint8_t* buffer;
	size_t bufferSize;
};


/*
 * Initalize handler.
 */
struct SSD1360_d Init(int width, int height, I2C_HandleTypeDef oledI2c, uint8_t addr);

/*
 * Initalize display.
 */
void OLED_Init(struct SSD1360_d ssd1360_d);
/*
 * Empty screen.
 */
void Empty_Screen(struct SSD1360_d ssd1360_d);
/*
 * Fill screen.
 */
void Fill_Screen(struct SSD1360_d ssd1360_d);
/*
 * Turn display off
 */
void DisplayOff(struct SSD1360_d ssd1360_d);
/**
 * Turn display on.
 */
void DisplayOn(struct SSD1360_d ssd1360_d);
/*
 * Write string in position.
 */
void PrintString(struct SSD1360_d ssd1360_d, char* string, int y, int x);
/*
 * Print character in position;
 */
void PrintChar(struct SSD1360_d ssd1360_d, char ch, int y, int x);
/*
 * Write memory buffer to display.
 */
void OLED_Write_Display(struct SSD1360_d ssd1360_d);
/*
 * Update display buffer.
 */
void SetBytes(struct SSD1360_d ssd1360_d, uint8_t* ch, int y, int x);

#endif /* SSD1360_H_ */
