/*
 * ssd1360.c
 *
 *  Created on: 4 Nov 2017
 *      Author: Kjetil
 */

#include <string.h>
#include <stdlib.h>

#include "ssd1360.h"

uint8_t CHARACTER_A[] = { 0x00, 0x7C, 0x12, 0x12, 0x7C, 0x00 };
uint8_t CHARACTER_B[] = { 0x00, 0x7E, 0x4A, 0x4A, 0x3C, 0x00 };
uint8_t CHARACTER_C[] = { 0x00, 0x3C, 0x42, 0x42, 0x42, 0x00 };
uint8_t CHARACTER_D[] = { 0x00, 0x7E, 0x42, 0x42, 0x3C, 0x00 };
uint8_t CHARACTER_E[] = { 0x00, 0x7E, 0x4A, 0x4A, 0x42, 0x00 };
uint8_t CHARACTER_F[] = { 0x00, 0x7E, 0x0A, 0x0A, 0x0A, 0x00 };
uint8_t CHARACTER_G[] = { 0x00, 0x3C, 0x4A, 0x4A, 0x30, 0x00 };
uint8_t CHARACTER_H[] = { 0x00, 0x7E, 0x10, 0x10, 0x7E, 0x00 };
uint8_t CHARACTER_I[] = { 0x00, 0x00, 0x7A, 0x00, 0x00, 0x00 };
uint8_t CHARACTER_J[] = { 0x00, 0x22, 0x42, 0x42, 0x7E, 0x00 };
uint8_t CHARACTER_K[] = { 0x00, 0x7E, 0x18, 0x24, 0x42, 0x00 };
uint8_t CHARACTER_L[] = { 0x00, 0x7E, 0x40, 0x40, 0x40, 0x00 };
uint8_t CHARACTER_M[] = { 0x00, 0x7E, 0x0C, 0x0C, 0x7E, 0x00 };
uint8_t CHARACTER_N[] = { 0x00, 0x7E, 0x0C, 0x30, 0x7E, 0x00 };
uint8_t CHARACTER_O[] = { 0x00, 0x3C, 0x42, 0x42, 0x3C, 0x00 };
uint8_t CHARACTER_P[] = { 0x00, 0x7E, 0x0A, 0x0A, 0x0E, 0x00 };
uint8_t CHARACTER_Q[] = { 0x00, 0x3C, 0x42, 0x62, 0x7C, 0x00 };
uint8_t CHARACTER_R[] = { 0x00, 0x7E, 0x1A, 0x2A, 0x4E, 0x00 };
uint8_t CHARACTER_S[] = { 0x00, 0x4E, 0x4A, 0x4A, 0x7A, 0x00 };
uint8_t CHARACTER_T[] = { 0x00, 0x02, 0x7E, 0x02, 0x02, 0x00 };
uint8_t CHARACTER_U[] = { 0x00, 0x7E, 0x40, 0x40, 0x7E, 0x00 };
uint8_t CHARACTER_V[] = { 0x00, 0x1E, 0x60, 0x60, 0x1E, 0x00 };
uint8_t CHARACTER_W[] = { 0x00, 0x7E, 0x30, 0x38, 0x7E, 0x00 };
uint8_t CHARACTER_X[] = { 0x00, 0x42, 0x3C, 0x3C, 0x42, 0x00 };
uint8_t CHARACTER_Y[] = { 0x00, 0x06, 0x08, 0x78, 0x06, 0x00 };
uint8_t CHARACTER_Z[] = { 0x00, 0x62, 0x52, 0x4A, 0x46, 0x00 };
uint8_t CHARACTER_0[] = { 0x00, 0x7E, 0x42, 0x42, 0x7E, 0x00 };
uint8_t CHARACTER_1[] = { 0x00, 0x00, 0x44, 0x7E, 0x40, 0x00 };
uint8_t CHARACTER_2[] = { 0x00, 0x44, 0x62, 0x52, 0x4C, 0x00 };
uint8_t CHARACTER_3[] = { 0x00, 0x4A, 0x4A, 0x4A, 0x7E, 0x00 };
uint8_t CHARACTER_4[] = { 0x00, 0x0E, 0x08, 0x08, 0x7E, 0x00 };
uint8_t CHARACTER_5[] = { 0x00, 0x4E, 0x4A, 0x4A, 0x7A, 0x00 };
uint8_t CHARACTER_6[] = { 0x00, 0x3C, 0x4A, 0x4A, 0x72, 0x00 };
uint8_t CHARACTER_7[] = { 0x00, 0x02, 0x02, 0x72, 0x0E, 0x00 };
uint8_t CHARACTER_8[] = { 0x00, 0x7E, 0x4A, 0x4A, 0x7E, 0x00 };
uint8_t CHARACTER_9[] = { 0x00, 0x0E, 0x0A, 0x0A, 0x7E, 0x00 };
uint8_t CHARACTER_DECIMAL[] = { 0x00, 0x00, 0x60, 0x60, 0x00, 0x00 };

void sendOledCommand(I2C_HandleTypeDef oledI2c, uint8_t addr, uint8_t regAddr, uint8_t pData) {
	uint8_t cmd[2] = { regAddr, pData };
	HAL_I2C_Master_Transmit(&oledI2c, addr << 1, cmd, 2, 100);
}

struct SSD1360_d Init(int width, int height, I2C_HandleTypeDef oledI2c,
		uint8_t addr) {
	struct SSD1360_d ssd1360_d;
	ssd1360_d.width = width;
	ssd1360_d.height = height;
	ssd1360_d.oledI2c = oledI2c;
	ssd1360_d.addr = addr;
	ssd1360_d.buffer = malloc(width * height / 8);
	ssd1360_d.bufferSize = width * height / 8;
	return ssd1360_d;
}

void OLED_Init(struct SSD1360_d ssd1360_d) {
	HAL_Delay(100);
	sendOledCommand(ssd1360_d.oledI2c, ssd1360_d.addr, 0x00, 0xAE); //Display off
	sendOledCommand(ssd1360_d.oledI2c, ssd1360_d.addr, 0x00, 0x20); //Set Memory Addressing Mode
	sendOledCommand(ssd1360_d.oledI2c, ssd1360_d.addr, 0x00, 0x00); //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	sendOledCommand(ssd1360_d.oledI2c, ssd1360_d.addr, 0x00, 0xB0); //Set Page Start Address for Page Addressing Mode,0-7
	sendOledCommand(ssd1360_d.oledI2c, ssd1360_d.addr, 0x00, 0xC8); //Set COM Output Scan Direction
	sendOledCommand(ssd1360_d.oledI2c, ssd1360_d.addr, 0x00, 0x00); //---set low column address
	sendOledCommand(ssd1360_d.oledI2c, ssd1360_d.addr, 0x00, 0x10); //---set high column address
	sendOledCommand(ssd1360_d.oledI2c, ssd1360_d.addr, 0x00, 0x40); //--set start line address
	sendOledCommand(ssd1360_d.oledI2c, ssd1360_d.addr, 0x00, 0x81); //--set contrast control register
	sendOledCommand(ssd1360_d.oledI2c, ssd1360_d.addr, 0x00, 0xFF);
	sendOledCommand(ssd1360_d.oledI2c, ssd1360_d.addr, 0x00, 0xA1); //--set segment re-map 0 to 127
	sendOledCommand(ssd1360_d.oledI2c, ssd1360_d.addr, 0x00, 0xA6); //--set normal display
	sendOledCommand(ssd1360_d.oledI2c, ssd1360_d.addr, 0x00, 0xA8); //--set multiplex ratio(1 to 64)
	sendOledCommand(ssd1360_d.oledI2c, ssd1360_d.addr, 0x00, 0x3F); //
	sendOledCommand(ssd1360_d.oledI2c, ssd1360_d.addr, 0x00, 0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	sendOledCommand(ssd1360_d.oledI2c, ssd1360_d.addr, 0x00, 0xD3); //-set display offset
	sendOledCommand(ssd1360_d.oledI2c, ssd1360_d.addr, 0x00, 0x00); //-not offset
	sendOledCommand(ssd1360_d.oledI2c, ssd1360_d.addr, 0x00, 0xD5); //--set display clock divide ratio/oscillator frequency
	sendOledCommand(ssd1360_d.oledI2c, ssd1360_d.addr, 0x00, 0xF0); //--set divide ratio
	sendOledCommand(ssd1360_d.oledI2c, ssd1360_d.addr, 0x00, 0xD9); //--set pre-charge period
	sendOledCommand(ssd1360_d.oledI2c, ssd1360_d.addr, 0x00, 0x22); //
	sendOledCommand(ssd1360_d.oledI2c, ssd1360_d.addr, 0x00, 0xDA); //--set com pins hardware configuration
	sendOledCommand(ssd1360_d.oledI2c, ssd1360_d.addr, 0x00, 0x12);
	sendOledCommand(ssd1360_d.oledI2c, ssd1360_d.addr, 0x00, 0xDB); //--set vcomh
	sendOledCommand(ssd1360_d.oledI2c, ssd1360_d.addr, 0x00, 0x20); //0x20,0.77xVcc
}

void OLED_Write_Display(struct SSD1360_d ssd1360_d) {
	for (uint8_t m = 0; m < 8; m++) {
		sendOledCommand(ssd1360_d.oledI2c, ssd1360_d.addr, 0x00, 0xB0 + m); //Set Page Start Address for Page Addressing Mode,0-7
		sendOledCommand(ssd1360_d.oledI2c, ssd1360_d.addr, 0x00, 0x00); //---set low column address
		sendOledCommand(ssd1360_d.oledI2c, ssd1360_d.addr, 0x00, 0x10); //---set high column address

		HAL_I2C_Mem_Write(&ssd1360_d.oledI2c, ssd1360_d.addr << 1, 0x40,
		I2C_MEMADD_SIZE_8BIT, &ssd1360_d.buffer[ssd1360_d.width * m],
				ssd1360_d.width, 1000);
	}
}

void Empty_Screen(struct SSD1360_d ssd1360_d) {
	memset(ssd1360_d.buffer, 0x00, ssd1360_d.bufferSize);
}

void Fill_Screen(struct SSD1360_d ssd1360_d) {
	memset(ssd1360_d.buffer, 0xFF, ssd1360_d.bufferSize);
}

void DisplayOff(struct SSD1360_d ssd1360_d) {
	sendOledCommand(ssd1360_d.oledI2c, ssd1360_d.addr, 0x00, 0x8D);
	sendOledCommand(ssd1360_d.oledI2c, ssd1360_d.addr, 0x00, 0x10);
	sendOledCommand(ssd1360_d.oledI2c, ssd1360_d.addr, 0x00, 0xAE);
}

void DisplayOn(struct SSD1360_d ssd1360_d) {
	sendOledCommand(ssd1360_d.oledI2c, ssd1360_d.addr, 0x00, 0x8D);
	sendOledCommand(ssd1360_d.oledI2c, ssd1360_d.addr, 0x00, 0x14);
	sendOledCommand(ssd1360_d.oledI2c, ssd1360_d.addr, 0x00, 0xAF);
}

void PrintString(struct SSD1360_d ssd1360_d, char* string, int y, int x) {
	for (int i = 0; i < strlen(string); i++) {
		char ch = string[i];
		PrintChar(ssd1360_d, ch, y, x + i);
	}
}

void SetBytes(struct SSD1360_d ssd1360_d, uint8_t* ch, int y, int x) {
	for (int i = 0; i < 6; i++) {
		int arrayStart = i + (x * 6) + (ssd1360_d.width * y);
		uint8_t* buffer = ssd1360_d.buffer;
		buffer[arrayStart] = ch[i];
	}
}

void PrintChar(struct SSD1360_d ssd1360_d, char ch, int y, int x) {
	switch (ch) {
	case 'A':
	case 'a':
		SetBytes(ssd1360_d, CHARACTER_A, y, x);
		break;
	case 'B':
	case 'b':
			SetBytes(ssd1360_d, CHARACTER_B, y, x);
			break;
	case 'C':
	case 'c':
			SetBytes(ssd1360_d, CHARACTER_C, y, x);
			break;
	case 'D':
	case 'd':
			SetBytes(ssd1360_d, CHARACTER_D, y, x);
			break;
	case 'E':
	case 'e':
			SetBytes(ssd1360_d, CHARACTER_E, y, x);
			break;
	case 'F':
	case 'f':
			SetBytes(ssd1360_d, CHARACTER_F, y, x);
			break;
	case 'G':
	case 'g':
			SetBytes(ssd1360_d, CHARACTER_G, y, x);
			break;
	case 'H':
	case 'h':
			SetBytes(ssd1360_d, CHARACTER_H, y, x);
			break;
	case 'I':
	case 'i':
			SetBytes(ssd1360_d, CHARACTER_I, y, x);
			break;
	case 'J':
	case 'j':
			SetBytes(ssd1360_d, CHARACTER_J, y, x);
			break;
	case 'K':
	case 'k':
			SetBytes(ssd1360_d, CHARACTER_K, y, x);
			break;
	case 'L':
	case 'l':
			SetBytes(ssd1360_d, CHARACTER_L, y, x);
			break;
	case 'M':
	case 'm':
			SetBytes(ssd1360_d, CHARACTER_M, y, x);
			break;
	case 'N':
	case 'n':
			SetBytes(ssd1360_d, CHARACTER_N, y, x);
			break;
	case 'O':
	case 'o':
			SetBytes(ssd1360_d, CHARACTER_O, y, x);
			break;
	case 'P':
	case 'p':
			SetBytes(ssd1360_d, CHARACTER_P, y, x);
			break;
	case 'Q':
	case 'q':
			SetBytes(ssd1360_d, CHARACTER_Q, y, x);
			break;
	case 'R':
	case 'r':
			SetBytes(ssd1360_d, CHARACTER_R, y, x);
			break;
	case 'S':
	case 's':
			SetBytes(ssd1360_d, CHARACTER_S, y, x);
			break;
	case 'T':
	case 't':
			SetBytes(ssd1360_d, CHARACTER_T, y, x);
			break;
	case 'U':
	case 'u':
			SetBytes(ssd1360_d, CHARACTER_U, y, x);
			break;
	case 'V':
	case 'v':
			SetBytes(ssd1360_d, CHARACTER_V, y, x);
			break;
	case 'W':
	case 'w':
			SetBytes(ssd1360_d, CHARACTER_W, y, x);
			break;
	case 'X':
	case 'x':
			SetBytes(ssd1360_d, CHARACTER_X, y, x);
			break;
	case 'Y':
	case 'y':
			SetBytes(ssd1360_d, CHARACTER_Y, y, x);
			break;
	case 'Z':
	case 'z':
			SetBytes(ssd1360_d, CHARACTER_Z, y, x);
			break;
	case '0':
			SetBytes(ssd1360_d, CHARACTER_0, y, x);
			break;
	case '1':
			SetBytes(ssd1360_d, CHARACTER_1, y, x);
			break;
	case '2':
			SetBytes(ssd1360_d, CHARACTER_2, y, x);
			break;
	case '3':
			SetBytes(ssd1360_d, CHARACTER_3, y, x);
			break;
	case '4':
			SetBytes(ssd1360_d, CHARACTER_4, y, x);
			break;
	case '5':
			SetBytes(ssd1360_d, CHARACTER_5, y, x);
			break;
	case '6':
			SetBytes(ssd1360_d, CHARACTER_6, y, x);
			break;
	case '7':
			SetBytes(ssd1360_d, CHARACTER_7, y, x);
			break;
	case '8':
			SetBytes(ssd1360_d, CHARACTER_8, y, x);
			break;
	case '9':
			SetBytes(ssd1360_d, CHARACTER_9, y, x);
			break;
	case ',':
			SetBytes(ssd1360_d, CHARACTER_DECIMAL, y, x);
			break;
	case '.':
			SetBytes(ssd1360_d, CHARACTER_DECIMAL, y, x);
			break;
	default:
		break;
	}
}

void _Error_Handler(char * file, int line) {
	while (1) {
	}
}
