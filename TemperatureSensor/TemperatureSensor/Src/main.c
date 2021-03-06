#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_adc.h"
#include <string.h>

ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c2;

const uint8_t FONT_DOT[] = { 0x00, 0x60, 0x60, 0x00 };
const uint8_t FONT_PLUS[] = { 0x00, 0x10, 0x38, 0x10, 0x00 };
const uint8_t FONT_MINUS[] = { 0x00, 0x10, 0x10, 0x10, 0x00 };
const uint8_t FONT_ZERO[] = { 0x00, 0x3C, 0x42, 0x42, 0x3C, 0x00 };
const uint8_t FONT_ONE[] = { 0x00, 0x04, 0x7E, 0x00 };
const uint8_t FONT_TWO[] = { 0x00, 0x44, 0x62, 0x50, 0x4E, 0x00 };
const uint8_t FONT_THREE[] = { 0x00, 0x4A, 0x4A, 0x7E, 0x00 };
const uint8_t FONT_FOUR[] = { 0x00, 0x0E, 0x08, 0x08, 0x7E, 0x00 };
const uint8_t FONT_FIVE[] = { 0x00, 0x4E, 0x4A, 0x4A, 0x32, 0x00 };
const uint8_t FONT_SIX[] = { 0x00, 0x3C, 0x52, 0x52, 0x34, 0x00 };
const uint8_t FONT_SEVEN[] = { 0x00, 0x62, 0x12, 0x0A, 0x06, 0x00 };
const uint8_t FONT_EIGHT[] = { 0x00, 0x7E, 0x4A, 0x4A, 0x7E, 0x00 };
const uint8_t FONT_NINE[] = { 0x00, 0x4E, 0x4A, 0x4A, 0x7E, 0x00 };
const uint8_t FONT_A[] = { 0x00, 0x7E, 0x12, 0x12, 0x12, 0x12, 0x7E, 0x00 };
const uint8_t FONT_E[] = { 0x00, 0x7E, 0x4A, 0x4A, 0x42, 0x42, 0x00 };
const uint8_t FONT_M[] = { 0x00, 0x7E, 0x04, 0x18, 0x18, 0x04, 0x7E, 0x00 };
const uint8_t FONT_P[] = { 0x00, 0x7E, 0x0A, 0x0A, 0x0A, 0x0A, 0x0E, 0x00 };
const uint8_t FONT_R[] = { 0x00, 0x7E, 0x0A, 0x0A, 0x1A, 0x2A, 0x4E, 0x00 };
const uint8_t FONT_T[] = { 0x00, 0x02, 0x02, 0x7E, 0x02, 0x02, 0x00 };
const uint8_t FONT_U[] = { 0x00, 0x7E, 0x40, 0x40, 0x40, 0x40, 0x7E, 0x00 };

typedef struct font_t {
	uint8_t* data;
	uint8_t size_t;
} Font;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_ADC1_Init(void);

void Init_OLED(I2C_HandleTypeDef i2cHandle, uint8_t addr);
void SSD1306_I2C_Write(I2C_HandleTypeDef i2cHandle, uint8_t addr, uint8_t reg,
		uint8_t data);
void SSD1306_UpdateScreen(I2C_HandleTypeDef i2cHandle, uint8_t addr);
Font getCharacter(char chr);
void writeLine(char* str, int line, int padPixels);

#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64
#define PIXELS_PER_LINE 128

uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

int main(void) {
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_I2C2_Init();
	MX_ADC1_Init();
	Init_OLED(hi2c2, (uint16_t) (0x3c << 1));
	SSD1306_UpdateScreen(hi2c2, (uint16_t) (0x3c << 1));

	char bufferResult[50];
	while (1) {
		HAL_ADC_Start(&hadc1);
		if (HAL_ADC_PollForConversion(&hadc1, 1000000) == HAL_OK) {
			memset(SSD1306_Buffer, 0x00, SSD1306_WIDTH * SSD1306_HEIGHT / 8);
			writeLine("TEMPERATUR", 0, 0);
			uint32_t tempADCValue = HAL_ADC_GetValue(&hadc1);
			float temperature = -50.0 + ((((float) tempADCValue) - 1755.0) * 100.0 / 841.0);
			sprintf(bufferResult, "%.1f", temperature);
			writeLine(bufferResult, 3, 8);
			SSD1306_UpdateScreen(hi2c2, (uint16_t) (0x3c << 1));
			HAL_Delay(2000);
		}
	}
}

void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	__HAL_RCC_PWR_CLK_ENABLE()
	;
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void) {

	ADC_ChannelConfTypeDef sConfig;
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 16;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}
}

/* I2C2 init function */
static void MX_I2C2_Init(void) {

	hi2c2.Instance = I2C2;
	hi2c2.Init.ClockSpeed = 100000;
	hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_ENABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}
	HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0x01);
}

static void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOH_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;
	__ADC1_CLK_ENABLE()
	;

	/*Configure GPIO pins : PA4 */
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : PB3 PB10 */
	GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	__HAL_RCC_I2C2_CLK_ENABLE()
	;
}

void Init_OLED(I2C_HandleTypeDef i2cHandle, uint8_t addr) {
	HAL_Delay(100);
	SSD1306_I2C_Write(i2cHandle, addr, 0x00, 0xAE); //Display off
	SSD1306_I2C_Write(i2cHandle, addr, 0x00, 0xD5); //Set Display Clock Divide Ratio/ Oscillator Frequency
	SSD1306_I2C_Write(i2cHandle, addr, 0x00, 0x80); //ID: Display clock and oscillator freq
	SSD1306_I2C_Write(i2cHandle, addr, 0x00, 0xA8); //Set multiplexer ratio
	SSD1306_I2C_Write(i2cHandle, addr, 0x00, 0x3F); //ID:Multiplexer ratio
	SSD1306_I2C_Write(i2cHandle, addr, 0x00, 0xD3); // Display offset
	SSD1306_I2C_Write(i2cHandle, addr, 0x00, 0x00); //ID: Display offset
	SSD1306_I2C_Write(i2cHandle, addr, 0x00, 0x40); //Start address of display memory
	SSD1306_I2C_Write(i2cHandle, addr, 0x00, 0x8D); //Set Charge Pump
	SSD1306_I2C_Write(i2cHandle, addr, 0x00, 0x14); //Charge Pump Setting
	SSD1306_I2C_Write(i2cHandle, addr, 0x00, 0xA1); //ID:Remap segment
	SSD1306_I2C_Write(i2cHandle, addr, 0x00, 0xC8); //
	SSD1306_I2C_Write(i2cHandle, addr, 0x00, 0xDA); //Set COM Pins Hardware Configuration
	SSD1306_I2C_Write(i2cHandle, addr, 0x00, 0x12);
	SSD1306_I2C_Write(i2cHandle, addr, 0x00, 0x81); //Set contrast
	SSD1306_I2C_Write(i2cHandle, addr, 0x00, 0xFF); //Contrast
	SSD1306_I2C_Write(i2cHandle, addr, 0x00, 0xD9); //Set pre charge period
	SSD1306_I2C_Write(i2cHandle, addr, 0x00, 0xF1); //Pre charge period
	SSD1306_I2C_Write(i2cHandle, addr, 0x00, 0xDB); //Set VCOMH Deselect Level
	SSD1306_I2C_Write(i2cHandle, addr, 0x00, 0x40); //Set display start line
	SSD1306_I2C_Write(i2cHandle, addr, 0x00, 0xA4); //Display on
	SSD1306_I2C_Write(i2cHandle, addr, 0x00, 0xA6); // Not inverted display
	HAL_Delay(100);
	SSD1306_I2C_Write(i2cHandle, addr, 0x00, 0xAF); //turn on SSD1306 panel
}

void SSD1306_I2C_Write(I2C_HandleTypeDef i2cHandle, uint8_t addr, uint8_t reg,
		uint8_t data) {
	uint8_t cmd[2] = { reg, data };
	HAL_I2C_Master_Transmit(&i2cHandle, addr, cmd, 2, 100);
}

void writeLine(char* str, int line, int padPixels) {
	int currentX = padPixels*8;
	for (int chrIx = 0; chrIx < strlen(str); chrIx++) {
		char chr = str[chrIx];
		Font data = getCharacter(chr);
		for (int i = 0; i < data.size_t; i++) {
			SSD1306_Buffer[currentX + i + (PIXELS_PER_LINE*line)] = data.data[i];
		}
		currentX += data.size_t;
	}
}

Font getCharacter(char chr) {
	Font data;
	switch (chr) {
	case '-':
		data.size_t=sizeof(FONT_MINUS);
		data.data = FONT_MINUS;
		break;
	case '.':
		data.size_t=sizeof(FONT_DOT);
		data.data = FONT_DOT;
		break;
	case '0':
		data.size_t=sizeof(FONT_ZERO);
		data.data = FONT_ZERO;
		break;
	case '1':
		data.size_t=sizeof(FONT_ONE);
		data.data = FONT_ONE;
		break;
	case '2':
		data.size_t=sizeof(FONT_TWO);
		data.data = FONT_TWO;
		break;
	case '3':
		data.size_t=sizeof(FONT_THREE);
		data.data = FONT_THREE;
		break;
	case '4':
		data.size_t=sizeof(FONT_FOUR);
		data.data = FONT_FOUR;
		break;
	case '5':
		data.size_t=sizeof(FONT_FIVE);
		data.data = FONT_FIVE;
		break;
	case '6':
		data.size_t=sizeof(FONT_SIX);
		data.data = FONT_SIX;
		break;
	case '7':
		data.size_t=sizeof(FONT_SEVEN);
		data.data = FONT_SEVEN;
		break;
	case '8':
		data.size_t=sizeof(FONT_EIGHT);
		data.data = FONT_EIGHT;
		break;
	case '9':
		data.size_t=sizeof(FONT_NINE);
		data.data = FONT_NINE;
		break;
	case 'A':
		data.size_t=sizeof(FONT_A);
		data.data = FONT_A;
		break;
	case 'E':
		data.size_t=sizeof(FONT_E);
		data.data = FONT_E;
		break;
	case 'M':
		data.size_t=sizeof(FONT_M);
		data.data = FONT_M;
		break;
	case 'P':
		data.size_t=sizeof(FONT_P);
		data.data = FONT_P;
		break;
	case 'R':
		data.size_t=sizeof(FONT_R);
		data.data = FONT_R;
		break;
	case 'T':
		data.size_t=sizeof(FONT_T);
		data.data = FONT_T;
		break;
	case 'U':
		data.size_t=sizeof(FONT_U);
		data.data = FONT_U;
		break;
	}
	return data;
}

void SSD1306_UpdateScreen(I2C_HandleTypeDef i2cHandle, uint8_t addr) {
	uint8_t i;
	for (i = 0; i < 8; i++) {
		SSD1306_I2C_Write(i2cHandle, addr, 0x00, 0xb0 + i);
		SSD1306_I2C_Write(i2cHandle, addr, 0x00, 0x02);
		SSD1306_I2C_Write(i2cHandle, addr, 0x00, 0x10);
		HAL_I2C_Mem_Write(&i2cHandle, addr, 0x40, 1,
				&SSD1306_Buffer[SSD1306_WIDTH * i], SSD1306_WIDTH, 100);
	}
}

void _Error_Handler(char * file, int line) {
	while (1) {
	}
}

#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t* file, uint32_t line)
{

}

#endif
