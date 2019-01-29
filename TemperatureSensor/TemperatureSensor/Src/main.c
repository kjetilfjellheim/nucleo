
#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_adc.h"
#include <string.h>

ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c2;

const uint8_t FONT_T[] = { 0x00, 0x06, 0x06, 0x7E, 0x7E, 0x06, 0x06, 0x00 };
const uint8_t FONT_E[] = { 0x00, 0x7E, 0x7E, 0x5A, 0x5A, 0x42, 0x42, 0x00 };


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_ADC1_Init(void);

void Init_OLED(I2C_HandleTypeDef i2cHandle, uint8_t addr);
void SSD1306_I2C_Write(I2C_HandleTypeDef i2cHandle, uint8_t addr,
		uint8_t reg, uint8_t data);
void SSD1306_UpdateScreen(I2C_HandleTypeDef i2cHandle, uint8_t addr);
void SSD1306_Fill();

#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64

static uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

int main(void) {
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_I2C2_Init();
	MX_ADC1_Init();
	Init_OLED(hi2c2, (uint16_t)(0x3c<<1));
	SSD1306_Fill();
	SSD1306_UpdateScreen(hi2c2, (uint16_t)(0x3c<<1));

	char bufferResult[50];
	while (1) {
		SSD1306_UpdateScreen(hi2c2, (uint16_t)(0x3c<<1));
		HAL_ADC_Start(&hadc1);
		if (HAL_ADC_PollForConversion(&hadc1, 1000000) == HAL_OK) {
			uint32_t tempADCValue = HAL_ADC_GetValue(&hadc1);
			float temperature =-150.0+((((float)tempADCValue)-1686.0)/841.0)*100.0;
			sprintf(bufferResult, "%.1f", temperature);
		}
	}

}

void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	__HAL_RCC_PWR_CLK_ENABLE();
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
	hadc1.Init.NbrOfConversion = 1;
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
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__ADC1_CLK_ENABLE();

	/*Configure GPIO pins : PA4 */
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : PB3 PB10 */
	GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
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

void SSD1306_I2C_Write(I2C_HandleTypeDef i2cHandle, uint8_t addr,
		uint8_t reg, uint8_t data) {
	uint8_t cmd[2] = { reg, data };
	HAL_I2C_Master_Transmit(&i2cHandle, addr, cmd, 2, 100);
}

void SSD1306_Fill() {
	memset(SSD1306_Buffer, 0x00, SSD1306_WIDTH * SSD1306_HEIGHT / 8);
	for (int i = 0; i < 8; i++) {
		SSD1306_Buffer[i] = FONT_T[i];
	}
	for (int i = 0; i < 8; i++) {
		SSD1306_Buffer[i+8] = FONT_E[i];
	}
}

void SSD1306_UpdateScreen(I2C_HandleTypeDef i2cHandle, uint8_t addr) {
	uint8_t i;
	for (i = 0; i < 8; i++) {
		SSD1306_I2C_Write(i2cHandle, addr, 0x00, 0xb0 + i);
		SSD1306_I2C_Write(i2cHandle, addr, 0x00, 0x02);
		SSD1306_I2C_Write(i2cHandle, addr, 0x00, 0x10);
		HAL_I2C_Mem_Write(&i2cHandle, addr, 0x40, 1, &SSD1306_Buffer[SSD1306_WIDTH * i], SSD1306_WIDTH, 100);
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
