#include "main.h"
#include <string.h>

#define MEASUREMENTS 100000

#define RS_INS 0x00
#define RS_DATA 0x40
#define SPI_COMMAND_SIZE 3

typedef struct
{
	uint32_t pulse;
	uint32_t period;
} measurement_t;

typedef struct
{
	float kelvin;
	float celsius;
} temperature_t;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
void InitDisplay(void);
void TransmitSpi(uint8_t rs, uint8_t data);
void SetCursor(uint8_t y, uint8_t x);
void ClearScreen();
void PrintString(char* str, int len);
void ResetDisplay();
void PrintValues();
measurement_t ReadPwmValues();
temperature_t CalculateTemperature(measurement_t);
void WriteCelsius(float celsius);
void WriteKelvin(float kelvin);

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_SPI1_Init();
	MX_TIM1_Init();
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	HAL_Delay(1000);
	InitDisplay();
	ClearScreen();
	SetCursor(0, 0);
	PrintString("Temperatur", 10);
	while (1) {
		measurement_t measurement = ReadPwmValues();
		temperature_t temperature = CalculateTemperature(measurement);
		WriteCelsius(temperature.celsius);
		WriteKelvin(temperature.kelvin);
	}
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1;
	PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
	hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_LSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {
	TIM_SlaveConfigTypeDef sSlaveConfig = { 0 };
	TIM_IC_InitTypeDef sConfigIC = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0xFFFF;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_IC_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
	sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
	sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
	sSlaveConfig.TriggerFilter = 0;
	if (HAL_TIM_SlaveConfigSynchronization(&htim1, &sSlaveConfig) != HAL_OK) {
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
	sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
	if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	HAL_TIM_Base_Init(&htim1);
	HAL_TIM_Base_Start(&htim1);
	__HAL_RCC_TIM1_CLK_ENABLE()
	;

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 38400;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOF_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_TIM1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	__HAL_RCC_SPI1_CLK_ENABLE()
	;

}

void InitDisplay(void) {
	HAL_Delay(500);
	ResetDisplay();
	HAL_Delay(20);
	TransmitSpi(RS_INS, 0x08);
	TransmitSpi(RS_INS, 0x3A);
	TransmitSpi(RS_INS, 0x09);
	TransmitSpi(RS_INS, 0x06);
	TransmitSpi(RS_INS, 0x1E);
	TransmitSpi(RS_INS, 0x39);
	TransmitSpi(RS_INS, 0x1B);
	TransmitSpi(RS_INS, 0x6E);
	TransmitSpi(RS_INS, 0x56);
	TransmitSpi(RS_INS, 0x70);
	TransmitSpi(RS_INS, 0x38);

	TransmitSpi(RS_INS, 0x3A);
	TransmitSpi(RS_INS, 0x17);
	TransmitSpi(RS_INS, 0x3C);

	TransmitSpi(RS_INS, 0x0C);

}

void TransmitSpi(uint8_t rs, uint8_t data) {
	HAL_Delay(5);
	uint8_t msb = (data & 0xF0) >> 4;
	uint8_t lsb = (data & 0x0F);
	uint8_t cmd[SPI_COMMAND_SIZE] = { 0x1F | rs, lsb, msb };
	HAL_SPI_Transmit(&hspi1, cmd, SPI_COMMAND_SIZE, 100);
	HAL_Delay(5);
}

void SetCursor(uint8_t y, uint8_t x) {
	TransmitSpi(RS_INS, (uint8_t) 0x80 + (0x20 * y) + x);
}

void ClearScreen() {
	TransmitSpi(RS_INS, 0x01);
}

void PrintString(char* str, int len) {
	for (int i = 0; i < len; i++) {
		TransmitSpi(RS_DATA, str[i]);
	}
}

void ResetDisplay() {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_Delay(20);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_Delay(20);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
	HAL_Delay(20);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_Delay(20);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
	HAL_Delay(20);
}

measurement_t ReadPwmValues() {
	measurement_t totalMeasurement;
	totalMeasurement.period = 0;
	totalMeasurement.pulse = 0;
	for (int x=0;x<MEASUREMENTS;x++) {
		totalMeasurement.period += HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);
		totalMeasurement.pulse += HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_2);
	}
	return totalMeasurement;
}

temperature_t CalculateTemperature(measurement_t measurement) {
	temperature_t temperature;
	double dc = ((double)measurement.pulse/(double)measurement.period);
	temperature.celsius = -1.43 * dc*dc + 214.56 * dc - 68.6;
	temperature.kelvin = temperature.celsius + 273.15;
	return temperature;
}

void WriteCelsius(float celsius) {
	SetCursor(1, 0);
	char str[10]="          ";
	sprintf(str, "%1.3fC", celsius);
	PrintString("   ", 1);
	PrintString(str, strlen(str));
	PrintString("          ", 10 - strlen(str) - 1);
}

void WriteKelvin(float kelvin) {
	SetCursor(2, 0);
	char str[10]="          ";
	sprintf(str, "%1.2fK", kelvin);
	PrintString("   ", 1);
	PrintString(str, strlen(str));
	PrintString("          ", 10 - strlen(str) - 1);
}


/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(char *file, uint32_t line)
{

}
#endif /* USE_FULL_ASSERT */

