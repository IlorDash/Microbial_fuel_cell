/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "SX1278.h"
#include "extEEPROM.h"

#include <stdio.h>
#include <string.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define START_MARK 's'
#define FINISH_MARK 'f'

#define CO2_LENGTH 6
#define TEMP_LENGTH 5
#define HUMID_LENGTH 4

#define MEAS_TX_BUFF_LENGTH 17
#define DATA_FRAME_LENGTH 32
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
bool parseMeas(char buff[], uint8_t len, char _co2[], char _temp[], char _humid[]) {
	int pos = 0;
	while (buff[pos] != START_MARK) {
		pos++;
	}
	if ((buff[pos] != START_MARK) || (buff[pos + CO2_LENGTH + TEMP_LENGTH + HUMID_LENGTH + 1] != FINISH_MARK)) {
		return false;
	}
	buff += pos + 1;
	strncpy(_co2, buff, CO2_LENGTH);
	strncpy(_temp, buff + CO2_LENGTH, TEMP_LENGTH);
	strncpy(_humid, buff + CO2_LENGTH + TEMP_LENGTH, HUMID_LENGTH);
	return true;
}

typedef struct {
	uint8_t Hour;

	uint8_t Month;
	uint8_t Date;
	uint8_t Year;

} dateTypeDef;

struct dataFrameStruct {
	uint16_t measNum;

	dateTypeDef date; // measNum(5) + hour(3) + date(3) + month(3) + year(3) + co2(5) + temp(5, with sign) + humid(3) = 30 bytes in string

	char co2[CO2_LENGTH + 1];
	char temp[TEMP_LENGTH + 1];
	char humid[HUMID_LENGTH + 1];
};

struct EEPROM_struct {
	uint8_t curDataFrameNum;
	uint16_t curPageNum;

	char txBuff[PAGE_SIZE];			// size is equal to page size, because reading from EEPROM by one page
	char rxBuff[DATA_FRAME_LENGTH]; // size is equal to data frame size
};

dataFrameStruct dataFrame;
EEPROM_struct EEPROM_data;

int i;
int loraStatus = 0;
SX1278_t SX1278;
char LoRaRxBuff[32] = "";
char LoRaTxBuff[32] = "";

char UartTxBuff[64] = "";
char UartRxBuff[32] = "";
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_SPI1_Init();
	MX_SPI2_Init();
	MX_USART1_UART_Init();
	MX_I2C1_Init();
	MX_RTC_Init();
	/* USER CODE BEGIN 2 */
	SX1278_hw_t SX1278_pins;

	SX1278_pins.dio0.pin = LORA_DIO0_Pin;
	SX1278_pins.dio0.port = LORA_DIO0_GPIO_Port;
	SX1278_pins.nss.pin = LORA_NSS_Pin;
	SX1278_pins.nss.port = LORA_NSS_GPIO_Port;
	SX1278_pins.reset.pin = LORA_RST_Pin;
	SX1278_pins.reset.port = LORA_RST_GPIO_Port;
	SX1278_pins.spi = &hspi1;

	SX1278.hw = &SX1278_pins;

	SX1278_begin(&SX1278, SX1278_433MHZ, SX1278_POWER_17DBM, SX1278_LORA_SF_8, SX1278_LORA_BW_20_8KHZ, 10);

	loraStatus = SX1278_LoRaEntryRx(&SX1278, MEAS_TX_BUFF_LENGTH, 2000);
	HAL_Delay(100);

	//*************************************
	// HAL_RTC_SetDate()		//write func to set actual time via UART
	//*************************************

	/* USER CODE END 2 */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */
		loraStatus = SX1278_LoRaRxPacket(&SX1278);

		if (loraStatus == MEAS_TX_BUFF_LENGTH) {

			HAL_GPIO_WritePin(LORA_TX_GPIO_Port, LORA_TX_Pin, GPIO_PIN_SET);

			RTC_TimeTypeDef sTime = {0};
			RTC_DateTypeDef DateToUpdate = {0};

			HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN); // RTC_FORMAT_BIN , RTC_FORMAT_BCD
			HAL_RTC_GetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN);

			dataFrame.date.Date = DateToUpdate.Date;
			dataFrame.date.Month = DateToUpdate.Month;
			dataFrame.date.Year = DateToUpdate.Year;
			dataFrame.date.Hour = sTime.Hours;

			dataFrame.measNum++;

			SX1278_read(&SX1278, (uint8_t *)LoRaRxBuff, loraStatus);

			bool parseResult = parseMeas(LoRaRxBuff, strlen(LoRaRxBuff), dataFrame.co2, dataFrame.temp, dataFrame.humid);
			if (parseResult) {
				sprintf(UartTxBuff, "#%d H:%d D:%02d M:%02d Y:%04d CO2:%s TEMP:%s HUMID:%s\r\n", dataFrame.measNum, dataFrame.date.Hour, dataFrame.date.Date,
						dataFrame.date.Month, dataFrame.date.Year, dataFrame.co2, dataFrame.temp, dataFrame.humid);
				HAL_UART_Transmit(&huart1, (uint8_t *)UartTxBuff, strlen(UartTxBuff), 1000);

				sprintf(EEPROM_data.txBuff + EEPROM_data.curDataFrameNum * DATA_FRAME_LENGTH, "%-5d%-3d%-3d%-3d%-3d%s%s%s", dataFrame.measNum,
						dataFrame.date.Date, dataFrame.date.Month, dataFrame.date.Year, dataFrame.date.Hour, dataFrame.co2, dataFrame.temp, dataFrame.humid);
				EEPROM_data.curDataFrameNum++;
				if (EEPROM_data.curDataFrameNum == (PAGE_SIZE / DATA_FRAME_LENGTH)) {
					HAL_GPIO_WritePin(STM_READY_GPIO_Port, STM_READY_Pin, GPIO_PIN_SET);
					bool writeResult = extEEPROM_writePage(EEPROM_data.curPageNum, (uint8_t *)EEPROM_data.txBuff, DEFAULT_NUM_ATTEMPTS);
					EEPROM_data.curPageNum++;
					EEPROM_data.curDataFrameNum = 0;
					if (!writeResult) {
						sprintf(UartTxBuff, "Failed to write %d page", EEPROM_data.curPageNum);
						HAL_UART_Transmit(&huart1, (uint8_t *)UartTxBuff, strlen(UartTxBuff), 1000);
						while (1) {
						}
					}

					EEPROM_data.curPageNum++;
					HAL_GPIO_WritePin(STM_READY_GPIO_Port, STM_READY_Pin, GPIO_PIN_RESET);
				}
			}
			HAL_Delay(500);
			HAL_GPIO_WritePin(LORA_TX_GPIO_Port, LORA_TX_Pin, GPIO_PIN_RESET);
		}
		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 400000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */
}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void) {

	/* USER CODE BEGIN RTC_Init 0 */

	/* USER CODE END RTC_Init 0 */

	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef DateToUpdate = {0};

	/* USER CODE BEGIN RTC_Init 1 */

	/* USER CODE END RTC_Init 1 */
	/** Initialize RTC Only
	 */
	hrtc.Instance = RTC;
	hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
	hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
	if (HAL_RTC_Init(&hrtc) != HAL_OK) {
		Error_Handler();
	}

	/* USER CODE BEGIN Check_RTC_BKUP */

	/* USER CODE END Check_RTC_BKUP */

	/** Initialize RTC and set the Time and Date
	 */
	sTime.Hours = 10;
	sTime.Minutes = 34;
	sTime.Seconds = 0;

	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
		Error_Handler();
	}
	DateToUpdate.WeekDay = RTC_WEEKDAY_WEDNESDAY;
	DateToUpdate.Month = RTC_MONTH_OCTOBER;
	DateToUpdate.Date = 6;
	DateToUpdate.Year = 0;

	if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN RTC_Init 2 */

	/* USER CODE END RTC_Init 2 */
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */
}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void) {

	/* USER CODE BEGIN SPI2_Init 0 */

	/* USER CODE END SPI2_Init 0 */

	/* USER CODE BEGIN SPI2_Init 1 */

	/* USER CODE END SPI2_Init 1 */
	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI2_Init 2 */

	/* USER CODE END SPI2_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LORA_RST_Pin | LORA_NSS_Pin | ET_SCSn_Pin | ET_RSTn_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, ET_INTn_Pin | LORA_TX_Pin | ERROR_Pin | STM_READY_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : LORA_RST_Pin LORA_NSS_Pin ET_SCSn_Pin ET_RSTn_Pin */
	GPIO_InitStruct.Pin = LORA_RST_Pin | LORA_NSS_Pin | ET_SCSn_Pin | ET_RSTn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : LORA_DIO0_Pin */
	GPIO_InitStruct.Pin = LORA_DIO0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(LORA_DIO0_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : ET_INTn_Pin LORA_TX_Pin ERROR_Pin STM_READY_Pin */
	GPIO_InitStruct.Pin = ET_INTn_Pin | LORA_TX_Pin | ERROR_Pin | STM_READY_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
