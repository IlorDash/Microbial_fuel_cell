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
#include "net.h"
#include "W5500.h"
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
// #define START_MARK 's'
// #define FINISH_MARK 'f'

// #define CO2_LENGTH 6
// #define TEMP_LENGTH 5
// #define HUMID_LENGTH 4
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
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_DMA_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
void set_time(uint8_t curHours, uint8_t curMinutes, uint8_t curSeconds, uint8_t curWeekday, uint8_t curMonth, uint8_t curDate, uint16_t curYear);
void FLASH_Write(uint32_t dest, uint32_t *data, uint8_t wordsNum);
void resetFlashData();
bool executeCmd();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint16_t defaultYear = 21;
static uint8_t defaultMonth = 10;
static uint16_t defaultDate = 30;
static uint8_t defaultWeekday = 1;

static uint8_t defaultHours = 12;
static uint8_t defaultMinutes = 0;
static uint8_t defaultSeconds = 0;

RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;

typedef struct {
	uint8_t sensorID;

	uint8_t Hour;
	uint8_t Minute;

	uint8_t Month;
	uint8_t Date; // sensorID(2) + hour(3) + minute(2) + date(3) + month(3) + co2(5) + temp(5, with sign) + humid(3) = 28 bytes in string

	uint16_t co2;
	int16_t temp;
	uint8_t humid;
} __attribute__((packed)) TDataFrame, *PTDataFrame;

EEPROM_struct EEPROM_data;

TDataFrame dataFrame;

int i;
int loraStatus = 0;
SX1278_t SX1278;
char LoRaRxBuff[32] = "";
char LoRaTxBuff[32] = "";

char UartTxBuff[UART_TX_BUFF_LEN] = "";
char UartRxBuff[UART_RX_BUFF_LEN] = "";

const char *UART_pass = "BaseBlock271828";
const char *UART_rstDataCmd = "RstData";
const char *UART_setTimeCmd = "SetTime";

/*************************************************************************************/
/*UART commands example*/
/*BaseBlock271828 RstData - reseting collected in EEPROM data*/
/*BaseBlock271828 SetTime 12 35 5 1 21 - setting current time in format HH MM DD MM YY CRLF*/
/*("\r\n") - sign of end transmition*/

// char *measTestDump[] = {
// 	"1    10 9  10 5  42424+272 85",
// 	"2    11 10 11 6  42425-280 90",
// 	"3    12 11 13 7  4200 -27 100",
// 	"4    8  9  10 5  42424+272 85",
// 	"5    10 9  10 5  42424+272 85",
// 	"6    11 10 11 6  42425-280 90",
// 	"7    12 11 13 7  4200 -27 100",
// 	"8    8  9  10 5  42424+272 85",
// 	"9    10 9  10 5  42424+272 85",
// 	"10   11 10 11 6  42425-280 90",
// 	"11   12 11 13 7  4200 -27 100",
// 	"12   8  9  10 5  42424+272 85",
// 	"13   10 9  10 5  42424+272 85",
// 	"14   11 10 11 6  42425-280 90",
// 	"15   12 11 13 7  4200 -27 100",
// 	"16   8  9  10 5  42424+272 85",
// 	"17   10 9  10 5  42424+272 85",
// 	"18   11 10 11 6  42425-280 90",
// 	"19   11 10 11 6  42425-280 90",
// 	"20   12 11 13 7  4200 -27 100",
// 	"21   8  9  10 5  42424+272 85",
// 	"22   10 9  10 5  42424+272 85",
// 	"23   11 10 11 6  42425-280 90",
// 	"24   11 10 11 6  42425-280 90",
// 	"25   12 11 13 7  4200 -27 100",
// 	"26   8  9  10 5  42424+272 85",
// 	"27   10 9  10 5  42424+272 85",
// 	"28   11 10 11 6  42425-280 90",
// 	"29   12 11 13 7  4200 -27 100",
// 	"30   8  9  10 5  42424+272 85",
// 	"31   10 9  10 5  42424+272 85",
// 	"32   11 10 11 6  42425-280 90",
// 	"33   11 10 11 6  42425-280 90",
// 	"34   12 11 13 7  4200 -27 100",
// 	"35   8  9  10 5  42424+272 85",
// 	"36   10 9  10 5  42424+272 85",
// 	"37   11 10 11 6  42425-280 90"};

//"%-5d%-3d%-3d%-3d%-3d%-5d%+-3d%3d"

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
	MX_I2C1_Init();
	MX_DMA_Init();
	MX_USART1_UART_Init();
	MX_RTC_Init();
	/* USER CODE BEGIN 2 */

	//*************************************
	// HAL_RTC_SetDate()		//write func to set actual time via UART
	// set_time(defaultHours, defaultMinutes, defaultSeconds, defaultWeekday, defaultMonth, defaultDate, defaultYear);
	//*************************************

	// resetFlashData();

	if (!*(__IO uint32_t *)(FLASH_WRITE_DATA_FRAME_NUM_CHECK)) {
		EEPROM_data.curDataFrameNum = *(__IO uint32_t *)(FLASH_WRITE_DATA_FRAME_NUM_ADDR);
		EEPROM_data.curPageNum = EEPROM_data.curDataFrameNum / DATA_FRAME_PER_PAGE;
	}

	if (!*(__IO uint32_t *)(FLASH_WRITE_LAST_EEPROM_PAGE_DATA_CHECK)) {
		uint8_t *dest_addr = (uint8_t *)FLASH_WRITE_LAST_EEPROM_PAGE_DATA_ADDR;
		for (int i = 0; i < PAGE_SIZE; i++) {
			EEPROM_data.txBuff[i] = (char)(*dest_addr);
			dest_addr++;
		}
	}

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

	uint8_t chipVer = readChipVerReg();
	net_init();

	HAL_UART_Receive_DMA(&huart1, (uint8_t *)UartRxBuff, UART_RX_BUFF_LEN);

	/* USER CODE END 2 */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */
		if (UartRxBuff[strlen(UartRxBuff) - 1] == '\n') {
			if (executeCmd()) {
				char *successCmdExe = "Command executed successfully\r\n";
				HAL_UART_Transmit(&huart1, (uint8_t *)successCmdExe, strlen(successCmdExe), 1000);
			}
			// else{
			// 	char *failedCmdExe = "Command execution failed";
			// 	HAL_UART_Transmit(&huart1, (uint8_t *)failedCmdExe, strlen(failedCmdExe), 1000);
			// }
			memset(UartRxBuff, 0, strlen(UartRxBuff));
			HAL_UART_DMAStop(&huart1);
			HAL_UART_Receive_DMA(&huart1, (uint8_t *)UartRxBuff, UART_RX_BUFF_LEN);
		}

		net_poll();

		loraStatus = SX1278_LoRaRxPacket(&SX1278);
		if (loraStatus == MEAS_TX_BUFF_LENGTH) {

			HAL_GPIO_WritePin(LORA_TX_GPIO_Port, LORA_TX_Pin, GPIO_PIN_SET);

			HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

			dataFrame.Date = sDate.Date;
			dataFrame.Month = sDate.Month;

			dataFrame.Hour = sTime.Hours;
			dataFrame.Minute = sTime.Minutes;

			SX1278_read(&SX1278, (uint8_t *)LoRaRxBuff, loraStatus);

			int tSensorID = 0;
			int tCO2 = 0;
			int tTemp = 0;
			int tHumid = 0;

			int8_t parseResult = sscanf(LoRaRxBuff, "s%2d %5d %4d %3d f", &tSensorID, &tCO2, &tTemp, &tHumid);
			if (parseResult == MEAS_VARS_NUM) {
				dataFrame.sensorID = tSensorID;
				dataFrame.co2 = tCO2;
				dataFrame.temp = tTemp;
				dataFrame.humid = tHumid;

				sprintf(UartTxBuff, "#%02d Min:%02d Hour:%0d Date:%02d Month:%02d CO2:%-5d TEMP:%+-3d HUMID:%3d\r\n", dataFrame.sensorID, dataFrame.Minute,
						dataFrame.Hour, dataFrame.Date, dataFrame.Month, dataFrame.co2, dataFrame.temp, dataFrame.humid);
				HAL_UART_Transmit(&huart1, (uint8_t *)UartTxBuff, strlen(UartTxBuff), 1000);

				sprintf(EEPROM_data.txBuff + EEPROM_data.curDataFrameNum % DATA_FRAME_PER_PAGE * DATA_FRAME_LENGTH, "%-2d %-2d %-2d %-2d %-2d %-5d %+-3d %3d",
						dataFrame.sensorID, dataFrame.Minute,
						dataFrame.Hour, // dataframe len in EEPROM = 27 (plus null term)
						dataFrame.Date, dataFrame.Month, dataFrame.co2, dataFrame.temp, dataFrame.humid);

				EEPROM_data.curDataFrameNum++;

				FLASH_Write(FLASH_WRITE_DATA_FRAME_NUM_ADDR, &EEPROM_data.curDataFrameNum, 1);

				if (*(__IO uint32_t *)(FLASH_WRITE_DATA_FRAME_NUM_CHECK)) {
					uint32_t _null = 0;
					FLASH_Write(FLASH_WRITE_DATA_FRAME_NUM_CHECK, &_null, 1);
				}

				HAL_GPIO_WritePin(STM_READY_GPIO_Port, STM_READY_Pin, GPIO_PIN_SET);
				bool writeResult = extEEPROM_writePage(EEPROM_data.curPageNum, (uint8_t *)EEPROM_data.txBuff, DEFAULT_NUM_ATTEMPTS);

				if (!writeResult) {
					sprintf(UartTxBuff, "Failed to write %d page", EEPROM_data.curPageNum);
					HAL_UART_Transmit(&huart1, (uint8_t *)UartTxBuff, strlen(UartTxBuff), 1000);
					HAL_GPIO_WritePin(ERROR_GPIO_Port, ERROR_Pin, GPIO_PIN_SET);
					while (1) {
					}
				}

				HAL_GPIO_WritePin(STM_READY_GPIO_Port, STM_READY_Pin, GPIO_PIN_RESET);

				if ((EEPROM_data.curDataFrameNum % DATA_FRAME_PER_PAGE) == 0) {
					EEPROM_data.curPageNum++;
					uint32_t _null = 0;
					FLASH_Write(FLASH_WRITE_LAST_EEPROM_PAGE_DATA_ADDR, &_null, PAGE_SIZE / FLASH_WORD_LENGTH);

					if (*(__IO uint32_t *)(FLASH_WRITE_LAST_EEPROM_PAGE_DATA_CHECK)) {
						uint32_t _ones = 0xFFFFFFFF;
						FLASH_Write(FLASH_WRITE_LAST_EEPROM_PAGE_DATA_CHECK, &_ones, 1);
					}
				} else {
					FLASH_Write(FLASH_WRITE_LAST_EEPROM_PAGE_DATA_ADDR, (uint32_t *)EEPROM_data.txBuff, PAGE_SIZE / FLASH_WORD_LENGTH);

					if (*(__IO uint32_t *)(FLASH_WRITE_LAST_EEPROM_PAGE_DATA_CHECK)) {
						uint32_t _null = 0;
						FLASH_Write(FLASH_WRITE_LAST_EEPROM_PAGE_DATA_CHECK, &_null, 1);
					}
				}
			}
			HAL_Delay(500);
			HAL_GPIO_WritePin(LORA_TX_GPIO_Port, LORA_TX_Pin, GPIO_PIN_RESET);
		}
		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}
void getMeasTableRowFromEEPROM(char *htmlTable, uint16_t startDataFrame) {

	if (!EEPROM_data.curDataFrameNum) { // if eeprom is empty then leave func
		return;
	}

	bool readResult = extEEPROM_read(startDataFrame / DATA_FRAME_PER_PAGE, startDataFrame % DATA_FRAME_PER_PAGE * DATA_FRAME_LENGTH,
									 (uint8_t *)EEPROM_data.rxBuff, DATA_FRAME_LENGTH, DEFAULT_NUM_ATTEMPTS);
	if (!readResult) {
		sprintf(UartTxBuff, "Failed to read %d frame", i);
		HAL_UART_Transmit(&huart1, (uint8_t *)UartTxBuff, strlen(UartTxBuff), 1000);
		HAL_GPIO_WritePin(ERROR_GPIO_Port, ERROR_Pin, GPIO_PIN_SET);
		while (1) {
		}
	}
	// strcpy(EEPROM_data.rxBuff, measTestDump[i]);

	int tsensorID = 0;
	int tMinute = 0;
	int tHour = 0;
	int tDate = 0;
	int tMonth = 0;
	int tCO2 = 0;
	int tTemp = 0;
	int tHumid = 0;
	int8_t parseResult = sscanf(EEPROM_data.rxBuff, "%2u %2u %2u %2u %2u %5u %4d %3u", &tsensorID, &tMinute, &tHour, &tDate, &tMonth, &tCO2, &tTemp, &tHumid);

	int8_t tTempInt = tTemp / 10;
	uint8_t tTempFract = ((tTemp % 10) > 0) ? (tTemp % 10) : (-1 * tTemp % 10);

	if (parseResult != DATA_FRAME_VARS_NUM) {
		HAL_GPIO_WritePin(ERROR_GPIO_Port, ERROR_Pin, GPIO_PIN_SET);
		while (1) {
		}
	}
	char rowBuff[HTML_TABLE_MEAS_ROW_LEN + 1] = "0";
	sprintf(rowBuff, "<tr><td>%-2d</td><td>%-2d</td><td>%-2d</td><td>%-2d</td><td>%-2d</td><td>%-5d</td><td>%+-2d.%1d</td><td>%-3d</td></tr>", tsensorID,
			tMinute, tHour, tDate, tMonth, tCO2, tTempInt, tTempFract, tHumid);
	strncat(htmlTable, rowBuff, strlen(rowBuff));
}

void set_time(uint8_t curHours, uint8_t curMinutes, uint8_t curSeconds, uint8_t curWeekday, uint8_t curMonth, uint8_t curDate, uint16_t curYear) {
	sTime.Hours = curHours;		// set hours
	sTime.Minutes = curMinutes; // set minutes
	sTime.Seconds = curSeconds; // set seconds
	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
		char *RTC_failedSetTimeMsg = "Failed to set time to RTC";
		HAL_UART_Transmit(&huart1, (uint8_t *)RTC_failedSetTimeMsg, strlen(RTC_failedSetTimeMsg), 1000);
		Error_Handler();
	}
	sDate.WeekDay = curWeekday; // week day
	sDate.Month = curMonth;		// month
	sDate.Date = curDate;		// date
	sDate.Year = curYear;		// year
	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
		char *RTC_failedSetTimeMsg = "Failed to set time to RTC";
		HAL_UART_Transmit(&huart1, (uint8_t *)RTC_failedSetTimeMsg, strlen(RTC_failedSetTimeMsg), 1000);
		Error_Handler();
	}
}

void FLASH_Write(uint32_t dest, uint32_t *data, uint8_t wordsNum) {
	HAL_FLASH_Unlock();
	FLASH_EraseInitTypeDef FLASH_Erase_config;
	FLASH_Erase_config.TypeErase = FLASH_TYPEERASE_PAGES;
	FLASH_Erase_config.PageAddress = dest;
	FLASH_Erase_config.NbPages = 1;
	uint32_t pageError = 0;
	HAL_FLASHEx_Erase(&FLASH_Erase_config, &pageError);

	uint32_t *dest_addr = (uint32_t *)dest;

	for (int i = 0; i < wordsNum; i++) {
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)dest_addr, *data);
		data++;
		dest_addr++;
	}

	HAL_FLASH_Lock();
}

void resetFlashData() {
	EEPROM_data.curPageNum = 0;
	EEPROM_data.curDataFrameNum = 0;
	memset(EEPROM_data.txBuff, 0, strlen(EEPROM_data.txBuff));

	uint32_t _ones = 0xFFFFFFFF;
	FLASH_Write(FLASH_WRITE_DATA_FRAME_NUM_ADDR, &_ones, 1);
	FLASH_Write(FLASH_WRITE_DATA_FRAME_NUM_CHECK, &_ones, 1);

	FLASH_Write(FLASH_WRITE_LAST_EEPROM_PAGE_DATA_ADDR, &_ones, 1);
	FLASH_Write(FLASH_WRITE_LAST_EEPROM_PAGE_DATA_CHECK, &_ones, 1);
}

enum UART_cmdOptions { none, setTime, rstData };

bool getUartCmd(UART_cmdOptions *_cmdOption) {
	char password[UART_PASS_LEN] = "";
	char cmd[UART_CMD_LEN] = "";
	int8_t parseResult = sscanf(UartRxBuff, "%15s %7s", &password, &cmd);
	if (!parseResult == 2) {
		return false;
	}
	if (strcmp(password, UART_pass)) {
		return false;
	}
	if (!strcmp(cmd, UART_setTimeCmd)) {
		*_cmdOption = setTime;
	} else if (!strcmp(cmd, UART_rstDataCmd)) {
		*_cmdOption = rstData;
	} else {
		return false;
	}
}

bool executeCmd() {
	UART_cmdOptions UART_currentCmd = none;
	if (!getUartCmd(&UART_currentCmd)) {
		return false;
	}
	switch (UART_currentCmd) {
		case setTime: {
			int currentHour = 0;
			int currentMinute = 0;
			int currentDay = 0;
			int currentMonth = 0;
			int currentYear = 0;
			UartRxBuff[strlen(UartRxBuff) - 2] = 0;
			int8_t parseResult
				= sscanf(UartRxBuff + UART_PASS_LEN + UART_CMD_LEN + 2, "%2d %2d %2d %2d %2d", &currentHour, &currentMinute, &currentDay, &currentMonth,
						 &currentYear); // shift start of sscanf after password, cmd and 2 spaces
			if (parseResult != 5) {
				return false;
			}
			set_time(currentHour, currentMinute, defaultSeconds, defaultWeekday, currentMonth, currentDay, currentYear);
			break;
		}
		case rstData: {
			resetFlashData();
			break;
		}
		default:
			break;
	}
}
/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
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
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
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
	/**Initialize RTC and set the Time and Date
	 */

	if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != 0x32F2) {
		if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
			char *RTC_failedSetTimeMsg = "Failed to set time to RTC from backup register";
			HAL_UART_Transmit(&huart1, (uint8_t *)RTC_failedSetTimeMsg, strlen(RTC_failedSetTimeMsg), 1000);
			HAL_GPIO_WritePin(ERROR_GPIO_Port, ERROR_Pin, GPIO_PIN_SET);
			while (1) {
			}
		}

		if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
			char *RTC_failedSetDataMsg = "Failed to set data to RTC from backup register";
			HAL_UART_Transmit(&huart1, (uint8_t *)RTC_failedSetDataMsg, strlen(RTC_failedSetDataMsg), 1000);
			HAL_GPIO_WritePin(ERROR_GPIO_Port, ERROR_Pin, GPIO_PIN_SET);
			while (1) {
			}
		}

		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2);
	}
	/* USER CODE END Check_RTC_BKUP */
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
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
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

	/*Configure GPIO pins : LORA_RST_Pin LORA_NSS_Pin */
	GPIO_InitStruct.Pin = LORA_RST_Pin | LORA_NSS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : LORA_DIO0_Pin */
	GPIO_InitStruct.Pin = LORA_DIO0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(LORA_DIO0_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : ET_INTn_Pin */
	GPIO_InitStruct.Pin = ET_INTn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(ET_INTn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : ET_SCSn_Pin ET_RSTn_Pin */
	GPIO_InitStruct.Pin = ET_SCSn_Pin | ET_RSTn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : LORA_TX_Pin ERROR_Pin STM_READY_Pin */
	GPIO_InitStruct.Pin = LORA_TX_Pin | ERROR_Pin | STM_READY_Pin;
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
