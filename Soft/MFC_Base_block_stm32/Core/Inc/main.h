/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define MEAS_TX_BUFF_LENGTH 20
#define DATA_FRAME_LENGTH 30
#define DATA_FRAME_PER_PAGE 4

#define MEAS_VARS_NUM 4
#define DATA_FRAME_VARS_NUM 8
#define TABLE_PAGE_MEAS_NUM 15

#define HTML_TABLE_MEAS_ROW_LEN 104

#define FLASH_WRITE_DATA_FRAME_NUM_CHECK 0x800F000
#define FLASH_WRITE_DATA_FRAME_NUM_ADDR 0x800F400

#define FLASH_WRITE_LAST_EEPROM_PAGE_DATA_CHECK 0x800F800
#define FLASH_WRITE_LAST_EEPROM_PAGE_DATA_ADDR 0x800FC00

#define FLASH_WORD_LENGTH 4

#define UART_PASS_LEN 15
#define UART_CMD_LEN 7
#define UART_TX_BUFF_LEN 64
#define UART_RX_BUFF_LEN 64

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
struct EEPROM_struct {
	uint32_t curDataFrameNum;
	uint32_t curPageNum;

	char txBuff[128 + 1]; // size is equal to page size, because writing to EEPROM by one page
	char rxBuff[DATA_FRAME_LENGTH + 1]; // size is equal to data frame size
};
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LORA_RST_Pin GPIO_PIN_3
#define LORA_RST_GPIO_Port GPIOA
#define LORA_NSS_Pin GPIO_PIN_4
#define LORA_NSS_GPIO_Port GPIOA
#define LORA_SCK_Pin GPIO_PIN_5
#define LORA_SCK_GPIO_Port GPIOA
#define LORA_MISO_Pin GPIO_PIN_6
#define LORA_MISO_GPIO_Port GPIOA
#define LORA_MOSI_Pin GPIO_PIN_7
#define LORA_MOSI_GPIO_Port GPIOA
#define LORA_DIO0_Pin GPIO_PIN_0
#define LORA_DIO0_GPIO_Port GPIOB
#define ET_INTn_Pin GPIO_PIN_12
#define ET_INTn_GPIO_Port GPIOB
#define ET_SCK_Pin GPIO_PIN_13
#define ET_SCK_GPIO_Port GPIOB
#define ET_MISO_Pin GPIO_PIN_14
#define ET_MISO_GPIO_Port GPIOB
#define ET_MOSI_Pin GPIO_PIN_15
#define ET_MOSI_GPIO_Port GPIOB
#define ET_SCSn_Pin GPIO_PIN_8
#define ET_SCSn_GPIO_Port GPIOA
#define ET_RSTn_Pin GPIO_PIN_11
#define ET_RSTn_GPIO_Port GPIOA
#define LORA_TX_Pin GPIO_PIN_3
#define LORA_TX_GPIO_Port GPIOB
#define ERROR_Pin GPIO_PIN_4
#define ERROR_GPIO_Port GPIOB
#define STM_READY_Pin GPIO_PIN_5
#define STM_READY_GPIO_Port GPIOB
#define EEPROM_SCL_Pin GPIO_PIN_6
#define EEPROM_SCL_GPIO_Port GPIOB
#define EEPROM_SDA_Pin GPIO_PIN_7
#define EEPROM_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
void getMeasTableRowFromEEPROM(char *htmlTable, uint16_t startDataFrame);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
