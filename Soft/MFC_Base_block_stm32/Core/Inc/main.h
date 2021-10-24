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

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

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
void getMeasTablePageFromEEPROM(char *htmlTable, uint16_t pageNum);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
