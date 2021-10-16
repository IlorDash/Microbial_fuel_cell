#include "extEEPROM.h"

//*********************************
#include <stdio.h>
#include <string.h>					  //libs for EEPROM test func
#include <stdlib.h> /* srand, rand */ //comment if not using test
#include <time.h>
//*********************************

bool extEEPROM_read(uint16_t pageNum, uint16_t offset, uint8_t *dest, uint8_t size, uint32_t numAttempts) {

	uint8_t i2c_addres = EEPROM_ADDR;

	if (pageNum >= BLOCK_PAGE_NUM) {
		i2c_addres |= 2;
	}

	uint16_t memAddr = pageNum << PAGE_ADDR_POS | offset;

	HAL_I2C_Mem_Read(EEPROM_I2C, i2c_addres, memAddr, I2C_MEMADD_SIZE_16BIT, dest, size, HAL_MAX_DELAY);

	HAL_StatusTypeDef status = HAL_BUSY;
	while ((status != HAL_OK) && numAttempts) {
		numAttempts--;
		status = HAL_I2C_IsDeviceReady(EEPROM_I2C, i2c_addres, 1, HAL_MAX_DELAY);
	}
	return (status == HAL_OK);
}

bool extEEPROM_writePage(uint16_t pageNum, uint8_t *src, uint32_t numAttempts) {

	uint8_t i2c_addres = EEPROM_ADDR;

	if (pageNum >= BLOCK_PAGE_NUM) {
		i2c_addres |= 2;
	}

	uint16_t memAddr = pageNum << PAGE_ADDR_POS;

	HAL_I2C_Mem_Write(EEPROM_I2C, i2c_addres, memAddr, I2C_MEMADD_SIZE_16BIT, src, PAGE_SIZE, HAL_MAX_DELAY);

	HAL_StatusTypeDef status = HAL_BUSY;
	while ((status != HAL_OK) && numAttempts) {
		numAttempts--;
		status = HAL_I2C_IsDeviceReady(EEPROM_I2C, i2c_addres, 1, HAL_MAX_DELAY);
	}
	return (status == HAL_OK);
}

void testEEPROM(UART_HandleTypeDef huart) {
	char EEPROM_txBuff[128 + 1] = "";
	char EEPROM_rxBuff[128 + 1] = "";

	uint16_t iter = 0;
	srand(time(NULL));
	while (iter < PAGE_NUM) {
		for (int i = 0; i < 128; i++) {
			EEPROM_txBuff[i] = i + (rand() % 128);
		}
		bool writeResult = extEEPROM_writePage(iter, (uint8_t *)EEPROM_txBuff, 1000);
		if (!writeResult) {
			char txBuff[32] = "";
			sprintf(txBuff, "Failed to write %d page", iter);
			HAL_UART_Transmit(&huart, (uint8_t *)txBuff, strlen(txBuff), 1000);
			return;
		}
		bool readResult = extEEPROM_read(iter, 0, (uint8_t *)EEPROM_rxBuff, 128, 1000);
		if (!readResult) {
			char txBuff[32] = "";
			sprintf(txBuff, "Failed to read %d page", iter);
			HAL_UART_Transmit(&huart, (uint8_t *)txBuff, strlen(txBuff), 1000);
			return;
		}
		bool cmpResult = strcmp(EEPROM_txBuff, EEPROM_rxBuff);
		if (cmpResult) {
			char txBuff[32] = "";
			sprintf(txBuff, "Wrote and read buffers isn't equal in %d page", iter);
			HAL_UART_Transmit(&huart, (uint8_t *)txBuff, strlen(txBuff), 1000);
			return;
		}
		memset(EEPROM_rxBuff, 0, strlen(EEPROM_rxBuff));
		iter++;
	}
}