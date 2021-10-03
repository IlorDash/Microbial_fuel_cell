#include "extEEPROM.h"

I2C_HandleTypeDef *hi2c;

void extEEPROM_init(I2C_HandleTypeDef *_hi2c) {
	hi2c = _hi2c;
	HAL_StatusTypeDef status;
	while (status != HAL_OK) {
		status = HAL_I2C_IsDeviceReady(hi2c, I2C_BASE_ADR, 1, HAL_MAX_DELAY);
	}
}

void extEEPROM_read(uint16_t memAddr, char *dest, uint8_t size, uint32_t timeout) {
	HAL_I2C_Mem_Read(hi2c, I2C_BASE_ADR, memAddr + MEM_ADR_OFFSET, I2C_MEMADD_SIZE_16BIT, (uint8_t *)dest, size, timeout);

	HAL_StatusTypeDef status;
	while (status != HAL_OK) {
		status = HAL_I2C_IsDeviceReady(hi2c, I2C_BASE_ADR, 1, timeout);
	}
}

void extEEPROM_write(uint16_t memAddr, char *src, uint8_t size, uint32_t timeout) {
	HAL_I2C_Mem_Write(hi2c, I2C_BASE_ADR, memAddr + MEM_ADR_OFFSET, I2C_MEMADD_SIZE_16BIT, (uint8_t *)src, size, timeout);

	HAL_StatusTypeDef status;
	while (status != HAL_OK) {
		status = HAL_I2C_IsDeviceReady(hi2c, I2C_BASE_ADR, 1, timeout);
	}
}