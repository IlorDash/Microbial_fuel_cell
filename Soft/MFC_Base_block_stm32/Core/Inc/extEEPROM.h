#pragma once

/*
Library for reading and writing external EEPROM via I2C. Choose EEPROM I2C addres according to A0, A1, A2 pins.
Base adress is 0b1010000, where last 3 bits is setting by A0, A1, A2 pins: if one pin is tied to VDD then it's bit is set to 1 and vice-versa.
Based on https://github.com/afiskon/stm32-external-eeprom example
*/

#include <main.h>

#define I2C_BASE_ADR (0x50 << 1) // HAL expects address to be shifted one bit to the left
#define MEM_ADR_OFFSET 100

void extEEPROM_init(I2C_HandleTypeDef *_hi2c);
void extEEPROM_read(uint16_t memAddr, char *dest, uint8_t size, uint32_t timeout);
void extEEPROM_write(uint16_t memAddr, char *src, uint8_t size, uint32_t timeout);