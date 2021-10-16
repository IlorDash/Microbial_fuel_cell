#pragma once

/*
Library for reading and writing external EEPROM 24LC1026 via I2C. Choose EEPROM I2C addres according to A0, A1, A2 pins.
Base adress is 0b1010000, where last 3 bits is setting by A1, A2 pins: if one pin is tied to VDD then it's bit is set to 1 and vice-versa.
Based on https://github.com/afiskon/stm32-external-eeprom example
*/

#include <main.h>

extern I2C_HandleTypeDef hi2c1;
#define EEPROM_I2C &hi2c1

// EEPROM ADDRESS (8bits)
#define EEPROM_ADDR (0x50 << 1) // HAL expects address to be shifted one bit to the left

// Define the Page Size and number of pages
#define PAGE_SIZE 128	   // in Bytes
#define BLOCK_PAGE_NUM 512 // number of pages in one memory block. B0 bit in CONTROL byte sets first or second memory segment of 64KB
#define PAGE_NUM 1024	   // number of pages

#define PAGE_ADDR_POS 7 // number of bit that defines where the page addressing starts = log(PAGE_SIZE)/log(2)

#define DEFAULT_NUM_ATTEMPTS 1000

bool extEEPROM_read(uint16_t pageNum, uint16_t offset, uint8_t *dest, uint8_t size, uint32_t numAttempts);
bool extEEPROM_writePage(uint16_t pageNum, uint8_t *src, uint32_t numAttempts); // writes 128 bytes in page

void testEEPROM(UART_HandleTypeDef huart);