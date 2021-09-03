/*
	A library for Grove -

	Copyright (c) 2018 seeed technology co., ltd.
	Author      : Wayen Weng
	Create Time : November 2018
	Change Log  :

	The MIT License (MIT)

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/

#include "SCD30.h"

SCD30::SCD30(void) {
	devAddr = SCD30_I2C_ADDRESS << 1;
}

void SCD30::initialize(I2C_HandleTypeDef _hi2c) {
	hi2c = _hi2c;
	writeCommandWithArguments(SCD30_SET_MEASUREMENT_INTERVAL, 2);	// 2 seconds between measurements
	writeCommandWithArguments(SCD30_CONTINUOUS_MEASUREMENT, 0x0000);
}

bool SCD30::isAvailable(void) {
	return readRegister(SCD30_GET_DATA_READY);
}

void SCD30::getCarbonDioxideConcentration(float *result) {
	uint8_t buf[18] = {0};
	uint32_t co2U32 = 0;
	uint32_t tempU32 = 0;
	uint32_t humU32 = 0;
	float co2Concentration = 0;
	float temperature = 0;
	float humidity = 0;

	writeCommand(SCD30_READ_MEASUREMENT);
	readBuffer(buf, 18);

	co2U32 = (uint32_t)((((uint32_t)buf[0]) << 24) | (((uint32_t)buf[1]) << 16) | (((uint32_t)buf[3]) << 8) | ((uint32_t)buf[4]));

	tempU32 = (uint32_t)((((uint32_t)buf[6]) << 24) | (((uint32_t)buf[7]) << 16) | (((uint32_t)buf[9]) << 8) | ((uint32_t)buf[10]));

	humU32 = (uint32_t)((((uint32_t)buf[12]) << 24) | (((uint32_t)buf[13]) << 16) | (((uint32_t)buf[15]) << 8) | ((uint32_t)buf[16]));

	memcpy(&result[0], &co2U32, sizeof(co2Concentration));
	memcpy(&result[1], &tempU32, sizeof(temperature));
	memcpy(&result[2], &humU32, sizeof(humidity));
}

void SCD30::writeCommand(uint16_t command) {

	uint8_t buf[2];
	buf[0] = command >> 8;
	buf[1] = command & 0xff;
	HAL_I2C_Master_Transmit(&hi2c, devAddr, buf, sizeof(buf), 1000);
}

void SCD30::writeCommandWithArguments(uint16_t command, uint16_t arguments) {
	uint8_t checkSum, buf[5] = {0};

	buf[0] = command >> 8;
	buf[1] = command & 0xff;
	buf[2] = arguments >> 8;
	buf[3] = arguments & 0xff;
	checkSum = calculateCrc(&buf[2], 2);
	buf[4] = checkSum;

	writeBuffer(buf, 5);
}

uint16_t SCD30::readRegister(uint16_t address) {
	uint8_t buf[2] = {0};

	writeCommand(address);
	readBuffer(buf, 2);

	return ((((uint16_t)buf[0]) << 8) | buf[1]);
}

void SCD30::writeBuffer(uint8_t *data, uint8_t len) {
	HAL_I2C_Master_Transmit(&hi2c, devAddr, data, len, 1000);
}

void SCD30::readBuffer(uint8_t *data, uint8_t len) {
	HAL_I2C_Master_Receive(&hi2c, devAddr, data, len, 1000);
}

uint8_t SCD30::calculateCrc(uint8_t data[], uint8_t len) {
	uint8_t bit, byteCtr, crc = 0xff;

	// calculates 8-Bit checksum with given polynomial
	for (byteCtr = 0; byteCtr < len; byteCtr++) {
		crc ^= (data[byteCtr]);

		for (bit = 8; bit > 0; --bit) {
			if (crc & 0x80) {
				crc = (crc << 1) ^ SCD30_POLYNOMIAL;
			} else {
				crc = (crc << 1);
			}
		}
	}

	return crc;
}

SCD30 scd30;
