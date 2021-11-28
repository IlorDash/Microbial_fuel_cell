#pragma once

#include "main.h"

typedef struct {
	uint8_t Hours;
	uint8_t Minutes;
	uint8_t Seconds;
	uint8_t Date;
	uint8_t Wday;
	uint8_t Month;
	uint16_t Year;
} RTC_DateTimeTypeDef;

extern RTC_HandleTypeDef hrtc;
extern RTC_DateTimeTypeDef startDateTime;

extern HAL_StatusTypeDef RTC_WriteTimeCounter(RTC_HandleTypeDef *hrtc, uint32_t TimeCounter);
extern uint32_t RTC_ReadTimeCounter(RTC_HandleTypeDef *hrtc);

void RTC_GetDateTime(RTC_DateTimeTypeDef *RTC_DateTimeStruct);
bool RTC_SetTime(uint8_t curHours, uint8_t curMinutes, uint8_t curDate, uint8_t curMonth, uint16_t curYear);