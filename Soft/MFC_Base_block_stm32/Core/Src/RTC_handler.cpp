#include "RTC_handler.h"
#include "time.h"

static uint16_t defaultYear = 21;
static uint8_t defaultMonth = 10;
static uint16_t defaultDate = 30;
static uint8_t defaultWeekday = 1;

static uint8_t defaultHours = 12;
static uint8_t defaultMinutes = 0;
static uint8_t defaultSeconds = 0;

RTC_DateTimeTypeDef startDateTime;

void GetLocalTime(time_t unixTimestamp, tm *outBuffer) {
	time(&unixTimestamp);
	outBuffer = localtime(&unixTimestamp);
}

// Get current date
void RTC_GetDateTime(RTC_DateTimeTypeDef *RTC_DateTime) {
	uint32_t RTC_Counter = RTC_ReadTimeCounter(&hrtc);
	struct tm *timeinfo;
	time_t tempRTC_Counter = (time_t)RTC_Counter;
	timeinfo = localtime(&tempRTC_Counter);

	RTC_DateTime->Date = timeinfo->tm_mday;
	RTC_DateTime->Month = timeinfo->tm_mon + 1;
	RTC_DateTime->Year = timeinfo->tm_year + 1900;
	RTC_DateTime->Hours = timeinfo->tm_hour;
	RTC_DateTime->Minutes = timeinfo->tm_min;
	RTC_DateTime->Seconds = timeinfo->tm_sec;
}

// Convert Date to Counter
uint32_t RTC_GetRTC_Counter(RTC_DateTimeTypeDef RTC_DateTime) {
	struct tm timeinfo;
	timeinfo.tm_mday = RTC_DateTime.Date;
	timeinfo.tm_mon = RTC_DateTime.Month - 1;
	timeinfo.tm_year = RTC_DateTime.Year - 1900;
	timeinfo.tm_hour = RTC_DateTime.Hours;
	timeinfo.tm_min = RTC_DateTime.Minutes;
	timeinfo.tm_sec = RTC_DateTime.Seconds;

	timeinfo.tm_isdst = 0;
	timeinfo.tm_wday = 0;
	timeinfo.tm_yday = 0;
	return mktime(&timeinfo);
}

bool RTC_SetTime(uint8_t curHours, uint8_t curMinutes, uint8_t curDate, uint8_t curMonth, uint16_t curYear) {

	startDateTime.Date = curDate;
	startDateTime.Month = curMonth;
	startDateTime.Year = curYear;

	startDateTime.Hours = curHours;
	startDateTime.Minutes = curMinutes;
	startDateTime.Seconds = defaultSeconds;

	//После инициализации требуется задержка. Без нее время не устанавливается.
	HAL_Delay(500);

	if (RTC_WriteTimeCounter(&hrtc, RTC_GetRTC_Counter(startDateTime)) != HAL_OK) {
		return false;
	}
	return true;
}