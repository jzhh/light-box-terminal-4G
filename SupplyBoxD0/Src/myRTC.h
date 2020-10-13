#ifndef __MYRTC_H
#define __MYRTC_H

#include "stm32f1xx_hal.h"

typedef struct 
{
	__IO uint8_t hour;
	__IO uint8_t min;
	__IO uint8_t sec;			
	
	__IO uint16_t w_year;
	__IO uint8_t  w_month;
	__IO uint8_t  w_date;
	__IO uint8_t  week;	
}_calendar_obj;	


void rtc_init_user(void);
HAL_StatusTypeDef RTC_Set(RTC_HandleTypeDef *hrtc, uint16_t syear,uint8_t smon,uint8_t sday,uint8_t hour,uint8_t min,uint8_t sec);
HAL_StatusTypeDef RTC_Get(void);
uint8_t Is_Leap_Year(uint16_t year);
uint8_t RTC_Get_Week(uint16_t year,uint8_t month,uint8_t day);
HAL_StatusTypeDef RTC_SetDateTime(RTC_HandleTypeDef *hrtc, _calendar_obj *calen);
uint32_t RTC_get_count_from_date(uint16_t syear,uint8_t smon,uint8_t sday);
uint32_t RTC_get_count(uint16_t syear,uint8_t smon,uint8_t sday,uint8_t hour,uint8_t min,uint8_t sec);
HAL_StatusTypeDef RTC_Get_date_time_from_count(uint32_t timecount, _calendar_obj *calen);
void RTC_Get_time_from_count(uint32_t timecount, RTC_TimeTypeDef *timeType);
uint32_t RTC_get_count_from_date_time(uint16_t syear,uint8_t smon,uint8_t sday, uint8_t hour, uint8_t minute, uint8_t second);
#endif
