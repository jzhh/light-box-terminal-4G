#ifndef __SUNRISESET_H
#define __SUNRISESET_H

#include "myRTC.h"
#include "Math.h"

uint8_t getSunrise(double glong, double glat, _calendar_obj calendar, RTC_TimeTypeDef *sunRiseTime);
uint8_t getSunset(double glong, double glat, _calendar_obj calendar, RTC_TimeTypeDef *sunSetTime);

#endif
