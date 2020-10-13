#include "myRTC.h"
#include "stm32f1xx_hal_def.h"
#if 1

#define RTC_BKP_DR1_FLAG 0x5051
extern RTC_HandleTypeDef hrtc;
extern _calendar_obj calendar;
//?????											 
const uint8_t table_week[12]={0,3,3,6,1,4,6,2,5,0,3,5}; //??????	 
//????????
const uint8_t mon_table[12]={31,28,31,30,31,30,31,31,30,31,30,31};
#define RTC_ALARM_RESETVALUE             0xFFFFFFFFU
extern UART_HandleTypeDef huart1;
extern HAL_StatusTypeDef  RTC_WriteTimeCounter(RTC_HandleTypeDef *hrtc, uint32_t TimeCounter);
extern uint32_t           RTC_ReadAlarmCounter(RTC_HandleTypeDef *hrtc);
extern HAL_StatusTypeDef  RTC_WriteAlarmCounter(RTC_HandleTypeDef *hrtc, uint32_t AlarmCounter);
extern uint32_t           RTC_ReadTimeCounter(RTC_HandleTypeDef *hrtc);
void rtc_init_user(void)
{	
	if(HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR1)!=RTC_BKP_DR1_FLAG)
	{
		HAL_UART_Transmit(&huart1, (uint8_t *)"RTC bkp reg set!\r\n", 18, 100);
		if(HAL_OK != RTC_Set(&hrtc, 2019,4,7,19,29,0))
		{
			HAL_UART_Transmit(&huart1, (uint8_t *)"RTC_Set error\r\n", 15, 100);
		}
		HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR1,RTC_BKP_DR1_FLAG);
	}

	RTC_Get();
}
uint32_t RTC_get_count(uint16_t syear,uint8_t smon,uint8_t sday,uint8_t hour,uint8_t min,uint8_t sec)
{
	uint16_t t;
	uint32_t seccount=0;
	
	if(syear<1970||syear>2099)	return 0; 
	for(t=1970;t<syear;t++)	//??????????
	{
		if(Is_Leap_Year(t))seccount+=31622400;//??????
		else seccount+=31536000;			  //??????
	}
	smon-=1;
	for(t=0;t<smon;t++)	   //???????????
	{
		seccount+=(uint32_t)mon_table[t]*86400;//???????
		if(Is_Leap_Year(syear)&&t==1)seccount+=86400;//??2??????????	   
	}
	seccount+=(uint32_t)(sday-1)*86400;//??????????? 
	seccount+=(uint32_t)hour*3600;//?????
    seccount+=(uint32_t)min*60;	 //?????
	seccount+=sec;//????????	
	
	return seccount;
}

uint32_t RTC_get_count_from_date(uint16_t syear,uint8_t smon,uint8_t sday)
{
	uint16_t t;
	uint32_t seccount=0;
	
	if(syear<1970||syear>2099)	return 0; 
	for(t=1970;t<syear;t++)	//??????????
	{
		if(Is_Leap_Year(t))seccount+=31622400;//??????
		else seccount+=31536000;			  //??????
	}
	smon-=1;
	for(t=0;t<smon;t++)	   //???????????
	{
		seccount+=(uint32_t)mon_table[t]*86400;//???????
		if(Is_Leap_Year(syear)&&t==1)seccount+=86400;//??2??????????	   
	}
	seccount+=(uint32_t)(sday-1)*86400;//??????????? 
	
	return seccount;
}


uint32_t RTC_get_count_from_date_time(uint16_t syear,uint8_t smon,uint8_t sday, uint8_t hour, uint8_t minute, uint8_t second)
{
	uint16_t t;
	uint32_t seccount=0;
	
	seccount = RTC_get_count_from_date(syear, smon, sday);
	seccount +=hour*3600+minute*60+second;
	return seccount;
}

HAL_StatusTypeDef RTC_Set(RTC_HandleTypeDef *hrtc, uint16_t syear,uint8_t smon,uint8_t sday,uint8_t hour,uint8_t min,uint8_t sec)
{
	uint16_t t;
	uint32_t seccount=0U;
	if(NULL == hrtc)
	{
		return HAL_ERROR;
	}
	assert_param(IS_RTC_HOUR24(sTime->Hours));
	assert_param(IS_RTC_MINUTES(sTime->Minutes));
	assert_param(IS_RTC_SECONDS(sTime->Seconds));
	
	//counter_time = hour*3600 + min*60 + sec;
	seccount = RTC_get_count(syear, smon, sday, hour, min, sec);
	if(0 == seccount)
	{
		HAL_UART_Transmit(&huart1, (uint8_t *)"0\r\n", 3, 100);
		return HAL_ERROR;
	}
 
	#if 0
	RCC->APB1ENR|=1<<28;//??????
	RCC->APB1ENR|=1<<27;//??????
	PWR->CR|=1<<8;    //????????
	//????????!
	RTC->CRL|=1<<4;   //???? 
	RTC->CNTL=seccount&0xffff;
	RTC->CNTH=seccount>>16;
	RTC->CRL&=~(1<<4);//????
	while(!(RTC->CRL&(1<<5)));//??RTC??????? 
	#else
	__HAL_LOCK(hrtc);

  hrtc->State = HAL_RTC_STATE_BUSY;
	/* Write time counter in RTC registers */
  if (RTC_WriteTimeCounter(hrtc, seccount) != HAL_OK)
  {
    /* Set RTC state */
    hrtc->State = HAL_RTC_STATE_ERROR;

    /* Process Unlocked */
    __HAL_UNLOCK(hrtc);
		HAL_UART_Transmit(&huart1, (uint8_t *)"1\r\n", 3, 100);
    return HAL_ERROR;
  }
  else
  {
    /* Clear Second and overflow flags */
    CLEAR_BIT(hrtc->Instance->CRL, (RTC_FLAG_SEC | RTC_FLAG_OW));

    /* Read current Alarm counter in RTC registers */
    uint32_t counter_alarm = RTC_ReadAlarmCounter(hrtc);

    /* Set again alarm to match with new time if enabled */
    if (counter_alarm != RTC_ALARM_RESETVALUE)
    {
      if (counter_alarm < seccount)
      {
        /* Add 1 day to alarm counter*/
        counter_alarm += (uint32_t)(24U * 3600U);

        /* Write new Alarm counter in RTC registers */
        if (RTC_WriteAlarmCounter(hrtc, counter_alarm) != HAL_OK)
        {
          /* Set RTC state */
          hrtc->State = HAL_RTC_STATE_ERROR;

          /* Process Unlocked */
          __HAL_UNLOCK(hrtc);
					HAL_UART_Transmit(&huart1, (uint8_t *)"2\r\n", 3, 100);
          return HAL_ERROR;
        }
      }
    }

    hrtc->State = HAL_RTC_STATE_READY;

    __HAL_UNLOCK(hrtc);

  }
	#endif

	RTC_Get();//??????????? 	
	return HAL_OK;
}
 
HAL_StatusTypeDef RTC_Get(void)
{
	static uint16_t daycnt=0;
	uint32_t timecount=0; 
	uint32_t temp=0;
	uint16_t temp1=0;	
	
#if 0  
 	timecount=RTC->CNTH;//????????(???)
	timecount<<=16;
	timecount+=RTC->CNTL;			
#else
	timecount = RTC_ReadTimeCounter(&hrtc);
#endif	

 	temp=timecount/86400;   //????(??????)
	
	if(daycnt!=temp)//?????
	{	  
		daycnt=temp;
		temp1=1970;	//?1970???
		while(temp>=365)
		{				 
			if(Is_Leap_Year(temp1))//???
			{
				if(temp>=366)temp-=366;//??????
				else break;  
			}
			else temp-=365;	  //?? 
			temp1++;  
		} 
		
		calendar.w_year=temp1;//????
		temp1=0;
		while(temp>=28)//??????
		{
			if(Is_Leap_Year(calendar.w_year)&&temp1==1)//???????/2??
			{
				if(temp>=29)temp-=29;//??????
				else break; 
			}
			else 
			{
				if(temp>=mon_table[temp1])temp-=mon_table[temp1];//??
				else break;
			}
			temp1++;  
		}
		calendar.w_month=temp1+1;	//????
		calendar.w_date=temp+1;  	//???? 
	}
	temp=timecount%86400;     		//?????   	   
	calendar.hour=temp/3600;     	//??
	calendar.min=(temp%3600)/60; 	//??	
	calendar.sec=(temp%3600)%60; 	//??
	calendar.week=RTC_Get_Week(calendar.w_year,calendar.w_month,calendar.w_date);//???? 

	#if 0
	/* Read Alarm counter in RTC registers */
	uint32_t counter_alarm = RTC_ReadAlarmCounter(&hrtc);

	uint32_t counter_time = calendar.hour*3600 + calendar.min*60 + calendar.sec;
	/* Calculate remaining time to reach alarm (only if set and not yet expired)*/
	if ((counter_alarm != RTC_ALARM_RESETVALUE) && (counter_alarm > counter_time))
	{
		counter_alarm -= counter_time;
	}
	else
	{
		/* In case of counter_alarm < counter_time */
		/* Alarm expiration already occurred but alarm not deactivated */
		counter_alarm = RTC_ALARM_RESETVALUE;
	}

	 /* Set updated alarm to be set */
	if (counter_alarm != RTC_ALARM_RESETVALUE)
	{
		counter_alarm += counter_time;

		/* Write time counter in RTC registers */
		if (RTC_WriteAlarmCounter(&hrtc, counter_alarm) != HAL_OK)
		{
			return HAL_ERROR;
		}
	}
	else
	{
		/* Alarm already occurred. Set it to reset values to avoid unexpected expiration */
		if (RTC_WriteAlarmCounter(&hrtc, counter_alarm) != HAL_OK)
		{
			return HAL_ERROR;
		}
	}
	#endif
	
}	
 
HAL_StatusTypeDef RTC_Get_date_time_from_count(uint32_t timecount, _calendar_obj *calen)
{
	//static uint16_t daycnt=0;
	uint32_t temp=0;
	uint16_t temp1=0;	
	
 	temp=timecount/86400;   //????(??????)
	
	//if(daycnt!=temp)//?????
	{	  
		//daycnt=temp;
		temp1=1970;	//?1970???
		while(temp>=365)
		{				 
			if(Is_Leap_Year(temp1))//???
			{
				if(temp>=366)temp-=366;//??????
				else break;  
			}
			else temp-=365;	  //?? 
			temp1++;  
		} 
		
		calen->w_year=temp1;//????
		temp1=0;
		while(temp>=28)//??????
		{
			if(Is_Leap_Year(calen->w_year)&&temp1==1)//???????/2??
			{
				if(temp>=29)temp-=29;//??????
				else break; 
			}
			else 
			{
				if(temp>=mon_table[temp1])temp-=mon_table[temp1];//??
				else break;
			}
			temp1++;  
		}
		calen->w_month=temp1+1;	//????
		calen->w_date=temp+1;  	//???? 
	}
	temp=timecount%86400;     		//?????   	   
	calen->hour=temp/3600;     	//??
	calen->min=(temp%3600)/60; 	//??	
	calen->sec=(temp%3600)%60; 	//??
	calen->week=RTC_Get_Week(calen->w_year,calen->w_month,calen->w_date);//???? 
}

void RTC_Get_time_from_count(uint32_t timecount, RTC_TimeTypeDef *timeType)
{
	if(NULL == timeType)
	{
		return;
	}
	uint32_t temp=0;
	
	temp=timecount%86400;     		//?????   	   
	timeType->Hours=temp/3600;     	//??
	timeType->Minutes=(temp%3600)/60; 	//??	
	timeType->Seconds=(temp%3600)%60; 	//??
}

//?????????
//??   1  2  3  4  5  6  7  8  9  10 11 12
//??   31 29 31 30 31 30 31 31 30 31 30 31
//??? 31 28 31 30 31 30 31 31 30 31 30 31
//year:??
//???:????????.1,?.0,??
uint8_t Is_Leap_Year(uint16_t year)
{			  
	if(year%4==0) //????4??
	{ 
		if(year%100==0) 
		{ 
			if(year%400==0)return 1;//???00??,????400?? 	   
			else return 0;   
		}else return 1;   
	}else return 0;	
}	
 
//????????
//????:??????????(???1901-2099?)
//year,month,day:????? 
//???:???																						 
uint8_t RTC_Get_Week(uint16_t year,uint8_t month,uint8_t day)
{	
	uint16_t temp2;
	uint8_t yearH,yearL;
	
	yearH=year/100;	yearL=year%100; 
	// ???21??,????100  
	if (yearH>19)yearL+=100;
	// ???????1900????  
	temp2=yearL+yearL/4;
	temp2=temp2%7; 
	temp2=temp2+day+table_week[month-1];
	if (yearL%4==0&&month<3)temp2--;
	return(temp2%7);
}
#define RTC_ALARM_RESETVALUE             0xFFFFFFFFU
HAL_StatusTypeDef RTC_SetDateTime(RTC_HandleTypeDef *hrtc, _calendar_obj *calen)
{
  uint32_t counter_time = 0U, counter_alarm = 0U;
	uint16_t year = 0;
	uint8_t month,day,hour,minute,second;
  /* Check input parameters */
  if ((hrtc == NULL) || (calen == NULL))
  {
    return HAL_ERROR;
  }
	return RTC_Set(hrtc, calen->w_year, calen->w_month, calen->w_date, 
												calen->hour, calen->min, calen->sec);
}

#if 0
HAL_StatusTypeDef MY_RTC_GetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, uint32_t Format)
{
  uint32_t counter_time = 0U, counter_alarm = 0U, days_elapsed = 0U, hours = 0U;

  /* Check input parameters */
  if ((hrtc == NULL) || (sTime == NULL))
  {
    return HAL_ERROR;
  }

  /* Check the parameters */
  assert_param(IS_RTC_FORMAT(Format));

  /* Check if counter overflow occurred */
  if (__HAL_RTC_OVERFLOW_GET_FLAG(hrtc, RTC_FLAG_OW))
  {
    return HAL_ERROR;
  }

  /* Read the time counter*/
  counter_time = RTC_ReadTimeCounter(hrtc);

  /* Fill the structure fields with the read parameters */
  hours = counter_time / 3600U;
  sTime->Minutes  = (uint8_t)((counter_time % 3600U) / 60U);
  sTime->Seconds  = (uint8_t)((counter_time % 3600U) % 60U);

  if (hours >= 24U)
  {
    /* Get number of days elapsed from last calculation */
    days_elapsed = (hours / 24U);

    /* Set Hours in RTC_TimeTypeDef structure*/
    sTime->Hours = (hours % 24U);

    /* Read Alarm counter in RTC registers */
    counter_alarm = RTC_ReadAlarmCounter(hrtc);

    /* Calculate remaining time to reach alarm (only if set and not yet expired)*/
    if ((counter_alarm != RTC_ALARM_RESETVALUE) && (counter_alarm > counter_time))
    {
      counter_alarm -= counter_time;
    }
    else
    {
      /* In case of counter_alarm < counter_time */
      /* Alarm expiration already occurred but alarm not deactivated */
      counter_alarm = RTC_ALARM_RESETVALUE;
    }

    /* Set updated time in decreasing counter by number of days elapsed */
    counter_time -= (days_elapsed * 24U * 3600U);

    /* Write time counter in RTC registers */
    if (RTC_WriteTimeCounter(hrtc, counter_time) != HAL_OK)
    {
      return HAL_ERROR;
    }

    /* Set updated alarm to be set */
    if (counter_alarm != RTC_ALARM_RESETVALUE)
    {
      counter_alarm += counter_time;

      /* Write time counter in RTC registers */
      if (RTC_WriteAlarmCounter(hrtc, counter_alarm) != HAL_OK)
      {
        return HAL_ERROR;
      }
    }
    else
    {
      /* Alarm already occurred. Set it to reset values to avoid unexpected expiration */
      if (RTC_WriteAlarmCounter(hrtc, counter_alarm) != HAL_OK)
      {
        return HAL_ERROR;
      }
    }

    /* Update date */
    RTC_DateUpdate(hrtc, days_elapsed);
  }
  else
  {
    sTime->Hours = hours;
  }

  /* Check the input parameters format */
  if (Format != RTC_FORMAT_BIN)
  {
    /* Convert the time structure parameters to BCD format */
    sTime->Hours    = (uint8_t)RTC_ByteToBcd2(sTime->Hours);
    sTime->Minutes  = (uint8_t)RTC_ByteToBcd2(sTime->Minutes);
    sTime->Seconds  = (uint8_t)RTC_ByteToBcd2(sTime->Seconds);
  }

  return HAL_OK;
}
#endif

#endif
