/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
	
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define DEVICE_BOARD_CONFIG_ID		0xA5A87A6A 			//config flag
#define VERSION "1.0.0_20200702"

#define MAX_SWITCH_COUNT 8

#define TIMx                           TIM3
#define TIMx_CLK_ENABLE()              __HAL_RCC_TIM3_CLK_ENABLE()


/* Definition for TIMx's NVIC */
#define TIMx_IRQn                      TIM3_IRQn
#define TIMx_IRQHandler                TIM3_IRQHandler

#define TIMx_Period            1000-1
#define TIMx_Prescaler         71

typedef struct
{ 
	uint8_t AtCmd[256]; 
	uint16_t CmdLen;
} 
CmdStruct;

enum ATSTATUS
{
	START=1,AT,DISSLP,CPIN,IPADD,CSQ,CREG,QLWSERV,CEREG,CGREG,NCDP,QLWCONF,
	QLWADDOBJ0,QLWADDOBJ1,QLWOPEN,QLWCFG,QLWDATASEND,CONNECTOK,GETIMEI,GETTIME,ENDATT,ATE,QLWDEL,QLWOBSERVE,
	QBAND,CPSMS,CGATT,QCGDEFCONT,LIFETIME
};


typedef struct
{ 
	uint8_t  Alarm;//alarm State bit:1=ERROR  0=OK
	uint8_t  SwitchState[MAX_SWITCH_COUNT];//Switch1~8 state to be set (1=ON  0=OFF)
	uint8_t  EnableAutoSwitch;//Enable/Disable auto switch control
	uint8_t  SwitchOnTimeHh[MAX_SWITCH_COUNT];//ON Time:hour
	uint8_t  SwitchOnTimeMm[MAX_SWITCH_COUNT];//ON Time:minute
	uint8_t  SwitchOFFTimeHh[MAX_SWITCH_COUNT];//OFF Time:hour
	uint8_t  SwitchOFFTimeMm[MAX_SWITCH_COUNT];//OFF Time:minute
	uint32_t quantity;//The Meter of WattMeter
	uint16_t Current;
	uint16_t Power;
	uint16_t Temperature;
	uint16_t LastPower;
	uint16_t LastTemperature;	
	uint16_t ReportCondPeriod;
	uint16_t ReportCondPower;
	uint16_t ReportCondTemp;
	uint8_t  Phase;
	uint32_t Gapstart;
	uint32_t ReportGapstart;
	uint32_t ReportGap;
	uint16_t  SwitchLineCurrent[MAX_SWITCH_COUNT];//互感器电流
	uint16_t  SwitchLineVoltage[MAX_SWITCH_COUNT];//线电压
} 
CurrentState;

enum AUTO_SWITCH
{
	AUTO_SWITCH_CLOSE=0,AUTO_SWITCH_OPEN
};

enum AUTO_SWITCH_MODE
{
	SUNRISE_CONTROL=1,TIMMING_CONTROL
};
	
#pragma pack(1)
typedef struct
{
    uint16_t  period;
    uint16_t  powerDelta;
    uint16_t  tempDelta;
    
}REPORT_CONDITION;

typedef struct{
	uint8_t  autoSwitch;//自动开关,第1位代表第1路自动开关使能状态，1表示使能，0表示不使能
	uint8_t  autoSwitchMode[MAX_SWITCH_COUNT];//自动开关模式：日出日落或固定时间开关
	uint8_t  sunRiseDelayCloseMinute[MAX_SWITCH_COUNT];//日出日落控制模式下，日出时间延时再关闭,单位：分钟
	uint8_t  sunSetEarlyOpenMinute[MAX_SWITCH_COUNT];//日出日落控制模式下，日落时间提前打开
	uint8_t  SwitchOnTimeHh[MAX_SWITCH_COUNT];//ON Time:hour
	uint8_t  SwitchOnTimeMm[MAX_SWITCH_COUNT];//ON Time:minute
	uint8_t  SwitchOFFTimeHh[MAX_SWITCH_COUNT];//OFF Time:hour
	uint8_t  SwitchOFFTimeMm[MAX_SWITCH_COUNT];//OFF Time:minute
}SWITCH_POLICY;

typedef struct{
	float longitude;
	float latitude;
}LOCATION;

typedef struct
{
	uint32_t ID;
	char version[16];
	LOCATION location;
  REPORT_CONDITION reportConfig;
	SWITCH_POLICY switchPolicy;
}DEVICE_CONFIG;
#pragma pack()

typedef struct 
{
	uint8_t timeInterval_1_IsExecuted;
	uint8_t timeInterval_2_IsExecuted;
	uint32_t second;
}Switch_Policy;

typedef struct 
{
	Switch_Policy openPolicy[8];
	Switch_Policy closePolicy[8];
}Timming_Switch_Policy_Second;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define L4_RLY_Pin GPIO_PIN_5
#define L4_RLY_GPIO_Port GPIOC
#define L5_RLY_Pin GPIO_PIN_4
#define L5_RLY_GPIO_Port GPIOC
#define L6_RLY_Pin GPIO_PIN_3
#define L6_RLY_GPIO_Port GPIOC
#define L1_RLY_Pin GPIO_PIN_2
#define L1_RLY_GPIO_Port GPIOC
#define NB_POWEREN_Pin GPIO_PIN_1
#define NB_POWEREN_GPIO_Port GPIOA
#define NB_RESETB_Pin GPIO_PIN_4
#define NB_RESETB_GPIO_Port GPIOA
#define NB_PSM_EINT_Pin GPIO_PIN_6
#define NB_PSM_EINT_GPIO_Port GPIOA
#define NB_PWRKEY_Pin GPIO_PIN_7
#define NB_PWRKEY_GPIO_Port GPIOA
#define L2_RLY_Pin GPIO_PIN_1
#define L2_RLY_GPIO_Port GPIOC
#define L3_RLY_Pin GPIO_PIN_0
#define L3_RLY_GPIO_Port GPIOC
#define K6_Pin GPIO_PIN_7
#define K6_GPIO_Port GPIOC
#define K5_Pin GPIO_PIN_8
#define K5_GPIO_Port GPIOC
#define K4_Pin GPIO_PIN_9
#define K4_GPIO_Port GPIOC
#define K3_Pin GPIO_PIN_8
#define K3_GPIO_Port GPIOA
#define K1_Pin GPIO_PIN_11
#define K1_GPIO_Port GPIOA
#define K2_Pin GPIO_PIN_12
#define K2_GPIO_Port GPIOA
#define ALARM_Pin GPIO_PIN_3
#define ALARM_GPIO_Port GPIOB
#define L1_INDICATION_Pin GPIO_PIN_4
#define L1_INDICATION_GPIO_Port GPIOB
#define L2_INDICATION_Pin GPIO_PIN_5
#define L2_INDICATION_GPIO_Port GPIOB
#define L3_INDICATION_Pin GPIO_PIN_6
#define L3_INDICATION_GPIO_Port GPIOB
#define L4_INDICATION_Pin GPIO_PIN_7
#define L4_INDICATION_GPIO_Port GPIOB
#define L5_INDICATION_Pin GPIO_PIN_8
#define L5_INDICATION_GPIO_Port GPIOB
#define L6_INDICATION_Pin GPIO_PIN_9
#define L6_INDICATION_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
//Line Switch
#define L1 L1_RLY_Pin
#define L1_PORT L1_RLY_GPIO_Port
#define L2 L2_RLY_Pin
#define L2_PORT L2_RLY_GPIO_Port
#define L3 L3_RLY_Pin
#define L3_PORT L3_RLY_GPIO_Port
#define L4 L4_RLY_Pin
#define L4_PORT L4_RLY_GPIO_Port
#define L5 L5_RLY_Pin
#define L5_PORT L5_RLY_GPIO_Port
#define L6 L6_RLY_Pin
#define L6_PORT L6_RLY_GPIO_Port
//LED
#define LED1 L1_INDICATION_Pin
#define LED1_PORT L1_INDICATION_GPIO_Port
#define LED2 L2_INDICATION_Pin
#define LED2_PORT L2_INDICATION_GPIO_Port
#define LED3 L3_INDICATION_Pin
#define LED3_PORT L3_INDICATION_GPIO_Port
#define LED4 L4_INDICATION_Pin
#define LED4_PORT L4_INDICATION_GPIO_Port
#define LED5 L5_INDICATION_Pin
#define LED5_PORT L5_INDICATION_GPIO_Port
#define LED6 L6_INDICATION_Pin
#define LED6_PORT L6_INDICATION_GPIO_Port
#define ALARM ALARM_Pin
#define ALARM_PORT ALARM_GPIO_Port
//Switch state
#define K1 1
#define K1_PORT K1_GPIO_Port
#define K2 2
#define K2_PORT K2_GPIO_Port
#define K3 3
#define K3_PORT K3_GPIO_Port
#define K4 4
#define K4_PORT K4_GPIO_Port
#define K5 5
#define K5_PORT K5_GPIO_Port
#define K6 6
#define K6_PORT K6_GPIO_Port

#define NOFOUND  -1
#define NUMOFCMD 14
#define ATCOMMAND 5
#define NUMOFMSKEY 4
#define FLASHPAGESIZE 1024
#define PAGEBLANK 1
#define PAGENOBLANK 0
#define PARAGETED 1
#define NOPARAGETED 0
#define BLOCKBLANK 1
#define BLOCKNOBLANK 0
#define FLASHBLCOKSIZE 8
#define TA_FAIL 1
#define TA_SUCCESS 0
#define NUMBITOFIMEI 15
#define TIMEGAP 5000

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
