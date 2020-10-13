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
typedef struct
{ 
	uint8_t AtCmd[256]; 
	uint16_t CmdLen;
} 
CmdStruct;

enum NBSTATUS
{
	NBSTART=1,AT,DISSLP,CPIN,IPADD,CESQ,CFUN,QLWSERV,CEREG,NCDP,QLWCONF,
	QLWADDOBJ0,QLWADDOBJ1,QLWOPEN,QLWCFG,QLWDATASEND,CONNECTOK,GETIMEI,GETTIME,ENDATT
};

typedef struct
{ 
	uint8_t  Alarm;//alarm State bit:1=ERROR  0=OK
	uint8_t  SwitchState[8];//Switch1~8 state to be set (1=ON  0=OFF)
	uint8_t  EnableAutoSwitch;//Enable/Disable auto switch control
	uint16_t SwitchOnTime[8];
	uint16_t SwitchOFFTime[8];
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
} 
CurrentState;


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
#define L4_RLY_Pin GPIO_PIN_0
#define L4_RLY_GPIO_Port GPIOC
#define L5_RLY_Pin GPIO_PIN_1
#define L5_RLY_GPIO_Port GPIOC
#define L6_RLY_Pin GPIO_PIN_2
#define L6_RLY_GPIO_Port GPIOC
#define L1_RLY_Pin GPIO_PIN_3
#define L1_RLY_GPIO_Port GPIOC
#define NB_POWEREN_Pin GPIO_PIN_1
#define NB_POWEREN_GPIO_Port GPIOA
#define NB_RESETB_Pin GPIO_PIN_4
#define NB_RESETB_GPIO_Port GPIOA
#define NB_PSM_EINT_Pin GPIO_PIN_6
#define NB_PSM_EINT_GPIO_Port GPIOA
#define NB_PWRKEY_Pin GPIO_PIN_7
#define NB_PWRKEY_GPIO_Port GPIOA
#define L2_RLY_Pin GPIO_PIN_4
#define L2_RLY_GPIO_Port GPIOC
#define L3_RLY_Pin GPIO_PIN_5
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
#define NUMOFCMD 10
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
