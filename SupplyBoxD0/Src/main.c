/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

#if 1
/* Includes ------------------------------------------------------------------*/
#include "main.h" 
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "key.h"
#include "myRTC.h"
#include "sunRiseSet.h"
#include "cmsis_armcc.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define TXBUFFERSIZE                  (COUNTOF(aTxBuffer) - 1)
#define xTBUFFERSIZE                  (COUNTOF(xTBuffer) - 1)

//const uint8_t ret_ok = 0;
//const uint8_t ret_err = 1;

extern uint32_t           RTC_ReadTimeCounter(RTC_HandleTypeDef *hrtc);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

_calendar_obj calendar;
/* USER CODE BEGIN PV */
//const uint8_t aTxBuffer[] = " **** The Smart Street Lamp SwitchBox_Controller ****\r";
//const uint8_t xTBuffer[]  = " **************Guilin XX inc 2020.03.16**************\r";
const uint8_t HardWareVer[]  = "HardWare Version D.0\r";
const uint8_t SoftWareVer[]  = "SoftWare Version D.0 Test\r";
static const char *const CmdString[] =
{/* Command KeyWord Table */
  "SWITCHON+",  /* SWITCHON+ */
  "SWITCHOFF+", /* SWITCHOFF+ */
  "NBON",       /* NBON */
  "NBOFF",      /* NBOFF */
  "NBRESET",    /* NBRESET */
  "AT",         /* AT */
	"VER",        /* Read Software/hardware Version */
	"TIME",       /* Read RN8209C register*/
	"PUBLISH",       /* Write RN8209C register*/	
	"GETDATA",     /* Get data */
	"FLASH",	/*print config*/
	"NOW",		/*print current time*/
	"REREG",		/*NB reregiste to network*/
	"COUNTER="		/*set RTC counter*/
};
static const char CmdSize[]={9,10,4,5,7,2,3,4,7,7,5,3,5,8};

static const char ATQSCLK[]="AT+QSCLK=0";
static const char ATCESQ[]="AT+CESQ";
static const char ATCREG[]="AT+CREG?";
static const char ATQLWSERV[]="AT+QLWSERV=\"221.229.214.202\",5683";
static const char ATGSN[]="AT+GSN";
static const char ATQLWCONF[]="AT+QLWCONF=\"861050043011100\"";
static const char ATQLWADDOBJ0[]="AT+QLWADDOBJ=19,0,1,\"0\"";
static const char ATQLWADDOBJ1[]="AT+QLWADDOBJ=19,1,1,\"0\"";
static const char ATQLWCFG[]="AT+QLWCFG=\"dataformat\",0,1";
static const char ATQLWOPEN[]="AT+QLWOPEN=0";
static const char ATCEREG[]="AT+CEREG?";
static const char ATCGREG[]="AT+CGREG?";
static const char NBSTA[] ="start...\n";
static const char WIATIPADD[] ="Wait for IP address...\n";
static const char ATCCLK[] ="AT+CCLK?";
static const char ATECLOSE[]="ATE0";
//static const char ATCGPADDR[] = "AT+CGPADDR=1";
//static const char ATQLWDEL[] = "AT+QLWDEL";
static const char ATQBAND[]="AT+QBAND=1,5";
static const char ATCSQ[]="AT+CSQ";
static const char ATCGATT[]="AT+CGATT?";
static const char ATCPIN[]="AT+CPIN?";

char SUB_TOPIC_FORMAT[] = "\"device/4G/%s/down\"";
char sub_topic[48] = {0};
char deviceId[18] = {0};
//static const char ATQCGDEFCONT[]="AT+QCGDEFCONT=\"IP\",\"PSM0.edrx0.ctnb\"";
//static const char ATQLWUPDATE[]="AT+QLWUPDATE";
//static const char ATLIFETIME[]="AT+QLWCFG=\"lifetime\",900";

static const char InCMD[]="\nInvalid Command!\n";
static const char NBPowerOff[]="NB power off!\n";
static const char NBPowerOn[]="NB power on!\n";
//static const char PUSHKEY[]="\r\n PUSH POWER KEY....";

/***************************NB message define*************************/
//Platform Command(not all)
//const char PlatformReadCMD[] = {0x05,0x01,0xC1,0x00,0xB8,0x00};
//const char PLSetCondCMD[] =          {0x0C,0x01,0xC2,0xD2,0x00,0x0E,0x10,0x00,0x05,0x00,0x1E,0x75,0xFD};/*3600S,5W,3.0*/
//const char PLSetCondReplyDataCMD[] = {0x0C,0x01,0xC2,0xD2,0x01,0x0E,0x10,0x00,0x05,0x00,0x1E,0x75,0xFD};/*3600S,5W,3.0*/
//const char SwitchCtrlCMD[] = 
//{ 0x27,0x01,0xC3,0xD2,0x00,0x01 
//  0x18,0x00,0x07,0x00,0x18,0x00,0x07,0x00,0x18,0x00,0x07,0x00,0x18,0x00,0x07,0x00
//  0x18,0x00,0x07,0x00,0x18,0x00,0x07,0x00,0x18,0x00,0x07,0x00,0x18,0x00,0x07,0x00
//  0x60,0xAE
//}
//const char SwitchCtrlReplyDataCMD[] = 
//{ 0x27,0x01,0xC3,0xD2,0x01,0x01
//  0x18,0x00,0x07,0x00,0x18,0x00,0x07,0x00,0x18,0x00,0x07,0x00,0x18,0x00,0x07,0x00
//  0x18,0x00,0x07,0x00,0x18,0x00,0x07,0x00,0x18,0x00,0x07,0x00,0x18,0x00,0x07,0x00
//  0xF5,0x6D
//}

//const char SwitchOnOffCMD[] = 
//{0x0E,0x01,0xC4,0xD2,0x00,0x010,x01,0x01,0x01,0x01,0x01,0x01,0x01,0xA9,0x5F}
//

//reply message(Typical)
#if 0
static char SwitchCtrlParaMsg[]=   
{//Switch controller return
  0x18,0x01,0xC1,0xD2,0x00,/*Length,SN,CMD,TYPE,ALARM*/
	0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,/*CH1~CH8*/
	0x00,0x00,0x15,0x32,/*The READ NUMBER OF ENERGY METER*/
	0x00,0x02,/*CURRENT:mA*/
	0x04,0x00,/*POWER:W*/
	0x01,0x2C,/*Temperature:0.1*/
	0xEE,0x0A/*CRC*/
};
#endif

static char paramAndTimmingSwitchTableMsg[]=   
{//Switch controller return
  0x67,0x01,0xC1,0xD2,0x00,/*Length,SN,CMD,TYPE,ALARM*/
	0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,/*CH1~CH8*/
	0x00,0x00,0x15,0x32,/*The READ NUMBER OF ENERGY METER*/
	0x00,0x00,/*CURRENT:mA*/
	0x00,0x00,/*POWER:W*/
	0x00,0x00,/*Temperature:0.1*/
	0x00,0x00,/*priod*/
	0x00,0x00,/*power delta*/
	0x00,0x00,/*temp delta*/
	0x00,/*timming switch enable*/
	0x00,0x00,0x00,0x00,/*CH1 switch timming*/
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,/*CH8 switch timming*/
	0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,//switch control mode
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//line current
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//line current
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//line voltage
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//line voltage
	0x00,0x00/*CRC*/
};

static char CondWrOKMsg[] = {0x07,0x01,0xC2,0xD2,0x00,0x00,0xED,0xA1};
static char CondWrErrMsg[]= {0x07,0x01,0xC2,0xD2,0x00,0xE0,0x65,0xA0};
static char CondWrWithDataOKMsg[] = 
{
  0x0D,0x01,0xC2,0xD2,0x01,0x00,/*Length,SN,CMD,TYPE,FLAG,RESULT*/
  0x0E,0x10,/*PERIOD:S*/
  0x00,0x05,/*POWER:W*/
  0x00,0x1E,/*Temperature:0.1*/
  0x56,0x3C/*CRC*/
};
#if 0
static char CondWrWithDataErrMsg[]= 
{
  0x0D,0x01,0xC2,0xD2,0x01,0xE0,/*Length,SN,CMD,TYPE,FLAG,RESULT*/
  0x0E,0x10,/*PERIOD:S*/
  0x00,0x05,/*POWER:W*/
  0x00,0x1E,/*Temperature:0.1*/
  0x98,0xDD/*CRC*/
};
#endif
static char ReadCondMsg[]= 
{
  0x0B,0x01,0xC6,0xD2,/*Length,SN,CMD,TYPE*/
  0x0E,0x10,/*PERIOD:S*/
  0x00,0x05,/*POWER:W*/
  0x00,0x1E,/*Temperature:0.1*/
  0x98,0xDD/*CRC*/
};

//static char SCWrOKMsg[]  = {0x07,0x01,0xC3,0xD2,0x00,0x00,0x11,0xA0};
static char NoDataResp[]  = {0x07,0x01,0xC3,0xD2,0x00,0x00,0x11,0xA0};
static char SCWrErrMsg[] = {0x07,0x01,0xC3,0xD2,0x00,0xE0,0x99,0xA1};
static char SetPolicyResp[]=
{
	0x30,0x01,0xC3,0xD2,0x01,0x00,0x00,/*Length,SN,CMD,TYPE,FLAG,RESULT,EN*/
	0x18,0x00,0x07,0x00,/*1 18:00=ON 7:00=OFF  Time:BCD Code*/
	0x18,0x00,0x07,0x00,/*2 18:00=ON 7:00=OFF*/
	0x18,0x00,0x07,0x00,/*3 18:00=ON 7:00=OFF*/
	0x18,0x00,0x07,0x00,/*4 18:00=ON 7:00=OFF*/
	0x18,0x00,0x07,0x00,/*5 18:00=ON 7:00=OFF*/
	0x18,0x00,0x07,0x00,/*6 18:00=ON 7:00=OFF*/
  0x18,0x00,0x07,0x00,/*7 18:00=ON 7:00=OFF*/
	0x18,0x00,0x07,0x00,/*8 18:00=ON 7:00=OFF*/  
	0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,//switch control mode
	0xD1,0x2E           /*CRC*/
};
static char SCWrWithDataErrMsg[]=
{
	0x28,0x01,0xC3,0xD2,0x01,0xE0,0x00,/*Length,SN,CMD,TYPE,FLAG,RESULT,EN*/
	0x18,0x00,0x07,0x00,/*1 18:00=ON 7:00=OFF  Time:BCD Code*/
	0x18,0x00,0x07,0x00,/*2 18:00=ON 7:00=OFF*/
	0x18,0x00,0x07,0x00,/*3 18:00=ON 7:00=OFF*/
	0x18,0x00,0x07,0x00,/*4 18:00=ON 7:00=OFF*/
	0x18,0x00,0x07,0x00,/*5 18:00=ON 7:00=OFF*/
	0x18,0x00,0x07,0x00,/*6 18:00=ON 7:00=OFF*/
	0x18,0x00,0x07,0x00,/*7 18:00=ON 7:00=OFF*/
	0x18,0x00,0x07,0x00,/*8 18:00=ON 7:00=OFF*/  
	0xB1,0x0E/*CRC*/
};

static char readTimmingSwitchMsg[]=
{
	0x26,0x01,0xC7,0xD2,0x00,/*Length,SN,CMD,TYPE,EN*/
	0x18,0x00,0x07,0x00,/*1 18:00=ON 7:00=OFF  Time:BCD Code*/
	0x18,0x00,0x07,0x00,/*2 18:00=ON 7:00=OFF*/
	0x18,0x00,0x07,0x00,/*3 18:00=ON 7:00=OFF*/
	0x18,0x00,0x07,0x00,/*4 18:00=ON 7:00=OFF*/
	0x18,0x00,0x07,0x00,/*5 18:00=ON 7:00=OFF*/
	0x18,0x00,0x07,0x00,/*6 18:00=ON 7:00=OFF*/
	0x18,0x00,0x07,0x00,/*7 18:00=ON 7:00=OFF*/
	0x18,0x00,0x07,0x00,/*8 18:00=ON 7:00=OFF*/  
	0xB1,0x0E/*CRC*/
};

static char SWConWrOKMsg[]  = {0x07,0x01,0xC4,0xD2,0x00,0x00,0x65,0xA1};
static char SWConWrErrMsg[] = {0x07,0x01,0xC4,0xD2,0x00,0xE0,0xED,0xA0};
static char SWConWrWithDataOKMsg[] = 
{
  0x2F,0x01,0xC4,0xD2,0x01,0x00,/*Length,SN,CMD,TYPE,FLAG,RESULT*/
	0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,/*CH1~CH8*/
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//line current
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//line current
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//line voltage
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//line voltage
	0xCE,0xDB/*CRC*/
};
static char SWConWrWithDataErrMsg[] = 
{
  0x0F,0x01,0xC4,0xD2,0x01,0xE0,/*Length,SN,CMD,TYPE,FLAG,RESULT*/
	0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,/*CH1~CH8*/
	0x0A,0x12/*CRC*/
};

static char readQuantityCmd[] = 
{
	//0x01,0x03,0x00,0x0e,0x00,0x02,0xa5,0xc8 //L1 voltage
	0x01,0x03,0x01,0x00,0x00,0x02,0xC5,0xF7 
};

static char readSwitchLineCurrentCmd[] = 
{
	0x01,0x03,0x00,0x40,0x00,0x2A,0x00,0x00 //读取42个寄存器值
};

static char reportControlSwitchMsg[] = 
{
  0x0D,0x00,0xC8,0xD2,/*Length,SN,CMD,TYPE*/
	0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,/*CH1~CH8*/
	0xCE,0xDB/*CRC*/
};

static char commonRespMsg[] = 
{
  0x07,0x01,0x00,0xD2,/*Length,SN,CMD,TYPE*/
	0X00,//not need reply data
	0x00,//result
	0x00,0x00/*CRC*/
};

static char testData[] = 
{
  0x07,0x01,0x00,0x00,/*Length,SN,CMD,TYPE*/
	0X00,//not need reply data
	0x00,//result
	0x00,0x00/*CRC*/
};

static CmdStruct NBMsg,ConMsg,ReStr;
static CmdStruct AttRetMsg,AttOpMsg;//,RespMsg;
static CurrentState CtrlData;
static DEVICE_CONFIG deviceConfig;
static DEVICE_CONFIG defaultDeviceConfig = 
{
	DEVICE_BOARD_CONFIG_ID,//ID
	VERSION,
	//location
	{
		110.2966,
		25.275118
	},
	//report condition
	{
		480,//period
		0,//power delta
		0 //temp delta
	},
	{
		0xFF,//auto switch 自动开关功能全部使能
		SUNRISE_CONTROL,SUNRISE_CONTROL,SUNRISE_CONTROL,SUNRISE_CONTROL,
		SUNRISE_CONTROL,SUNRISE_CONTROL,SUNRISE_CONTROL,SUNRISE_CONTROL,//auto switch mode
		0,0,0,0,0,0,0,0,//日出延时关闭时间
		0,0,0,0,0,0,0,0,//日落延时打开时间
		19,19,19,19,19,19,19,19,//open hour
		0,0,0,0,0,0,0,0,//open minute
		6,6,6,6,6,6,6,6, //close hour
		0,0,0,0,0,0,0,0 //close minute
	}
};

uint32_t last_sysc_time_second = 0;
#define SYSC_TIME_GAP_SECOND 24*3600//单位：秒

uint32_t last_rcv_policy_tick = 0;
//uint32_t last_reset_tick = 0;

static Timming_Switch_Policy_Second TimmingSwitchPolicySecond;

//static uint32_t Gapstart = 0U;
//static uint32_t ReportGapstart = 0U;
//static uint32_t ReportGap = 0U;
//static uint8_t Phase;
/* USER CODE END PV */
#if 1
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI2_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
static void MIC29302AWU_ON(void);
static void MIC29302AWU_OFF(void);
static void NB_ON(void);
static void NB_OFF(void);
static void LineSwitch_ON(uint16_t LINE);
static void LineSwitch_OFF(uint16_t LINE);
static void BoxLED_ON(uint16_t LED);
static void BoxLED_OFF(uint16_t LED);
void AlarmLED(CurrentState *Ctrl);
void LineLED(uint16_t LED);
GPIO_PinState BoxGetSwitchState(uint16_t Swt);
static void NB_PSMEINT(void);
static void NB_PsmEintHIGH(void);
static void NB_PsmEintLOW(void);
static void NB_RESETB(void);
static void NB_PKEY(void);
static void NB_StartUp(void);
HAL_StatusTypeDef GetDBGCmd(UART_HandleTypeDef *huart,CmdStruct *ConCmd);
HAL_StatusTypeDef GetNBMsg(UART_HandleTypeDef *huart,CmdStruct *NBMsg);
int8_t LookUpCmd(uint8_t* pBuffer,uint16_t Cmdlength);
void NB_ATCMD(UART_HandleTypeDef *DBGhuart,UART_HandleTypeDef *NBhuart,CmdStruct *TxCmd,CmdStruct *RetString,uint32_t Timeout);//AT Command
void NBMsgUp(UART_HandleTypeDef *DBGhuart,UART_HandleTypeDef *NBhuart,char *NBMsgString,uint32_t Timeout, uint8_t confirm);
void StringCpy(uint8_t* String1,uint8_t* String2,uint16_t Stringlength);
unsigned char DecNum(unsigned char ch);
uint8_t NB_ACK_Check(char *ackstruct,uint8_t len,char *str);
uint8_t NB_Attachment(UART_HandleTypeDef *DBGhuart,UART_HandleTypeDef *NBhuart, uint8_t attachTimes);
uint8_t NB_CommCheck(UART_HandleTypeDef *NBhuart);
uint8_t NB_ACK_Check(char *ackstruct,uint8_t len,char *str);
uint8_t GetNBMsgAndCheck(UART_HandleTypeDef *NBhuart,CmdStruct *NBMsgPt,char *KeyWord,uint32_t Timeout);
uint8_t TurnOnPowerLineSwitch(uint8_t LineNumber);
uint8_t TurnOFFPowerLineSwitch(uint8_t LineNumber);
uint8_t CheckSwitch(uint8_t LineNumber,uint8_t StateToBeSet);
void GetCurrentData(CurrentState* Ctrl,char *ChangedMsg);
void GetAlarmState(CurrentState* Ctrl);
uint16_t GetCurrentMeter(void);
uint16_t GetPowerMeter(void);
uint32_t GetMeterOFWattMeter(void);
uint16_t GetTemperature(ADC_HandleTypeDef*hadc);
void DisplayStringHex(UART_HandleTypeDef* USARTx,char *StringPoint,uint8_t length);
uint8_t StringToHex(unsigned char *StrPoint,uint8_t *Data,uint8_t length);
uint8_t HexToString(uint8_t *HexData,unsigned char *ChString,uint8_t length);
static uint16_t chkcrc(char *arr,char fxstat,char len);
unsigned char HexNum(unsigned char ch);
unsigned char HexToAsc(unsigned char num);
int32_t Myabs(int32_t num);
void MultiEventProcess(CurrentState* Ctrl,char *replyMsg);
void LEDEventProc(CurrentState* Ctrl);
void SwitchConEventProc(CurrentState* Ctrl);
static void RTC_TimeShow(uint8_t* showtime);
unsigned char IsDec(unsigned char ch);
void NBPowerRestart(void);
void NB_ATCMD_NO_PRINT(UART_HandleTypeDef *DBGhuart,UART_HandleTypeDef *NBhuart,CmdStruct *TxCmd,CmdStruct *RetString,uint32_t Timeout);
void sysc_time_from_network(void);
void turnOnAllLineSwitch(void);
void turnOffAllLineSwitch(void);
void MX_TIM6_Init(void);
void MX_TIMx_Init(void);
void sendPeriodReportData(void);
void GetSwitchLineCurrent(CurrentState* Ctrl);
void SoftReset(void);
int MQTT_Init(void);
int MQTT_publish(uint8_t *hexData, uint16_t len);

#endif
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
//#include "stm32f10x_flash.h"

uint16_t FLASH_ReadHalfWord(uint32_t address);

#define CONFIG_PAGE 100
#define FLASH_SIZE 256          
#define CONFIG_BUF_SIZE 256

#if FLASH_SIZE<256
  #define SECTOR_SIZE           1024   
#else 
  #define SECTOR_SIZE           2048    
#endif

#define false 0
#define true 1
	
#define SWITCH_NUM 6

#endif 

static int IS_FIRST_REPORT = true;
static int IS_FIRST_SYSC_TIME = true;

#if 1
union float2hex{
    uint32_t hexData;
    float floatData;
};

///float 2 hex
uint32_t f2h(float data) {
    union float2hex v;
    v.floatData = data;
    return v.hexData;
}

void printTimeCounter(uint8_t num)
{
	#if 1
	uint32_t timecount;
	char tt[32] = {0};
	timecount=RTC->CNTH;//????????(???)
	timecount<<=16;
	timecount+=RTC->CNTL;			 
	
	sprintf(tt, "%u timecount %10u\r\n", num, timecount);
	HAL_UART_Transmit(&huart1, (uint8_t *)tt, strlen(tt), 100);
	#endif
	RTC_Get();
	char str[32] = {0};
	sprintf(str, "%04d/%02d/%02d %02d:%02d:%02d\r\n",calendar.w_year, calendar.w_month, calendar.w_date,
																					calendar.hour, calendar.min, calendar.sec);
	HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),200);
}

void printAlarmCount(int num)
{
	uint32_t counter_alarm = RTC_ReadAlarmCounter(&hrtc);
	char tt[32] = {0};		 
	sprintf(tt, "%u alarmcount %10u\r\n", num, counter_alarm);
	HAL_UART_Transmit(&huart1, (uint8_t *)tt, strlen(tt), 100);
}

uint32_t DEVICE_CONFIG_START_ADDRESS=(uint32_t)FLASH_BASE + CONFIG_PAGE*SECTOR_SIZE;

#define CHECK_NETWORK_PERIOD 300
static uint32_t last_check_network_tick = 0;
static uint8_t CT_PLATFORM_CONNECT_FLAG = 0;
//static uint8_t NEED_TO_SYSC_TIME = 0;
static uint32_t network_first_unavailable_tick = 0;

#define PKEY_Pin GPIO_PIN_6
#define PKEY_GPIO_Port GPIOC
#define PKEY PKEY_Pin
#define PKEY_PORT PKEY_GPIO_Port
#define KEYDELAY 2000

//static uint32_t PKEYtickstart = 0U;
//static char PkeyStatus = 0;
//static char nextSwitchActionFlag = 0;

static uint8_t NBSendFailetimes = 0;
volatile uint32_t time = 0; // ms 计时变量 

#define SEND_FAILED_TIMES_TO_REREGIST 2

TIM_HandleTypeDef    TimHandle;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
#endif


#if 1
void FLASH_ReadMoreData(uint32_t startAddress,uint16_t *readData,uint16_t countToRead)
{
  uint16_t dataIndex;
  for(dataIndex=0;dataIndex<countToRead;dataIndex++)
  {
    readData[dataIndex]=FLASH_ReadHalfWord(startAddress+dataIndex*2);
  }
}

uint16_t FLASH_ReadHalfWord(uint32_t address)
{
  return *(__IO uint16_t*)address; 
}

uint32_t FLASH_ReadWord(uint32_t address)
{
  uint32_t temp1,temp2;
  temp1=*(__IO uint16_t*)address; 
  temp2=*(__IO uint16_t*)(address+2); 
  return (temp2<<16)+temp1;
}

int FLASH_Erase(uint32_t startAddress, uint32_t NbPages)
{
	int re = -1;
	FLASH_EraseInitTypeDef EraseInitStruct;		
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = startAddress; 
	EraseInitStruct.NbPages = NbPages;
	uint32_t PageError = 0; 
	
	HAL_FLASH_Unlock();
  if(HAL_OK == HAL_FLASHEx_Erase(&EraseInitStruct, &PageError))
	{
		re = 0;
	}
	HAL_FLASH_Lock();
	return re;
}
int FLASH_WriteMoreData(uint32_t startAddress,uint16_t *writeData,uint16_t countToWrite)
{
	int re = -1;
  if(startAddress<FLASH_BASE||((startAddress+countToWrite*2)>=(FLASH_BASE+SECTOR_SIZE*FLASH_SIZE)))
  {
    return re;
  }
	
	if(0 != FLASH_Erase(startAddress, 1))
	{
		goto END;
	}
	
	HAL_FLASH_Unlock();
  uint16_t dataIndex = 0;
  for(dataIndex=0;dataIndex<countToWrite;dataIndex++)
  {
		if(HAL_OK != HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, startAddress+dataIndex*2,writeData[dataIndex]))
		{
			re = -1;
			break;
		}
		
		re = 0;
  }
	
	END:
		HAL_FLASH_Lock();
	
	return re;
}


int writeDeviceConfig(DEVICE_CONFIG *config)
{
	uint8_t buf[CONFIG_BUF_SIZE] = {0};
	memcpy(buf, config, sizeof(DEVICE_CONFIG));
	return FLASH_WriteMoreData(DEVICE_CONFIG_START_ADDRESS, (uint16_t *)&buf, sizeof(buf)/2);
}

void readDeviceConfig(DEVICE_CONFIG *deviceConfig)
{
	uint8_t buf[CONFIG_BUF_SIZE] = {0};
	FLASH_ReadMoreData(DEVICE_CONFIG_START_ADDRESS, (uint16_t *)buf, sizeof(buf)/2);
	memcpy(deviceConfig, buf, sizeof(DEVICE_CONFIG));
}

void initDeviceConfigFromFlash(DEVICE_CONFIG *deviceConfig)
{
	char error[] = "write default config error\r\n";
	readDeviceConfig(deviceConfig);
	if(DEVICE_BOARD_CONFIG_ID != deviceConfig->ID)
	{
		if(-1 == writeDeviceConfig(&defaultDeviceConfig))
		{
			HAL_UART_Transmit(&huart1,(uint8_t*)error,strlen(error),100);
			FLASH_Erase(DEVICE_CONFIG_START_ADDRESS, 1);
		}
		memcpy((void *)deviceConfig, (void *)&defaultDeviceConfig, sizeof(DEVICE_CONFIG));
	}
}
#endif


#if 1
void setTimmingSwitchPolicySecond(DEVICE_CONFIG deviceConfig)
{
	for(int i = 0; i < MAX_SWITCH_COUNT; i++)
	{
		if(TIMMING_CONTROL == deviceConfig.switchPolicy.autoSwitchMode[i])
		{
			TimmingSwitchPolicySecond.openPolicy[i].timeInterval_1_IsExecuted = false;
			TimmingSwitchPolicySecond.openPolicy[i].timeInterval_2_IsExecuted = false;
			TimmingSwitchPolicySecond.openPolicy[i].second = deviceConfig.switchPolicy.SwitchOnTimeHh[i]*3600 +
																									deviceConfig.switchPolicy.SwitchOnTimeMm[i]*60;		
			TimmingSwitchPolicySecond.closePolicy[i].timeInterval_1_IsExecuted = false;
			TimmingSwitchPolicySecond.closePolicy[i].timeInterval_2_IsExecuted = false;
			TimmingSwitchPolicySecond.closePolicy[i].second = 	deviceConfig.switchPolicy.SwitchOFFTimeHh[i]*3600 +
																								deviceConfig.switchPolicy.SwitchOFFTimeMm[i]*60;
		}
	}
}

void setTimmingSwitchPolicySecond2()
{
	RTC_Get();
	uint32_t nowDateSecond = RTC_get_count_from_date(calendar.w_year, calendar.w_month, calendar.w_date);
	for(int i = 0; i < MAX_SWITCH_COUNT; i++)
	{
		if(TIMMING_CONTROL == deviceConfig.switchPolicy.autoSwitchMode[i])
		{
			TimmingSwitchPolicySecond.openPolicy[i].timeInterval_1_IsExecuted = false;
			TimmingSwitchPolicySecond.openPolicy[i].timeInterval_2_IsExecuted = false;
			TimmingSwitchPolicySecond.openPolicy[i].second = nowDateSecond + deviceConfig.switchPolicy.SwitchOnTimeHh[i]*3600 +
																									deviceConfig.switchPolicy.SwitchOnTimeMm[i]*60;		
			TimmingSwitchPolicySecond.closePolicy[i].timeInterval_1_IsExecuted = false;
			TimmingSwitchPolicySecond.closePolicy[i].timeInterval_2_IsExecuted = false;
			TimmingSwitchPolicySecond.closePolicy[i].second = nowDateSecond + deviceConfig.switchPolicy.SwitchOFFTimeHh[i]*3600 +
																								deviceConfig.switchPolicy.SwitchOFFTimeMm[i]*60;
		}
	}
}

uint8_t setTimmingSwitchSunRiseSetSecond()
{
	RTC_Get();
	uint32_t nowDateSecond = RTC_get_count_from_date(calendar.w_year, calendar.w_month, calendar.w_date);
	double longt = deviceConfig.location.longitude,lat = deviceConfig.location.latitude;
	RTC_TimeTypeDef timeRise, timeSet;
	if(0 != getSunrise(deviceConfig.location.longitude, lat, calendar, &timeRise) ||
					0 != getSunset(longt, lat, calendar, &timeSet))
	{
		return 1;
	}
	#if 0
	char tt[32] = {0};
	sprintf(tt, "rise:%02u:%02u\r\n", timeRise.Hours, timeRise.Minutes);
	HAL_UART_Transmit(&huart1, (uint8_t *)tt, strlen(tt), 100);
	sprintf(tt, "set:%02u:%02u\r\n", timeSet.Hours, timeSet.Minutes);
	HAL_UART_Transmit(&huart1, (uint8_t *)tt, strlen(tt), 100);
	#endif
	
	for(int i = 0; i < MAX_SWITCH_COUNT; i++)
	{
		if(SUNRISE_CONTROL == deviceConfig.switchPolicy.autoSwitchMode[i])
		{
			uint32_t openSecond = timeSet.Hours*3600 + timeSet.Minutes*60 - 
																		deviceConfig.switchPolicy.sunSetEarlyOpenMinute[i]*60;
			uint32_t closeSecond = timeRise.Hours*3600 +timeRise.Minutes*60 +
																		deviceConfig.switchPolicy.sunRiseDelayCloseMinute[i]*60;
			deviceConfig.switchPolicy.SwitchOnTimeHh[i] = openSecond/3600;
			deviceConfig.switchPolicy.SwitchOnTimeMm[i] = (openSecond%3600)/60;
			
			deviceConfig.switchPolicy.SwitchOFFTimeHh[i] = closeSecond/3600;
			deviceConfig.switchPolicy.SwitchOFFTimeMm[i] = (closeSecond%3600)/60;
			
			TimmingSwitchPolicySecond.openPolicy[i].timeInterval_1_IsExecuted = false;
			TimmingSwitchPolicySecond.openPolicy[i].timeInterval_2_IsExecuted = false;
			TimmingSwitchPolicySecond.openPolicy[i].second = nowDateSecond + timeSet.Hours*3600 + timeSet.Minutes*60 - 
																												deviceConfig.switchPolicy.sunSetEarlyOpenMinute[i]*60;
			TimmingSwitchPolicySecond.closePolicy[i].timeInterval_1_IsExecuted = false;
			TimmingSwitchPolicySecond.closePolicy[i].timeInterval_2_IsExecuted = false;
			TimmingSwitchPolicySecond.closePolicy[i].second = nowDateSecond + timeRise.Hours*3600 +timeRise.Minutes*60 +
																												deviceConfig.switchPolicy.sunRiseDelayCloseMinute[i]*60;
		}
	}
	
	return 0;
}


uint32_t time485 = 0;
void test485()
{
	if(HAL_GetTick() - time485 > 2000)
	{
		GetMeterOFWattMeter();
		time485 = HAL_GetTick();
	}
}

extern uint32_t           RTC_ReadTimeCounter(RTC_HandleTypeDef *hrtc);
void executeOpenPolicy(int line)
{
	if(line < 0 || line >= SWITCH_NUM)
	{
		return;
	}
	#if 1
	char tt[32];
	sprintf(tt, "open %01d %u\r\n", line, TimmingSwitchPolicySecond.openPolicy[line].second);
	HAL_UART_Transmit(&huart1,(uint8_t *)tt, strlen(tt),500);
	#endif
	CtrlData.SwitchState[line]=0x01;
	TurnOnPowerLineSwitch(line+1);//PowerLineNumber:1,2,3,4,5,6
}

void executeClosePolicy(int line)
{
	if(line < 0 || line >= SWITCH_NUM)
	{
		return;
	}
	char tt[32];
	sprintf(tt, "close %01d %u\r\n", line, TimmingSwitchPolicySecond.closePolicy[line].second);
	HAL_UART_Transmit(&huart1,(uint8_t *)tt, strlen(tt),500);
	CtrlData.SwitchState[line]=0x00;
	TurnOFFPowerLineSwitch(line+1);//PowerLineNumber:1,2,3,4,5,6
}

void executePolicy3()
{
	if(HAL_GetTick() - last_rcv_policy_tick < 10000)
	{
		return;
	}
	//RTC_TimeTypeDef stimeget = {0U};
	#if 0
	HAL_RTC_GetTime(&hrtc, &stimeget, RTC_FORMAT_BIN);
	#else
	RTC_Get();
	#endif
	uint32_t currentSecond = RTC_ReadTimeCounter(&hrtc);
	//uint32_t currentSecond = RTC_get_count(calendar.w_year, calendar.w_month, calendar.w_date, 
	//																																	calendar.hour, calendar.min, calendar.sec);
	uint8_t isAnyPolicyExecuted = false;
  uint8_t switchCommand[SWITCH_NUM] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
	
	for(int i = 0; i < SWITCH_NUM; i++)
	{
		//line policy disable
		if(AUTO_SWITCH_CLOSE == ((deviceConfig.switchPolicy.autoSwitch >> i) & 0x01))
		{
			continue;
		}
		//compare openSecond and closeSecond
		uint32_t openSecond = TimmingSwitchPolicySecond.openPolicy[i].second;
		uint32_t closeSecond = TimmingSwitchPolicySecond.closePolicy[i].second;
		if(openSecond > closeSecond)
		{
			if(currentSecond > closeSecond && currentSecond < openSecond)
			{
				if(false == TimmingSwitchPolicySecond.closePolicy[i].timeInterval_1_IsExecuted)
				{
					#if 0
					_calendar_obj openCal, closeCal;
					RTC_Get_date_time_from_count(openSecond, &openCal);
					RTC_Get_date_time_from_count(closeSecond, &closeCal);
					char str[32] = {0};
					sprintf(str, "open:%04u/%02u/%02u %02u:%02u:%02u\r\n",openCal.w_year, openCal.w_month, openCal.w_date,
																										openCal.hour, openCal.min, openCal.sec);
					HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),200);
					sprintf(str, "close:%04u/%02u/%02u %02u:%02u:%02u\r\n",closeCal.w_year, closeCal.w_month, closeCal.w_date,
																										closeCal.hour, closeCal.min, closeCal.sec);
					HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),200);
					#endif
					executeClosePolicy(i);
					TimmingSwitchPolicySecond.closePolicy[i].timeInterval_1_IsExecuted = true;
					isAnyPolicyExecuted = true;
					switchCommand[i] = 0x00;
				}
			}
			else
			{
				if(currentSecond < closeSecond)
				{
					if(false == TimmingSwitchPolicySecond.openPolicy[i].timeInterval_1_IsExecuted)
					{
						executeOpenPolicy(i);
						TimmingSwitchPolicySecond.openPolicy[i].timeInterval_1_IsExecuted = true;
						isAnyPolicyExecuted = true;
						switchCommand[i] = 0x01;
					}
				}
				else if(currentSecond >= openSecond)
				{
					if(false == TimmingSwitchPolicySecond.openPolicy[i].timeInterval_2_IsExecuted)
					{
						executeOpenPolicy(i);
						TimmingSwitchPolicySecond.openPolicy[i].timeInterval_2_IsExecuted = true;
						isAnyPolicyExecuted = true;
						switchCommand[i] = 0x01;
					}
				}
			}
		}
		else
		{
			if(currentSecond > openSecond && currentSecond < closeSecond)
			{
				if(false == TimmingSwitchPolicySecond.openPolicy[i].timeInterval_1_IsExecuted)
				{
					executeOpenPolicy(i);
					TimmingSwitchPolicySecond.openPolicy[i].timeInterval_1_IsExecuted = true;
					isAnyPolicyExecuted = true;
					switchCommand[i] = 0x01;
				}
			}
			else
			{
				if(currentSecond < openSecond)
				{
					if(false == TimmingSwitchPolicySecond.closePolicy[i].timeInterval_1_IsExecuted)
					{
						executeClosePolicy(i);
						TimmingSwitchPolicySecond.closePolicy[i].timeInterval_1_IsExecuted = true;
						isAnyPolicyExecuted = true;
						switchCommand[i] = 0x00;
					}
				}
				else if(currentSecond >= closeSecond)
				{
					if(false == TimmingSwitchPolicySecond.closePolicy[i].timeInterval_2_IsExecuted)
					{
						executeClosePolicy(i);
						TimmingSwitchPolicySecond.closePolicy[i].timeInterval_2_IsExecuted = true;
						isAnyPolicyExecuted = true;
						switchCommand[i] = 0x00;
					}
				}
			}
		}
	}
	
	if(true == isAnyPolicyExecuted)
	{
		for(int i = 0; i < SWITCH_NUM; i++)
		{
			reportControlSwitchMsg[4+i] = switchCommand[i];
		}
		//NBMsgUp(&huart1, &huart3, reportControlSwitchMsg, 300, 0);
		MQTT_publish((uint8_t *)reportControlSwitchMsg, sizeof(reportControlSwitchMsg));
		
		if(false == IS_FIRST_REPORT)
		{
			HAL_Delay(2000);
			sendPeriodReportData();
		}
	}
}
#endif

#if 1
void checkNBmoduleAndNetwork()
{
	if(NBSendFailetimes >= SEND_FAILED_TIMES_TO_REREGIST)
	{
		char str[] = "NBSendFailetimes reached, reregiste!!\r\n"; 
		HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),500);
		NBSendFailetimes = 0;
		goto REREGISTE;
	}
	
	uint32_t nowTick = HAL_GetTick()/1000;
	if(nowTick - last_check_network_tick > CHECK_NETWORK_PERIOD)
	{
		last_check_network_tick = nowTick;
		//AT check
		int i = 0;
		for(i = 0; i < 3; i++)
		{
			if (0 == NB_CommCheck(&huart3))
			{
				break;
			}
		}
		if(3 == i)
		{
			char str[] = "at error, reregiste!!\r\n";
			HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),500);
			goto REREGISTE;
		}
		
		StringCpy(AttOpMsg.AtCmd, (uint8_t *)ATCREG,strlen(ATCREG));// Query whether the network is activated
		AttOpMsg.CmdLen=strlen(ATCREG);
		memset(AttRetMsg.AtCmd,0,sizeof(AttRetMsg.AtCmd));//clear array 
		NB_ATCMD_NO_PRINT(&huart1, &huart3, &AttOpMsg,&AttRetMsg, 300);
		if (NB_ACK_Check((char *)AttRetMsg.AtCmd,AttRetMsg.CmdLen,"+CREG: 0,1") && 
				NB_ACK_Check((char *)AttRetMsg.AtCmd,AttRetMsg.CmdLen,"+CREG: 0,5"))
		{
			char str[] = "ATCREG error, reregiste!!\r\n"; 
			HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),500);
			goto REREGISTE;
		}
		
		StringCpy(AttOpMsg.AtCmd, (uint8_t *)ATCGREG,strlen(ATCGREG));// Query whether the network is activated
		AttOpMsg.CmdLen=strlen(ATCGREG);
		memset(AttRetMsg.AtCmd,0,sizeof(AttRetMsg.AtCmd));//clear array 
		NB_ATCMD_NO_PRINT(&huart1, &huart3, &AttOpMsg,&AttRetMsg, 300);
		if (NB_ACK_Check((char *)AttRetMsg.AtCmd,AttRetMsg.CmdLen,"+CGREG: 0,1") && 
				NB_ACK_Check((char *)AttRetMsg.AtCmd,AttRetMsg.CmdLen,"+CGREG: 0,5"))
		{
			char str[] = "ATCGREG error, reregiste!!\r\n"; 
			HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),500);
			goto REREGISTE;
		}
		
		StringCpy(AttOpMsg.AtCmd, (uint8_t *)ATCGATT,strlen(ATCGATT));// Query whether the network is activated
		AttOpMsg.CmdLen=strlen(ATCGATT);
		memset(AttRetMsg.AtCmd,0,sizeof(AttRetMsg.AtCmd));//clear array 
		NB_ATCMD_NO_PRINT(&huart1, &huart3, &AttOpMsg,&AttRetMsg, 300);
		if (NB_ACK_Check((char *)AttRetMsg.AtCmd,AttRetMsg.CmdLen,"+CGATT: 1"))
		{
			char str[] = "CGATT error, reregiste!!\r\n"; 
			HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),500);
			goto REREGISTE;
		}
		#if 0
		StringCpy(AttOpMsg.AtCmd,(uint8_t *)ATCEREG,9);//EPS Network Registration Status
		AttOpMsg.CmdLen=9;
		memset(AttRetMsg.AtCmd,0,sizeof(AttRetMsg.AtCmd));//clear array 
		NB_ATCMD_NO_PRINT(&huart1, &huart3, &AttOpMsg,&AttRetMsg, 300);
		if (NB_ACK_Check((char *)AttRetMsg.AtCmd,AttRetMsg.CmdLen,"+CEREG: 0,1"))
		{
			char str[] = "CEREG error, reregiste!!\r\n";
			HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),500);
			goto REREGISTE;
		}
		#endif
		/*
		StringCpy(AttOpMsg.AtCmd,(uint8_t *)ATQLWUPDATE,strlen(ATQLWUPDATE));//EPS Network Registration Status
		AttOpMsg.CmdLen=strlen(ATQLWUPDATE);
		memset(AttRetMsg.AtCmd,0,sizeof(AttRetMsg.AtCmd));//clear array 
		NB_ATCMD_NO_PRINT(&huart1, &huart3, &AttOpMsg,&AttRetMsg, 300);//result return not imediatly, not need to check 
		*/
		
		#if 0
		if(0 == CT_PLATFORM_CONNECT_FLAG)
		{
			char str[] = "not register to ct platform, reregiste!\r\n";
			HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),500);
			goto REREGISTE;
		}
		else
		{
			return;
		}
		#endif
		return;
	}
	else
	{
		return;
	}
	
REREGISTE:
	last_check_network_tick = HAL_GetTick()/1000;
	if(0 == network_first_unavailable_tick)
	{
		network_first_unavailable_tick = HAL_GetTick();
	}
	NB_Attachment(&huart1, &huart3, 3);
	MQTT_Init();
	return;
}

void NBPowerRestart()
{
	//int j = 0;
	//char RepChar[40] = {0};
	NB_OFF();
	HAL_UART_Transmit(&huart1,(uint8_t*)NBPowerOff, strlen(NBPowerOff),200);
	HAL_Delay(800);
	NB_ON();
	HAL_UART_Transmit(&huart1,(uint8_t*)NBPowerOff, strlen(NBPowerOn),200);
	HAL_Delay(200);
	NB_StartUp();					
}

#endif

void setTimmingSwitchPolicyUnexecute(void)
{
	for(int i = 0; i < SWITCH_NUM; i++)
	{
			TimmingSwitchPolicySecond.openPolicy[i].timeInterval_1_IsExecuted = false;
			TimmingSwitchPolicySecond.openPolicy[i].timeInterval_2_IsExecuted = false;
			TimmingSwitchPolicySecond.closePolicy[i].timeInterval_1_IsExecuted = false;
			TimmingSwitchPolicySecond.closePolicy[i].timeInterval_2_IsExecuted = false;
	}
}	
void printDeviceConfig(DEVICE_CONFIG deviceConfig)
{
	char tt[256] = {0};
	//DEVICE_CONFIG deviceConfig;
	//readDeviceConfig(&deviceConfig);
	
	sprintf(tt, "\r\n version:%s\r\n", deviceConfig.version);
	HAL_UART_Transmit(&huart1,(uint8_t*)tt,strlen(tt),500);
	//sprintf(tt, "\r\n autoSwitch:%u\r\n", deviceConfig.autoSwitch);
	//HAL_UART_Transmit(&huart1,(uint8_t*)tt,strlen(tt),500);
	//sprintf(tt, "\r\n autoSwitch mode:%u\r\n", deviceConfig.autoSwitchMode);
	//HAL_UART_Transmit(&huart1,(uint8_t*)tt,strlen(tt),500);
	sprintf(tt, "\r\n location:%.6f %.6f\r\n", deviceConfig.location.longitude, deviceConfig.location.latitude);
	HAL_UART_Transmit(&huart1,(uint8_t*)tt,strlen(tt),500);
	
	sprintf(tt, "\r\nreportCondition:\r\nperiod:%05u, powerDelta:%03u, tempDelta:%02u\r\n", 
																									deviceConfig.reportConfig.period,
																									deviceConfig.reportConfig.powerDelta,
																									deviceConfig.reportConfig.tempDelta);
	HAL_UART_Transmit(&huart1,(uint8_t*)tt,strlen(tt),500);
	
	
	
	memset(tt, 0, sizeof(tt));
	sprintf(tt, "\r\ntimmingSwitch:\r\nperiod:%05u, powerDelta:%03u, tempDelta:%02u\r\n", 
																									deviceConfig.reportConfig.period,
																									deviceConfig.reportConfig.powerDelta,
																									deviceConfig.reportConfig.tempDelta);
	sprintf(tt, "autoSwitch:%x\r\n", deviceConfig.switchPolicy.autoSwitch);
	HAL_UART_Transmit(&huart1,(uint8_t*)tt,strlen(tt),500);					
	
	for(int i = 0; i < 6; i++)
	{
		sprintf(tt, "switch%d: mode: %01d earlyOpen:%02u delayClose:%02u\r\n", i, 
																					deviceConfig.switchPolicy.autoSwitchMode[i],
																					deviceConfig.switchPolicy.sunSetEarlyOpenMinute[i],
																					deviceConfig.switchPolicy.sunRiseDelayCloseMinute[i]);
		HAL_UART_Transmit(&huart1,(uint8_t*)tt,strlen(tt),500);
	}
	
	for(int i = 0; i < 6; i++)
	{
		sprintf(tt, "switch%01d open:%02u %02u close:%02u %02u\r\n", i, 
																					deviceConfig.switchPolicy.SwitchOnTimeHh[i],
																					deviceConfig.switchPolicy.SwitchOnTimeMm[i],
																					deviceConfig.switchPolicy.SwitchOFFTimeHh[i],
																					deviceConfig.switchPolicy.SwitchOFFTimeMm[i]);
		HAL_UART_Transmit(&huart1,(uint8_t*)tt,strlen(tt),500);
	}
	
	
}

/***********************************************/
/* @brief  Shows the current time (HH:MM:SS).  */
/* @param  None                                */
/* @retval None                                */
/***********************************************/   
uint32_t timePrintTick = 0;
void Time_Show(void)
{
  //printf("\n\r");
	
	RTC_TimeTypeDef stimeget;
	RTC_DateTypeDef sDate;
	uint8_t aShowTime[50] = {0};
	//uint8_t ch;
	
  /* Infinite loop */
  while (1)
  { 
		//RTC_TimeShow(aShowTime);
			
		//EnableAutoSwitch
		/* Get the RTC current Date */
		//HAL_RTC_GetDate(&hrtc, &sdateget, RTC_FORMAT_BIN);
		/* Get the RTC current Time */
		if(HAL_GetTick() - timePrintTick > 1000)
		{
			timePrintTick = HAL_GetTick();
			HAL_UART_Transmit(&huart1,(uint8_t*)"\r\n", 2,10);
			HAL_RTC_GetTime(&hrtc, &stimeget, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
			/* Display time Format : hh:mm:ss */
			sprintf((char*)aShowTime,"Date:%04d-%02d-%02d Time:%02d:%02d:%02d\r\n",\
						sDate.Year, sDate.Month, sDate.Date,\
						 stimeget.Hours, stimeget.Minutes, stimeget.Seconds);
			HAL_UART_Transmit(&huart1,(uint8_t*)aShowTime, strlen((char *)aShowTime),100);
			
			char tt[32] = {0};
			sprintf(tt, "tim time: %05u\r\n", time);
			HAL_UART_Transmit(&huart1,(uint8_t*)tt, strlen(tt),100);
		}
  }
}
/**
  * @brief  The application entry point.
  * @retval int
  */

static void RTC_AlarmConfig(void);

RTC_AlarmTypeDef reset_policy_alarm_structure;
RTC_AlarmTypeDef sysc_time_alarm_structure;
static void RTC_AlarmConfig(void)
{
  //RTC_DateTypeDef  sdatestructure;
  //RTC_TimeTypeDef  stimestructure;
  //RTC_AlarmTypeDef salarmstructure;
 #if 0
  /*##-1- Configure the Date #################################################*/
  /* Set Date: Tuesday February 18th 2014 */
  sdatestructure.Year = 0x14;
  sdatestructure.Month = RTC_MONTH_FEBRUARY;
  sdatestructure.Date = 0x18;
  sdatestructure.WeekDay = RTC_WEEKDAY_TUESDAY;
  
  if(HAL_RTC_SetDate(&hrtc,&sdatestructure,RTC_FORMAT_BCD) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  } 
  
  /*##-2- Configure the Time #################################################*/
  /* Set Time: 02:20:00 */
  stimestructure.Hours = 0x08;
  stimestructure.Minutes = 0x20;
  stimestructure.Seconds = 0x00;
  
  if(HAL_RTC_SetTime(&hrtc,&stimestructure,RTC_FORMAT_BCD) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  }  
#endif
	//RTC_Get();
	//MY_RTC_SetTime(&hrtc, &calendar);
  /*##-3- Configure the RTC Alarm peripheral #################################*/
  /* Set Alarm to 02:20:30 
     RTC Alarm Generation: Alarm on Hours, Minutes and Seconds */
  HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);
	
	reset_policy_alarm_structure.Alarm = RTC_ALARM_A;
	sysc_time_alarm_structure.Alarm = RTC_ALARM_A;
	#if 1
	reset_policy_alarm_structure.AlarmTime.Hours = 0;
  reset_policy_alarm_structure.AlarmTime.Minutes = 0;
  reset_policy_alarm_structure.AlarmTime.Seconds = 1;
	
	sysc_time_alarm_structure.AlarmTime.Hours = 8;
  sysc_time_alarm_structure.AlarmTime.Minutes = 0;
  sysc_time_alarm_structure.AlarmTime.Seconds = 0;
	
	#else
		RTC_TimeTypeDef nTime;
		HAL_RTC_GetTime(&hrtc,&nTime,RTC_FORMAT_BIN);
		salarmstructure.AlarmTime.Hours = nTime.Hours;
		salarmstructure.AlarmTime.Minutes = nTime.Minutes+1;
		salarmstructure.AlarmTime.Seconds = nTime.Seconds;
	#endif
  if(HAL_RTC_SetAlarm_IT(&hrtc,&reset_policy_alarm_structure,RTC_FORMAT_BIN) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  }
}


void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *nhrtc)
{
	//RTC_TimeTypeDef nTime;
	RTC_AlarmTypeDef alarmStructure;
	
	//HAL_RTC_GetTime(nhrtc,&nTime,RTC_FORMAT_BIN);
	RTC_Get();
	char str[32] = {0};
  sprintf(str, "Alarm--%d:%d:%d\r\n",calendar.hour,calendar.min, calendar.sec);
	//sprintf(str, "Alarm--%d:%d:%d\r\n",nTime.Hours,nTime.Minutes, nTime.Seconds);
	HAL_UART_Transmit(&huart1,(uint8_t *)str, strlen(str),500);
	#if 0
	if(calendar.hour == reset_policy_alarm_structure.AlarmTime.Hours)
	{
		alarmStructure = sysc_time_alarm_structure;
		/*
		for(int i = 0; i < SWITCH_NUM; i++)
		{
				TimmingSwitchPolicySecond.openPolicy[i].isExecuted = false;
				TimmingSwitchPolicySecond.closePolicy[i].isExecuted = false;
		}
		*/
		setTimmingSwitchPolicyUnexecute();
		char str[] = "set unexecuted\r\n";
		HAL_UART_Transmit(&huart1,(uint8_t *)str, strlen(str),500);
	}
	else if(calendar.hour == sysc_time_alarm_structure.AlarmTime.Hours)
	{
		char str[] = "sysc time!\r\n";
		HAL_UART_Transmit(&huart1,(uint8_t *)str, strlen(str),500);
		alarmStructure = reset_policy_alarm_structure;
		//sysc_time_from_NB();
		NEED_TO_SYSC_TIME = 1;
	}
	#else
	alarmStructure = reset_policy_alarm_structure;
	//setTimmingSwitchPolicyUnexecute();
	setTimmingSwitchPolicySecond2();
	setTimmingSwitchSunRiseSetSecond();
	
	char str1[] = "set unexecuted\r\n";
	HAL_UART_Transmit(&huart1,(uint8_t *)str1, strlen(str1),500);
	#endif
	
  if(HAL_RTC_SetAlarm_IT(&hrtc,&alarmStructure,RTC_FORMAT_BIN) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  }
		
}

#if 1
void sysc_time_from_network()
{
	char *Index;
	RTC_DateTypeDef  sdate;
	uint16_t year = 0U;
  RTC_TimeTypeDef  stime;
	
	StringCpy(AttOpMsg.AtCmd,(uint8_t *)ATCCLK,strlen(ATCCLK));//get Network date and time
	AttOpMsg.CmdLen=strlen(ATCCLK);
	NB_ATCMD(&huart1,&huart3,&AttOpMsg,&AttRetMsg,500);
	Index=strstr((char *)AttRetMsg.AtCmd,"+CCLK: ");//location CCLK
	if(Index!=NULL && AttRetMsg.CmdLen > 24)
	{ //Date and time geted
		//Respond format    +CCLK: "20/09/17,03:50:01+00"
		//sdate.Year = DecNum(Index[9])*10+DecNum(Index[10]);
		year = 2000 + DecNum(Index[8])*10+DecNum(Index[9]);
		sdate.Month = DecNum(Index[11])*10+DecNum(Index[12]);
		sdate.Date = DecNum(Index[14])*10+DecNum(Index[15]);
		sdate.WeekDay = RTC_WEEKDAY_SATURDAY;
		
		stime.Hours = DecNum(Index[17])*10+DecNum(Index[18]);
		stime.Minutes = DecNum(Index[20])*10+DecNum(Index[21]);
		stime.Seconds = DecNum(Index[23])*10+DecNum(Index[24]);
		
		uint32_t count = RTC_get_count_from_date_time(year,sdate.Month,sdate.Date,
																									stime.Hours,stime.Minutes,stime.Seconds);
		count = count+8*3600;//8 is time zone
		_calendar_obj tmpCalendar;
		RTC_Get_date_time_from_count(count, &tmpCalendar);
		
		if(HAL_OK != RTC_Set(&hrtc, tmpCalendar.w_year, tmpCalendar.w_month, tmpCalendar.w_date, 
														tmpCalendar.hour, tmpCalendar.min, tmpCalendar.sec))
		{
			char str1[] = "sysc time failed!\r\n";
			HAL_UART_Transmit(&huart1,(uint8_t *)str1, strlen(str1),100);		
			return;
		}

		char str1[] = "sysc time success!\r\n";
		HAL_UART_Transmit(&huart1,(uint8_t *)str1, strlen(str1),100);
	}
}


void sysc_time()
{
	#if 0
	if(1 == NEED_TO_SYSC_TIME)
	{
		NEED_TO_SYSC_TIME = 0;
		sysc_time_from_NB();
	}
	#else
	if(HAL_GetTick()/1000 - last_sysc_time_second > SYSC_TIME_GAP_SECOND)
	{
		last_sysc_time_second = HAL_GetTick()/1000;
		sysc_time_from_network();
	}
	#endif
}



void sendPeriodReportData()
{
	paramAndTimmingSwitchTableMsg[1]=0x00;
	paramAndTimmingSwitchTableMsg[2]=0xc5;
	GetCurrentData(&CtrlData,paramAndTimmingSwitchTableMsg);//get data
	//DisplayStringHex(&huart1,replyMsg,25);//display respond message,25 include length byte
	for(int i = 0; i < SWITCH_NUM; i++)
	{
	 paramAndTimmingSwitchTableMsg[5+i] = BoxGetSwitchState(i+1);
	}
	//NBMsgUp(&huart1,&huart3,paramAndTimmingSwitchTableMsg,300, 0);//report	
	MQTT_publish((uint8_t *)paramAndTimmingSwitchTableMsg, sizeof(paramAndTimmingSwitchTableMsg));
}
void key_event_process(void)
{
	
	uint8_t key_value = key_handle(PKEY_GPIO_Port,PKEY_Pin);

	if(key_value == KEY_SHORT)
	{
		#if 1
		HAL_UART_Transmit(&huart1, (uint8_t *)"short\r\n", 7, 100);
		turnOnAllLineSwitch();
		for(int i = 0; i < SWITCH_NUM; i++)
		{
			reportControlSwitchMsg[4+i] = 0x01;
			CtrlData.SwitchState[i]=0x01;
		}
		//NBMsgUp(&huart1, &huart3, reportControlSwitchMsg, 300, 0);
		MQTT_publish((uint8_t *)reportControlSwitchMsg, sizeof(reportControlSwitchMsg));
		
		HAL_Delay(2000);
		sendPeriodReportData();
		
		#endif
	}
	else if(key_value == KEY_LONG)
	{
		#if 1 
		turnOffAllLineSwitch();
		for(int i = 0; i < SWITCH_NUM; i++)
		{
			reportControlSwitchMsg[4+i] = 0x00;
			CtrlData.SwitchState[i]=0x00;
		}
		//NBMsgUp(&huart1, &huart3, reportControlSwitchMsg, 300, 0);
		 MQTT_publish((uint8_t *)reportControlSwitchMsg, sizeof(reportControlSwitchMsg));
		
		HAL_Delay(2000);
		sendPeriodReportData();
		#endif
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3)
	{
		time++;
	}
}


void initCtrlData()
{
	CtrlData.Gapstart=HAL_GetTick();
	CtrlData.ReportGapstart=HAL_GetTick();
	CtrlData.Phase=0;
	
	CtrlData.ReportCondPeriod = deviceConfig.reportConfig.period;
	CtrlData.ReportGap=(uint32_t)CtrlData.ReportCondPeriod*1000U;//UINT is ms
	CtrlData.ReportCondPower = deviceConfig.reportConfig.powerDelta;
	CtrlData.ReportCondTemp = deviceConfig.reportConfig.tempDelta;
	
}



#endif


void SoftReset(void)
{
    __set_FAULTMASK(1);
    NVIC_SystemReset(); 
}

#define SYSTEM_RESET_TICK_GAP (30*60*1000)//30分钟
void system_reset(void)
{
	uint32_t nowTick = HAL_GetTick();
	if(0 != network_first_unavailable_tick)
	{
		if(nowTick - network_first_unavailable_tick > SYSTEM_RESET_TICK_GAP)
		{
			SoftReset();
		}
	}
}


int query_urc_msg(UART_HandleTypeDef *NBhuart, uint8_t *msg, uint8_t times, uint16_t Timeout)
{
	//const char Toolong[]="\nToo long Repond!\n";
	//const char TimeOutString[]="\nRespond Time out!\n";
	//volatile uint16_t NumOfChar;	
	volatile uint16_t Cnt, NumOfChar, tryTimes = 0;
	uint32_t tickstart = 0U;
	
	memset(AttRetMsg.AtCmd,0,sizeof(AttRetMsg.AtCmd));//clear array
	while(tryTimes < times)
	{
		tryTimes++;
		Cnt=0;
		NumOfChar = 0;
		tickstart = HAL_GetTick();
		for(;;)
		{
			if(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_RXNE) != RESET)
			{
					AttRetMsg.AtCmd[Cnt]=(uint8_t)(NBhuart->Instance->DR & 0xff);
					__HAL_UART_CLEAR_FLAG (NBhuart, UART_FLAG_RXNE);
					if (Cnt>254) 
					{
						//memset(AttRetMsg.AtCmd,0,sizeof(AttRetMsg.AtCmd));//clear array 
						//StringCpy(AttRetMsg.AtCmd,(uint8_t *)Toolong,sizeof(Toolong));
						//NumOfChar=sizeof(Toolong);//force to end of string
						break;
					}
					else
						Cnt++;
					//tickstart = HAL_GetTick();
			}
			else
			{
				if ((HAL_GetTick()-tickstart)>Timeout)
				{
					if (Cnt<2) 
					{
						//memset(AttRetMsg.AtCmd,0,sizeof(AttRetMsg.AtCmd));//clear array 
						//StringCpy(AttRetMsg.AtCmd,(uint8_t *)TimeOutString,sizeof(TimeOutString));
						//NumOfChar=sizeof(TimeOutString);
						break;
					}
					else
					{
						NumOfChar=Cnt-1;
						break;
					}//if (Cnt<2)	
				}
			}//if()
		}//for(;;)
	
		AttRetMsg.CmdLen = NumOfChar; 
		if(0 == NB_ACK_Check((char *)AttRetMsg.AtCmd, AttRetMsg.CmdLen, (char *)msg))
		{
			HAL_UART_Transmit(&huart1, AttRetMsg.AtCmd, AttRetMsg.CmdLen, 100);
			HAL_UART_Transmit(&huart1,(uint8_t*)"\r\n",2,50);
			break;
		}
	}	
	
	if(times == tryTimes)
	{
		return -1;
	}
	
	return 0;
}



const char ATQMTPUBEX_FORMAT[] = "AT+QMTPUBEX=0,1,1,0,\"device/4G/%s/up\",%u";//client,msgid,qos,retain
int  MQTT_Init(void)
{
	const char ATQMTOPEN[] = "AT+QMTOPEN=0,\"message.cloud.glxt.com\",61613";
	//const char ATQMTOPEN[] = "AT+QMTOPEN=0,\"desktop.cloud.glxt.com\",8088";
	const char ATQMTOPEN_QUERY[] = "AT+QMTOPEN?";
	const char ATQMTCONN_FORMAT[] = "AT+QMTCONN=0,\"%s\", \"admin\", \"password\"";
	const char ATQMTCONN_QUERY[] = "AT+QMTCONN?";
	const char ATQMTSUB[] = "AT+QMTSUB=0,1,%s,1";
	const char ATRCVMODE[] = "AT+QMTCFG=\"recv/mode\",0,0,1";
	char str[64] = {0};
	char buf[96] = {0};
	
	const char C0_OPEN_RSP[] = "+QMTOPEN: 0,0";
	const char C0_CONN_RSP[] = "+QMTCONN: 0,0,0";
	const char C0_SUB_RSP[] = "+QMTSUB: 0,1,0,1";
	
	StringCpy(AttOpMsg.AtCmd,(uint8_t *)ATRCVMODE,strlen(ATRCVMODE));
	AttOpMsg.CmdLen=strlen(ATRCVMODE);
	NB_ATCMD(&huart1,&huart3,&AttOpMsg,&AttRetMsg,300);
	if (0 != NB_ACK_Check((char *)AttRetMsg.AtCmd,AttRetMsg.CmdLen,"OK"))
	{
		sprintf(str, "mqtt config rcv mode failed!\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 100);
		return -1;
	}
	
	StringCpy(AttOpMsg.AtCmd,(uint8_t *)ATQMTOPEN,strlen(ATQMTOPEN));
	AttOpMsg.CmdLen=strlen(ATQMTOPEN);
	NB_ATCMD(&huart1,&huart3,&AttOpMsg,&AttRetMsg,800);
	if (0 != NB_ACK_Check((char *)AttRetMsg.AtCmd,AttRetMsg.CmdLen,(char *)C0_OPEN_RSP))
	{
		if(0 != query_urc_msg(&huart3, (uint8_t *)C0_OPEN_RSP, 20, 1000))
		{
			sprintf(str, "mqtt open failed!\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 100);
			return -1;
		}
	}
	
	HAL_Delay(200);
	sprintf(buf, ATQMTCONN_FORMAT, deviceId);
	StringCpy(AttOpMsg.AtCmd,(uint8_t *)buf,strlen(buf));
	AttOpMsg.CmdLen=strlen(buf);
	NB_ATCMD(&huart1,&huart3,&AttOpMsg,&AttRetMsg,500);
	
	if (0 != NB_ACK_Check((char *)AttRetMsg.AtCmd,AttRetMsg.CmdLen,(char *)C0_CONN_RSP))
	{
		if(0 != query_urc_msg(&huart3, (uint8_t *)C0_CONN_RSP, 5, 1000))
		{
			sprintf(str, "mqtt connect failed!\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 100);
			return -1;
		}
	}
	
	HAL_Delay(200);
	int len = strlen(deviceId);
	if(strlen(deviceId) + sizeof(SUB_TOPIC_FORMAT) > sizeof(sub_topic))
	{
		sprintf(str, "topic too long!\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 100);
		return -1;
	}
	sprintf(sub_topic, SUB_TOPIC_FORMAT, deviceId);
	//sprintf(pub_topic, PUB_TOPIC_FORMAT, deviceId);
	memset(buf, 0, sizeof(buf));
	sprintf(buf, ATQMTSUB, sub_topic);
	StringCpy(AttOpMsg.AtCmd,(uint8_t *)buf,strlen(buf));
	AttOpMsg.CmdLen=strlen(buf);
	NB_ATCMD(&huart1,&huart3,&AttOpMsg,&AttRetMsg,500);
	if (0 != NB_ACK_Check((char *)AttRetMsg.AtCmd,AttRetMsg.CmdLen,(char *)C0_SUB_RSP))
	{
		if(0 != query_urc_msg(&huart3, (uint8_t *)C0_SUB_RSP, 5, 1000))
		{
			sprintf(str, "mqtt sub failed!\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 100);
			return -1;
		}
	}
	
	return 0; 
}


int MQTT_publish(uint8_t *hexData, uint16_t len)
{
	uint8_t buf[256] = {0};
	
	if(NULL == hexData || len*2 > 256)
	{
		return -1; 
	}
	sprintf((char *)buf,ATQMTPUBEX_FORMAT, deviceId, len*2);
	uint16_t msgLen = strlen((char *)buf);
	memset(AttOpMsg.AtCmd, 0, sizeof(AttOpMsg.AtCmd));
	StringCpy(AttOpMsg.AtCmd,(uint8_t *)buf,msgLen);
	AttOpMsg.CmdLen = msgLen;
	memset(AttRetMsg.AtCmd,0,sizeof(AttRetMsg.AtCmd));//clear array 
	NB_ATCMD(&huart1,&huart3,&AttOpMsg,&AttRetMsg,50);

	HAL_Delay(5);
	memset(AttRetMsg.AtCmd,0,sizeof(AttRetMsg.AtCmd));//clear array 
	memset(AttOpMsg.AtCmd, 0, sizeof(AttOpMsg.AtCmd));
	memset(buf, 0, sizeof(buf));
	
	uint8_t NumOfChar=hexData[0];//Get the length of message 
	uint16_t temp=chkcrc((char *)hexData,0,NumOfChar-1);//CRC include Length byte
	hexData[NumOfChar-1]=temp>>8;
	hexData[NumOfChar]=temp&0x00ff;	
	
	HexToString(hexData, buf,len);
	StringCpy(AttOpMsg.AtCmd,(uint8_t *)buf,len*2);
	AttOpMsg.CmdLen = len*2;
	NB_ATCMD(&huart1,&huart3,&AttOpMsg,&AttRetMsg,200);
		//publish error,error count add 1
#if 1
	if(0 != NB_ACK_Check((char *)AttRetMsg.AtCmd, AttRetMsg.CmdLen, "OK"))
	{
		NBSendFailetimes++;
		return -1;
	}
#endif
	
	return 0;
}

int chartoint(const char* port)
{
	int tmp=0;
	while (*port >= '0' && *port <= '9')
	{
		tmp = tmp*10+*port-'0';
		port++;
	}
	return tmp;
}

int main(void)
{
	#if 1
  /* USER CODE BEGIN 1 */
	static const char SWITCH_ON[]="SWITCH ON!\r";
	static const char SWITCH_OFF[]="SWITCH OFF!\r";	
	static const char NBON[]="NB POWER ON!\r";
	static const char NBOFF[]="NB POWER OFF!\r";
	static const char NBRST[]="NB RESET!\r";
	static const char C1Read[]="C1(Read).....\r";
	static const char C2DLCond[]="C2(Define Conditions).....\r";
	static const char C3DLTimg[]="C3(Define Timing table).....\r";
	static const char C4SDIM[]="C4(Switch/DIM).....\r";
	static const char INVLID[]="***POWER LINE NOT EXIST***\r";
	static const char ComUND[]="***COMMAND UNDEFINE!***\r";
	static const char INCMDFORMAT[]="***INVLID COMMAND FORMAT!***\r";
	//static const char RTC_nyc[]="\r\n\n RTC not yet configured....";
	//static const char RTC_con[]="\r\n RTC configured....";
	//static const char RTC_PORo[]="\r\n\n Power On Reset occurred....";
	//static const char RTC_ExRSTo[]="\r\n\n External Reset occurred....";
	//static const char RTC_Nontc[]="\r\n No need to configure RTC....";
	
	volatile uint16_t PackageCHK,PackageCRC;
	volatile uint16_t temp,tp,j,OpNum;
	volatile uint8_t CmdNumber;
	char RepChar[64];//,OutCmdFrame[64],DimPercent;
	char *pch,*pch3;
	uint8_t NBReceiveData[64],NumOfData;
	//uint8_t aShowTime[50] = {0};
	RTC_TimeTypeDef  stime;
	
  /* USER CODE END 1 */
	
  /* MCU Configuration--------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	/***********************RTC Config******************************/
	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
	MX_TIMx_Init();
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_RTC_Init();
#endif
  /* USER CODE BEGIN 2 */ 
	NB_Attachment(&huart1,&huart3, 3);
	MQTT_Init();
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	initDeviceConfigFromFlash(&deviceConfig);
	initCtrlData();
	setTimmingSwitchPolicySecond2();
	setTimmingSwitchSunRiseSetSecond();
	
	
  /* Configure RTC Alarm */
  RTC_AlarmConfig();
  while (1)
  {
    /* USER CODE END WHILE */
		/*--------------------------Date and Time Test-------------------------*/		
		key_event_process();
		/*----------------------------Timing event-----------------------------*/
		/*-----------------Data collection and condition report----------------*/
		MultiEventProcess(&CtrlData,paramAndTimmingSwitchTableMsg);
		/*-----------------collection and report end---------------------------*/
		
		executePolicy3();
		checkNBmoduleAndNetwork();
		sysc_time();
		//如果长时间无法注册到电信平台，重启系统
		system_reset();
		
		/*-------------------------Moniter USART3(NB)--------------------------*/
		memset(NBMsg.AtCmd,0,sizeof(NBMsg.AtCmd));//clear array 
		NBMsg.CmdLen=0;
    if(GetNBMsg(&huart3,&NBMsg)== HAL_OK) //Query NB Message
    {
			//HAL_UART_Transmit(&huart1,(uint8_t*)NBMsg.AtCmd,NBMsg.CmdLen,500); 
		  if(NBMsg.CmdLen!=0)
		  {
				//+QMTRECV: 0,0,"test/down",3,"111"
			  if (strstr((char *)NBMsg.AtCmd,"+QMTRECV:"))
				{
					HAL_UART_Transmit(&huart1,(uint8_t*)NBMsg.AtCmd,NBMsg.CmdLen,500);//Display NB Message
					pch=strchr((char *)NBMsg.AtCmd,',');
					pch=strchr(pch+1,',');
					if(0 != strncmp(sub_topic, pch+1, strlen(sub_topic)))
					{
						continue;
					}
					pch=strchr(pch+1,',');
					NumOfData = chartoint(pch+1);
					
					
					pch=strchr(pch+1,',');
					StringToHex((uint8_t *)(pch+2),NBReceiveData, NumOfData);//ASCII string convert to Hex Arry
					//DisplayStringHex(&huart1,(char*)NBReceiveData,NumOfData);
					uint8_t hexLen = NumOfData/2;
					PackageCHK=chkcrc((char *)NBReceiveData,0, hexLen-2);//include Length byte,not include CRC bytes
					PackageCRC=NBReceiveData[hexLen-2]*256+NBReceiveData[hexLen-1];//CRC in package	
					sprintf(RepChar,"Check CRC:0x%04x\r\n",PackageCHK);
					HAL_UART_Transmit(&huart1,(uint8_t*)RepChar,18,500);
					sprintf(RepChar,"Package CRC:0x%04x\r\n",PackageCRC);
					HAL_UART_Transmit(&huart1,(uint8_t*)RepChar,20,500);
					temp=NBReceiveData[2];

					if(PackageCRC==PackageCHK)
					{//CRC check PASSED
						 switch(temp)
						 {
							 case 0xC1://Read Parameter command
								 if(NBReceiveData[3]==00)//00 is mark for read
								 {
									 //GetCurrentData(&CtrlData,SwitchCtrlParaMsg);//get data
									 GetCurrentData(&CtrlData,paramAndTimmingSwitchTableMsg);//get data and policy
									 paramAndTimmingSwitchTableMsg[1] = NBReceiveData[1];
									 paramAndTimmingSwitchTableMsg[2] = 0xC1;
									 //DisplayStringHex(&huart1,SwitchCtrlParaMsg,25);//display respond message,25 include length byte
									 //NBMsgUp(&huart1,&huart3,paramAndTimmingSwitchTableMsg,300, 0);//respond to Read command									 
									 MQTT_publish((uint8_t *)paramAndTimmingSwitchTableMsg, sizeof(paramAndTimmingSwitchTableMsg));
									 HAL_UART_Transmit(&huart1,(uint8_t*)C1Read,strlen(C1Read),200);
								 }
								 break;
							 case 0xC2:	
								 if(NBReceiveData[3]==0xD2)//0xD2 mark for Switch Controller
								 {//
									   //process Setup parameter
										 int re = -1;
										 DEVICE_CONFIG tmpConfig = deviceConfig;
										 tmpConfig.reportConfig.period = (uint16_t)NBReceiveData[5]*256+(uint16_t)NBReceiveData[6];
										 tmpConfig.reportConfig.powerDelta = (uint16_t)NBReceiveData[7]*256+(uint16_t)NBReceiveData[8];
										 tmpConfig.reportConfig.tempDelta = (uint16_t)NBReceiveData[9]*256+(uint16_t)NBReceiveData[10];
										 re = writeDeviceConfig(&tmpConfig);
										 if(0 == re)
										 {
											 deviceConfig = tmpConfig;
											 CtrlData.ReportCondPeriod= tmpConfig.reportConfig.period;
											 CtrlData.ReportCondPower= tmpConfig.reportConfig.powerDelta;
											 CtrlData.ReportCondTemp=tmpConfig.reportConfig.tempDelta;
											 CtrlData.ReportGap=CtrlData.ReportCondPeriod*1000U;
											 
											 if(!NBReceiveData[4])
											 {
												  CondWrOKMsg[1] = NBReceiveData[1];
													//NBMsgUp(&huart1,&huart3,(char *)CondWrOKMsg,300, 0);
												 MQTT_publish((uint8_t *)CondWrOKMsg, sizeof(CondWrOKMsg));
											 }
											 else
											 {
												 for(j=6;j<12;j++)//fill data bytes
												 {
													 CondWrWithDataOKMsg[j]=NBReceiveData[j-1];
												 }
												 //PackageCHK=chkcrc(CondWrWithDataOKMsg,0,12);//include Length byte
												 //CondWrWithDataOKMsg[12]=PackageCHK>>8;
												 //CondWrWithDataOKMsg[13]=PackageCHK&0xff;
												 CondWrWithDataOKMsg[1] = NBReceiveData[1];
												 //NBMsgUp(&huart1,&huart3,(char *)CondWrWithDataOKMsg,300, 0);
												 MQTT_publish((uint8_t *)CondWrWithDataOKMsg, sizeof(CondWrWithDataOKMsg));
											 }
										 }
										 else
										 {
											 CondWrErrMsg[1] = NBReceiveData[1];
											 //NBMsgUp(&huart1,&huart3,(char *)CondWrErrMsg,300, 0);
											 MQTT_publish((uint8_t *)CondWrErrMsg, sizeof(CondWrErrMsg));
										 }
									   //sprintf(RepChar,"Period:0x%08x\r\n",CtrlData.ReportGap);
					           //HAL_UART_Transmit(&huart1,(uint8_t*)RepChar,19,500);

									   
								 }
								 #if 0
								 else 
								 {//incorrect Type
									 NBMsgUp(&huart1,&huart3,(char *)CondWrErrMsg,300);
								 }		
								#endif								 
								 HAL_UART_Transmit(&huart1,(uint8_t*)C2DLCond,strlen(C2DLCond),100);								 
								 break;		
							 case 0xC3://Timing control table
								 if(NBReceiveData[3]==0xD2)//0xD2 mark for Switch Controller
								 {//Command is OK
									   //switch controller timing table
									 int re = -1;
									 DEVICE_CONFIG tmpConfig = deviceConfig;
									 tmpConfig.switchPolicy.autoSwitch = NBReceiveData[5];
									 for(j=0;j<8;j++)
									 {
										 if((NBReceiveData[j*4+6] & 0x80) || (NBReceiveData[j*4+6] > 23) || (NBReceiveData[j*4+7] > 59))
										 {
											 continue;
										 }
										 tmpConfig.switchPolicy.autoSwitchMode[j] = TIMMING_CONTROL;
										 tmpConfig.switchPolicy.SwitchOnTimeHh[j] = NBReceiveData[j*4+6];
										 tmpConfig.switchPolicy.SwitchOnTimeMm[j] = NBReceiveData[j*4+7];
										 tmpConfig.switchPolicy.SwitchOFFTimeHh[j]= NBReceiveData[j*4+8];
										 tmpConfig.switchPolicy.SwitchOFFTimeMm[j]= NBReceiveData[j*4+9];
									 }
									 
									 re = writeDeviceConfig(&tmpConfig);
									 if(0 == re)
									 {
										 last_rcv_policy_tick = HAL_GetTick();
										 deviceConfig = tmpConfig;
										 setTimmingSwitchPolicySecond2();
										 CtrlData.EnableAutoSwitch=NBReceiveData[5];
										 for(j=0;j<8;j++)
									   {
											 /*
											 if(NBReceiveData[j*4+6] & 0x80)
											 {
												 continue;
											 }
											 CtrlData.SwitchOnTimeHh[j] = NBReceiveData[j*4+6];
											 CtrlData.SwitchOnTimeMm[j] = NBReceiveData[j*4+7];
											 CtrlData.SwitchOFFTimeHh[j]= NBReceiveData[j*4+8];
											 CtrlData.SwitchOFFTimeMm[j]= NBReceiveData[j*4+9];
											 */
											 CtrlData.SwitchOnTimeHh[j] = tmpConfig.switchPolicy.SwitchOnTimeHh[j];
											 CtrlData.SwitchOnTimeMm[j] = tmpConfig.switchPolicy.SwitchOnTimeMm[j];
											 CtrlData.SwitchOFFTimeHh[j]= tmpConfig.switchPolicy.SwitchOFFTimeHh[j];
											 CtrlData.SwitchOFFTimeMm[j]= tmpConfig.switchPolicy.SwitchOFFTimeMm[j];
										 }
										 
										 if(!NBReceiveData[4])
										 {//Need'nt reply data
											 NoDataResp[1] = NBReceiveData[1];
											 NoDataResp[2] = 0xC3;
											 //NBMsgUp(&huart1,&huart3,(char *)NoDataResp,300, 0);
											 MQTT_publish((uint8_t *)NoDataResp, sizeof(NoDataResp));
										 }
										 else
										 {//Need reply data
											 SetPolicyResp[6] = CtrlData.EnableAutoSwitch;
											 for(j = 0; j < SWITCH_NUM; j++)
										   {
											   SetPolicyResp[7+j*4]=CtrlData.SwitchOnTimeHh[j];
												 SetPolicyResp[8+j*4]=CtrlData.SwitchOnTimeMm[j];
												 SetPolicyResp[9+j*4]=CtrlData.SwitchOFFTimeHh[j];
												 SetPolicyResp[10+j*4]=CtrlData.SwitchOFFTimeMm[j];
												 SetPolicyResp[39+j] = deviceConfig.switchPolicy.autoSwitchMode[j];
										   }
											 SetPolicyResp[1] = NBReceiveData[1];
											 SetPolicyResp[2] = 0xC3;
									     //NBMsgUp(&huart1,&huart3,(char *)SetPolicyResp,300, 0);
											 MQTT_publish((uint8_t *)SetPolicyResp, sizeof(SetPolicyResp));
										 }
									 }
									 else
									 {
										 SCWrErrMsg[1] = NBReceiveData[1];
										 //NBMsgUp(&huart1,&huart3,(char *)SCWrErrMsg,300, 0);
										 MQTT_publish((uint8_t *)SCWrErrMsg, sizeof(SCWrErrMsg));
									 }
										 
								 }
								 #if 0
								 else 
								 {//incorrect Type
									 NBMsgUp(&huart1,&huart3,(char *)SCWrErrMsg,300);
								 }
								 #endif								 
								 HAL_UART_Transmit(&huart1,(uint8_t*)C3DLTimg,strlen(C3DLTimg),100);
								 break;									 
							 case 0xC4:	//switch control Command
								 if(NBReceiveData[3]==0xD2)
								 {//Command is OK
									   //switch command process
									   for(j=0;j<8;j++)
										 {
											  if(NBReceiveData[j+5]!=0xFF)
												{
													if(NBReceiveData[j+5])
													{
														CtrlData.SwitchState[j]=0x01;
														TurnOnPowerLineSwitch(j+1);//PowerLineNumber:1,2,3,4,5,6,7
													}
													else 
													{
														CtrlData.SwitchState[j]=0x00;
														TurnOFFPowerLineSwitch(j+1);//PowerLineNumber:1,2,3,4,5,6,7
													}
											  }
										 }
										 HAL_Delay(100);
										 GetAlarmState(&CtrlData);//Get Alarm state
										 if(!NBReceiveData[4])
										 {//Need'nt reply data
											 SWConWrOKMsg[1] = NBReceiveData[1];
											 //NBMsgUp(&huart1,&huart3,(char *)SWConWrOKMsg,300, 0);
											 MQTT_publish((uint8_t *)SWConWrOKMsg, sizeof(SWConWrOKMsg));
										 }
										 else
										 {//Need reply data
											 GetSwitchLineCurrent(&CtrlData);//get line current
											 for(int i = 0; i < SWITCH_NUM; i++)
											 {
												 SWConWrWithDataOKMsg[6+i] =  BoxGetSwitchState(i+1);
												 
												 SWConWrWithDataOKMsg[14+i*2]= (CtrlData.SwitchLineCurrent[i] >> 8)&0xff;//line current
												 SWConWrWithDataOKMsg[15+i*2] = CtrlData.SwitchLineCurrent[i]&0xff;
												 SWConWrWithDataOKMsg[30+i*2]= (CtrlData.SwitchLineVoltage[i] >> 8)&0xff;//line voltage
												 SWConWrWithDataOKMsg[31+i*2] = CtrlData.SwitchLineVoltage[i]&0xff;
											 }
								
											 SWConWrWithDataOKMsg[1] = NBReceiveData[1];
									     //NBMsgUp(&huart1,&huart3,(char *)SWConWrWithDataOKMsg,300, 0);
											 MQTT_publish((uint8_t *)SWConWrWithDataOKMsg, sizeof(SWConWrWithDataOKMsg));
										 }
								 }	
								 #if 0
								 else
								 {//SwitchCmdErrMsg
									 NBMsgUp(&huart1,&huart3,(char *)SWConWrErrMsg,300);
								 }		
								 #endif								 
							   HAL_UART_Transmit(&huart1,(uint8_t*)C4SDIM,strlen(C4SDIM),500);
								 break;
							 case 0xC6:
								 if(NBReceiveData[3]==0xD2)
								 {
										DEVICE_CONFIG deviceConfig;
										readDeviceConfig(&deviceConfig);
					
									 ReadCondMsg[4] = (deviceConfig.reportConfig.period >> 8 & 0xff);
									 ReadCondMsg[5] = (deviceConfig.reportConfig.period & 0xff);
									 ReadCondMsg[6] = (deviceConfig.reportConfig.powerDelta >> 8 & 0xff);
									 ReadCondMsg[7] = (deviceConfig.reportConfig.powerDelta & 0xff);
									 ReadCondMsg[8] = (deviceConfig.reportConfig.tempDelta >> 8 & 0xff);
									 ReadCondMsg[9] = (deviceConfig.reportConfig.tempDelta & 0xff);
									 ReadCondMsg[1] = NBReceiveData[1];
									 //NBMsgUp(&huart1,&huart3,(char *)ReadCondMsg,300, 0);
									 MQTT_publish((uint8_t *)ReadCondMsg, sizeof(ReadCondMsg));
								 }
								 break;
							 case 0xC7:
								 if(NBReceiveData[3]==0xD2)
								 {
									 #if 0
									 static char readTimmingSwitchMsg[]=
										{
											0x26,0x01,0xC7,0xD2,0x00,/*Length,SN,CMD,TYPE,EN*/
											0x18,0x00,0x07,0x00,/*1 18:00=ON 7:00=OFF  Time:BCD Code*/
											0x18,0x00,0x07,0x00,/*2 18:00=ON 7:00=OFF*/
											0x18,0x00,0x07,0x00,/*3 18:00=ON 7:00=OFF*/
											0x18,0x00,0x07,0x00,/*4 18:00=ON 7:00=OFF*/
											0x18,0x00,0x07,0x00,/*5 18:00=ON 7:00=OFF*/
											0x18,0x00,0x07,0x00,/*6 18:00=ON 7:00=OFF*/
											0x18,0x00,0x07,0x00,/*7 18:00=ON 7:00=OFF*/
											0x18,0x00,0x07,0x00,/*8 18:00=ON 7:00=OFF*/  
											0xB1,0x0E/*CRC*/
										};
									 #endif
									 DEVICE_CONFIG deviceConfig;
									 readDeviceConfig(&deviceConfig);
									 readTimmingSwitchMsg[4] = deviceConfig.switchPolicy.autoSwitch & 0xff;
									 for(int i = 0; i < SWITCH_NUM; i++)
									 {
										 readTimmingSwitchMsg[5+i] = deviceConfig.switchPolicy.SwitchOnTimeHh[i];
										 readTimmingSwitchMsg[6+i] = deviceConfig.switchPolicy.SwitchOnTimeMm[i];
										 readTimmingSwitchMsg[7+i] = deviceConfig.switchPolicy.SwitchOFFTimeHh[i];
										 readTimmingSwitchMsg[8+i] = deviceConfig.switchPolicy.SwitchOFFTimeMm[i];
									 }
									 readTimmingSwitchMsg[1] = NBReceiveData[1];
									 //NBMsgUp(&huart1,&huart3,(char *)readTimmingSwitchMsg,300, 0);
									 MQTT_publish((uint8_t *)readTimmingSwitchMsg, sizeof(readTimmingSwitchMsg));
								 }
								 break;
							 case 0xC9:
								 if(NBReceiveData[3]==0xD2)
								 {
									 char clon[4] = {0}, clat[4] = {0};
									 for(int i = 0; i < 4; i++)
									 {
										 clon[i] = NBReceiveData[8-i];
										 clat[i] = NBReceiveData[12-i];
									 }
									 float *plon = (float *)clon;
									 float *plat = (float *)clat;
									 float flon = *plon;
									 float flat = *plat;
									 char tt[32] = {0};
									 sprintf(tt, "location:%.6f %.6f\r\n", flon, flat);
									 HAL_UART_Transmit(&huart1, (uint8_t *)tt, strlen(tt), 100);
									 
									 int re = -1;
									 DEVICE_CONFIG tmpConfig = deviceConfig;
									 tmpConfig.location.longitude = flon;
									 tmpConfig.location.latitude = flat;
									 re = writeDeviceConfig(&tmpConfig);
									 uint8_t result = 0xE0;
									 if(0 == re)
									 {
											deviceConfig = tmpConfig;
											result = 0x00;
											setTimmingSwitchSunRiseSetSecond();
									 }
									 commonRespMsg[4] = 0x00;
									 commonRespMsg[5] = result;
									 commonRespMsg[2] = 0xC9;
									 //NBMsgUp(&huart1,&huart3,(char *)commonRespMsg,300, 0);
									 MQTT_publish((uint8_t *)commonRespMsg, sizeof(commonRespMsg));
								 }
								 break;
							case 0xCA:
								 if(NBReceiveData[3]==0xD2)
								 {
									 //uint8_t enableAutoSwitch = NBReceiveData[5];
									 //uint8_t sunRiseSet = NBReceiveData[6];
									 
									 int re = -1;
									 DEVICE_CONFIG tmpConfig = deviceConfig;
									 tmpConfig.switchPolicy.autoSwitch = NBReceiveData[5];
									 for(int i = 0; i < SWITCH_NUM; i++)
									 {
										 if(NBReceiveData[i*2+6] & 0x80)
										 {
											 continue;
										 }

										 tmpConfig.switchPolicy.autoSwitchMode[i] = SUNRISE_CONTROL;
										 tmpConfig.switchPolicy.sunSetEarlyOpenMinute[i] = NBReceiveData[i*2+6];
										 tmpConfig.switchPolicy.sunRiseDelayCloseMinute[i] = NBReceiveData[i*2+7];
									 }
									 
									 re = writeDeviceConfig(&tmpConfig);
									 uint8_t result = 0xE0;
									 if(0 == re)
									 {
										  last_rcv_policy_tick = HAL_GetTick();
											deviceConfig = tmpConfig;
											result = 0x00;
											setTimmingSwitchSunRiseSetSecond();
										 
											CtrlData.EnableAutoSwitch=NBReceiveData[5];
											for(j=0;j<8;j++)
											{
											 /*
											 if(NBReceiveData[j*4+6] & 0x80)
											 {
												 continue;
											 }
											 CtrlData.SwitchOnTimeHh[j] = NBReceiveData[j*4+6];
											 CtrlData.SwitchOnTimeMm[j] = NBReceiveData[j*4+7];
											 CtrlData.SwitchOFFTimeHh[j]= NBReceiveData[j*4+8];
											 CtrlData.SwitchOFFTimeMm[j]= NBReceiveData[j*4+9];
											 */
											 CtrlData.SwitchOnTimeHh[j] = tmpConfig.switchPolicy.SwitchOnTimeHh[j];
											 CtrlData.SwitchOnTimeMm[j] = tmpConfig.switchPolicy.SwitchOnTimeMm[j];
											 CtrlData.SwitchOFFTimeHh[j]= tmpConfig.switchPolicy.SwitchOFFTimeHh[j];
											 CtrlData.SwitchOFFTimeMm[j]= tmpConfig.switchPolicy.SwitchOFFTimeMm[j];
											}

											if(!NBReceiveData[4])
											{//Need'nt reply data
											 commonRespMsg[1] = NBReceiveData[1];
											 commonRespMsg[2] = 0xCA;
											 //NBMsgUp(&huart1,&huart3,(char *)commonRespMsg,300, 0);
											 MQTT_publish((uint8_t *)commonRespMsg, sizeof(commonRespMsg));
											}
											else
											{//Need reply data
											 SetPolicyResp[6] = CtrlData.EnableAutoSwitch;
											 for(j = 0; j < SWITCH_NUM; j++)
											 {
												 SetPolicyResp[7+j*4]=CtrlData.SwitchOnTimeHh[j];
												 SetPolicyResp[8+j*4]=CtrlData.SwitchOnTimeMm[j];
												 SetPolicyResp[9+j*4]=CtrlData.SwitchOFFTimeHh[j];
												 SetPolicyResp[10+j*4]=CtrlData.SwitchOFFTimeMm[j];
												 SetPolicyResp[39+j] = deviceConfig.switchPolicy.autoSwitchMode[j];
											 }
											 SetPolicyResp[1] = NBReceiveData[1];
											 SetPolicyResp[2] = 0xCA;
											 //NBMsgUp(&huart1,&huart3,(char *)SetPolicyResp,300, 0);
											 MQTT_publish((uint8_t *)SetPolicyResp, sizeof(SetPolicyResp));
											}
									 }
									 
								 }
								 break;
							 default:
							   HAL_UART_Transmit(&huart1,(uint8_t*)ComUND,strlen(ComUND),500);
								 break;
						 }//switch
					}//if
					else
					{ // increct CRC 
						HAL_UART_Transmit(&huart1,(uint8_t*)NBMsg.AtCmd,strlen((const char *)NBMsg.AtCmd),500);
					}
			  }
				//+QMTSTAT: 0,1
				else if(strstr((char *)NBMsg.AtCmd,"+QMTSTAT: "))
				{
						HAL_UART_Transmit(&huart1, (uint8_t *)NBMsg.AtCmd, NBMsg.CmdLen, 300);
						pch = strchr((char *)NBMsg.AtCmd, ',');
						uint8_t mqttStat = chartoint(pch+1);
						//Connection is closed or reset by peer.Execute AT+QMTOPEN command and reopen MQTT connection.
						if(1 == mqttStat)
						{
							MQTT_Init();
						}
						//Sending PINGREQ packet timed out or failed.Deactivate PDP first, and then active PDP and reopen MQTT connection.
						else if(2 == mqttStat)
						{
							NB_Attachment(&huart1,&huart3, 3);
							MQTT_Init();
						}
						else if(6 == mqttStat || 7 == mqttStat)
						{
							NB_Attachment(&huart1,&huart3, 3);
							MQTT_Init();
						}
				}
				else
				{
					HAL_UART_Transmit(&huart1,(uint8_t*)NBMsg.AtCmd,strlen((const char *)NBMsg.AtCmd),500);
				}
			  NBMsg.CmdLen=0;
		  }
    }
		/*-----------------------End of NB Moniter-----------------------------*/
		
		/*------------------------Moniter USART1(DBG)--------------------------*/
		memset(ConMsg.AtCmd,0,sizeof(ConMsg.AtCmd));//clear array 
		ConMsg.CmdLen=0;
		char str[64] = {0};
		if(GetDBGCmd(&huart1,&ConMsg)== HAL_OK)
		{
			char *tmpIndex = strstr((char *)ConMsg.AtCmd, "=");
			if(NULL != tmpIndex)
			{
				int start = tmpIndex - (char *)&ConMsg.AtCmd[0];
				for(int i = 0; i < start; i++)
				{
					uint8_t tmpChar = ConMsg.AtCmd[i];
					ConMsg.AtCmd[i] = ((tmpChar>=97)&&(tmpChar<=122))?tmpChar-32:tmpChar;
				}
			}
			CmdNumber=LookUpCmd(ConMsg.AtCmd,ConMsg.CmdLen);//Search Command Number 
			if(CmdNumber!=ATCOMMAND) 
				HAL_UART_Transmit(&huart1, (uint8_t*)ConMsg.AtCmd,ConMsg.CmdLen,500);
			switch(CmdNumber)
			{
				#if 1
					case 0:/*SWITCH Control:SWITCHON+1|2|3|4|5|6|7|8<LF>*/
						if ((ConMsg.AtCmd[10]==0x0d)&&((ConMsg.AtCmd[9]>0x30)&&(ConMsg.AtCmd[9]<0x39)))
						{//format ok
							if (TurnOnPowerLineSwitch(DecNum(ConMsg.AtCmd[9])))
							{
								CtrlData.SwitchState[DecNum(ConMsg.AtCmd[9])-1]=0x01;//remember the switch state
								GetAlarmState(&CtrlData);//Get Alarm state
							  HAL_UART_Transmit(&huart1,(uint8_t*)SWITCH_ON,strlen(SWITCH_ON),200);
							}
							else
							  HAL_UART_Transmit(&huart1,(uint8_t*)INVLID,strlen(INVLID),200);
						}
						else
						{//incorrect format
							HAL_UART_Transmit(&huart1,(uint8_t*)INCMDFORMAT,strlen(INCMDFORMAT),500);
						}
						break;
					case 1:/*SWITCH Control:SWITCHOFF+1|2|3|4|5|6|7|8<LF>*/
						if ((ConMsg.AtCmd[11]==0x0d)&&((ConMsg.AtCmd[10]>0x30)&&(ConMsg.AtCmd[10]<0x39)))
						{//format ok
							if (TurnOFFPowerLineSwitch(DecNum(ConMsg.AtCmd[10])))
							{
								CtrlData.SwitchState[DecNum(ConMsg.AtCmd[10])-1]=0x00;//remember the switch state
								GetAlarmState(&CtrlData);//Get Alarm state
							  HAL_UART_Transmit(&huart1,(uint8_t*)SWITCH_OFF,strlen(SWITCH_OFF),200);
							}
							else
							  HAL_UART_Transmit(&huart1,(uint8_t*)INVLID,strlen(INVLID),200);
						}
						else
						{//incorrect format
						  HAL_UART_Transmit(&huart1,(uint8_t*)INCMDFORMAT,strlen(INCMDFORMAT),500);
						}
						break;
					case 2:/*NB POWERON Command:NBON*/
						NB_ON();
					  HAL_UART_Transmit(&huart1,(uint8_t*)NBON,COUNTOF(NBON),500);
						break;
					case 3:/*NB POWERON Command:NBOFF*/
						NB_OFF();
					  HAL_UART_Transmit(&huart1,(uint8_t*)NBOFF,COUNTOF(NBOFF),500);
						break;
					case 4:/*NB RESET Cmmmand:NBRESET*/
						NB_RESETB();
					  HAL_UART_Transmit(&huart1,(uint8_t*)NBRST,COUNTOF(NBRST),500);
						break;
					case 5:/*AT Command format:AT....*/
						//HAL_UART_Transmit(&huart1, (uint8_t*)ConMsg.AtCmd,ConMsg.CmdLen,500);
						if((ConMsg.AtCmd[ConMsg.CmdLen-1]==0x0d)||(ConMsg.AtCmd[ConMsg.CmdLen-1]==0x0a))
						{
							ConMsg.AtCmd[ConMsg.CmdLen-1]=0;
							ConMsg.CmdLen--;
							if((ConMsg.AtCmd[ConMsg.CmdLen-1]==0x0d)||(ConMsg.AtCmd[ConMsg.CmdLen-1]==0x0a))
							{
								ConMsg.AtCmd[ConMsg.CmdLen-1]=0;
								ConMsg.CmdLen--;
							}
						}
						NB_ATCMD(&huart1,&huart3,&ConMsg,&ReStr,300);
						ConMsg.CmdLen=0;	
						ReStr.CmdLen=0;	
						break;
						#endif
					case 6:/*VER Command:VER*/
						HAL_UART_Transmit(&huart1,(uint8_t*)HardWareVer,strlen((char*)HardWareVer),200);
					  HAL_UART_Transmit(&huart1,(uint8_t*)SoftWareVer,strlen((char*)SoftWareVer),200);
						MQTT_publish((uint8_t *)testData, sizeof(testData));
						break;
					case 7:/*Time Cmmmand*/
						#if 0
						RTC_TimeShow(aShowTime);
					  HAL_UART_Transmit(&huart1,(uint8_t*)aShowTime,strlen((char*)aShowTime),500);
						#else
						RTC_Get();
						sprintf(str, "%04d/%02d/%02d %02d:%02d:%02d\r\n",calendar.w_year, calendar.w_month, calendar.w_date,
																										calendar.hour, calendar.min, calendar.sec);
						HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),200);
						double longt = deviceConfig.location.longitude,lat = deviceConfig.location.latitude;
						RTC_TimeTypeDef timeRise, timeSet;
						if(0 == getSunrise(longt, lat, calendar, &timeRise) &&
										0 == getSunset(longt, lat, calendar, &timeSet))
						{
							sprintf(str, "rise:%02u:%02u set:%02u:%02u\r\n",timeRise.Hours, timeRise.Minutes, 
																															timeSet.Hours, timeSet.Minutes);
							HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),200);
						}
						
						sprintf(str, "close count: %u\r\n", TimmingSwitchPolicySecond.closePolicy[0].second);
						HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),200);
						_calendar_obj tmpCalen;
						RTC_Get_date_time_from_count(TimmingSwitchPolicySecond.closePolicy[0].second, &tmpCalen);
						sprintf(str, "close count to datetime:%04d/%02d/%02d %02d:%02d:%02d\r\n",tmpCalen.w_year, tmpCalen.w_month, tmpCalen.w_date,
																										tmpCalen.hour, tmpCalen.min, tmpCalen.sec);
						HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),200);
						
						sprintf(str, "now count: %u\r\n", RTC_ReadTimeCounter(&hrtc));
						HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),200);
					//HAL_UART_Transmit(&huart1,(uint8_t*)"",1,200);
					#endif
						break;
					case 8://PUBLISH
						MQTT_publish((uint8_t *)"6666", 2);
						break;
					case 9:
						break;
					case 10:/*Command format:FLASH*/
						printDeviceConfig(deviceConfig);
						#if 0
						turnOffAllLineSwitch();
						for(int i = 0; i < SWITCH_NUM; i++)
						{
							reportControlSwitchMsg[4+i] = 0x00;
							CtrlData.SwitchState[i]=0x00;
						}
						NBMsgUp(&huart1, &huart3, reportControlSwitchMsg, 300);
						#endif	
						break;
					case 11:/*Command format:NOW*/
						//RTC_TimeShow(aShowTime);
						//HAL_UART_Transmit(&huart1,(uint8_t*)aShowTime,strlen(aShowTime),500);
						sysc_time_from_network();
						sprintf(str, "now count: %u\r\n", RTC_ReadTimeCounter(&hrtc));
						HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),200);
						
						sprintf(str, "rtc bkp: 0x%x\r\n", HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR1));
						HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),200);
						
						#if 0
						turnOnAllLineSwitch();
						for(int i = 0; i < SWITCH_NUM; i++)
						{
							reportControlSwitchMsg[4+i] = 0x01;
							CtrlData.SwitchState[i]=0x01;
						}
						NBMsgUp(&huart1, &huart3, reportControlSwitchMsg, 300);
						#endif
						break;
					case 12:/*Command format:REREG*/
					{
						NB_Attachment(&huart1,&huart3, 1);
						MQTT_Init();
						//extern void SoftReset(void);
						//SoftReset();
						break;
					}
					case 13:/*Command format:COUNTER=*/
						if (IsDec(ConMsg.AtCmd[8])&&IsDec(ConMsg.AtCmd[9]) &&
							IsDec(ConMsg.AtCmd[10])&&IsDec(ConMsg.AtCmd[11]) && 
							IsDec(ConMsg.AtCmd[12])&&IsDec(ConMsg.AtCmd[13]))
						{
							stime.Hours = DecNum(ConMsg.AtCmd[8])*10+DecNum(ConMsg.AtCmd[9]);
							stime.Minutes = DecNum(ConMsg.AtCmd[10])*10+DecNum(ConMsg.AtCmd[11]);
							stime.Seconds = DecNum(ConMsg.AtCmd[12])*10+DecNum(ConMsg.AtCmd[13]);
							#if 1
							RTC_Get();
							_calendar_obj tmpCalendar =calendar;
							tmpCalendar.hour = stime.Hours;
							tmpCalendar.min = stime.Minutes;
							tmpCalendar.sec = stime.Seconds;
							if(HAL_OK != RTC_SetDateTime(&hrtc, &tmpCalendar))
							{
								HAL_UART_Transmit(&huart1,(uint8_t*)"RTC_Set error\r\n",15,500);
							}
														
							#else
							if(HAL_RTC_SetTime(&hrtc,&stime,RTC_FORMAT_BIN) != HAL_OK)
							{
								/* Initialization Error */
								Error_Handler(); 
							}	
							#endif
						}
						else
						{
							HAL_UART_Transmit(&huart1,(uint8_t*)InCMD,strlen(InCMD),500);
						}
						
						break;
					default:
						break;
			}
		}
		/*-----------------------End of DBG Moniter-----------------------------*/
		memset(ConMsg.AtCmd,0,sizeof(ConMsg.AtCmd));//clear array 
		ConMsg.CmdLen=0;
		if(GetDBGCmd(&huart4,&ConMsg)== HAL_OK)
		{
			HAL_UART_Transmit(&huart1, (uint8_t*)ConMsg.AtCmd,ConMsg.CmdLen,500);
		}
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
	
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  //sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  HAL_ADCEx_Calibration_Start(&hadc1);
	
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */
	
  //RTC_TimeTypeDef sTime = {0};
  //RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_SECOND;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

	rtc_init_user();

	#if 0
  /* USER CODE BEGIN Check_RTC_BKUP */
  if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) == 0X505A)
	{
		return;
	}
  /* USER CODE END Check_RTC_BKUP */

	
  /** Initialize RTC and set the Time and Date 
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
	
  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 0x1;
  DateToUpdate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
	
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0X505A);
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */
	#endif
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
	huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 9600;
	huart5.Init.WordLength = UART_WORDLENGTH_9B;
  huart5.Init.StopBits = UART_STOPBITS_1;
	huart5.Init.Parity = UART_PARITY_EVEN;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, L4_RLY_Pin|L5_RLY_Pin|L6_RLY_Pin|L1_RLY_Pin 
                          |L2_RLY_Pin|L3_RLY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NB_POWEREN_GPIO_Port, NB_POWEREN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, NB_RESETB_Pin|NB_PSM_EINT_Pin|NB_PWRKEY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, L1_INDICATION_Pin|L2_INDICATION_Pin|L3_INDICATION_Pin|L4_INDICATION_Pin 
                          |L5_INDICATION_Pin|L6_INDICATION_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : L4_RLY_Pin L5_RLY_Pin L6_RLY_Pin L1_RLY_Pin 
                           L2_RLY_Pin L3_RLY_Pin */
  GPIO_InitStruct.Pin = L4_RLY_Pin|L5_RLY_Pin|L6_RLY_Pin|L1_RLY_Pin 
                          |L2_RLY_Pin|L3_RLY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : NB_POWEREN_Pin NB_RESETB_Pin NB_PSM_EINT_Pin NB_PWRKEY_Pin */
  GPIO_InitStruct.Pin = NB_POWEREN_Pin|NB_RESETB_Pin|NB_PSM_EINT_Pin|NB_PWRKEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : K6_Pin K5_Pin k4_Pin */
  GPIO_InitStruct.Pin = K6_Pin|K5_Pin|K4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : K3_Pin K1_Pin K2_Pin */
  GPIO_InitStruct.Pin = K3_Pin|K1_Pin|K2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ALARM_Pin L1_INDICATION_Pin L2_INDICATION_Pin L3_INDICATION_Pin 
                           L4_INDICATION_Pin L5_INDICATION_Pin L6_INDICATION_Pin */
  GPIO_InitStruct.Pin = ALARM_Pin|L1_INDICATION_Pin|L2_INDICATION_Pin|L3_INDICATION_Pin 
                          |L4_INDICATION_Pin|L5_INDICATION_Pin|L6_INDICATION_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = PKEY_Pin;
	//GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(PKEY_GPIO_Port, &GPIO_InitStruct);

	//HAL_NVIC_SetPriority(EXTI9_5_IRQn, 3, 0);
	//HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void MX_TIMx_Init(void)
{
	//uwPrescalerValue = (uint32_t)(SystemCoreClock / 10000) - 1;

  /* Set TIMx instance */
  TimHandle.Instance = TIMx;

  /* Initialize TIMx peripheral as follows:
       + Period = 10000 - 1
       + Prescaler = (SystemCoreClock/10000) - 1
       + ClockDivision = 0
       + Counter direction = Up
  */
  //TimHandle.Init.Period            = 10 - 1;
	TimHandle.Init.Period            = TIMx_Period;
  //TimHandle.Init.Prescaler         = (uint32_t)(SystemCoreClock / 10000) - 1;
	TimHandle.Init.Prescaler         = TIMx_Prescaler;
  TimHandle.Init.ClockDivision     = 0;
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandle.Init.RepetitionCounter = 0;
  TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if (HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
}
/* USER CODE BEGIN 4 */
/**
  * @brief  Turns MIC29302AWU LDO ON.
  * @param  none
  */
static void MIC29302AWU_ON(void)
{
	
	HAL_GPIO_WritePin(NB_POWEREN_GPIO_Port, NB_POWEREN_Pin, GPIO_PIN_RESET); 
}
/**
  * @brief  Turns MIC29302AWU LDO Off.
  * @param  none
  */
static void MIC29302AWU_OFF(void)
{
	HAL_GPIO_WritePin(NB_POWEREN_GPIO_Port, NB_POWEREN_Pin, GPIO_PIN_SET); 
}

static void NB_ON(void)
{
	MIC29302AWU_ON();
}
static void NB_OFF(void)
{
	MIC29302AWU_OFF();
}
/************************************************************
 * @brief  Turns Line Switch ON.                            *
 * @param  LINE:Specifies the Line to be turn ON.           *
 *   This parameter can be one of following parameters:     *
 * @arg L1,L2,L3,L4,L5,L6                                   *
 ************************************************************/
static void LineSwitch_ON(uint16_t LINE)
{
	switch(LINE)
	{
		case L1:
			HAL_GPIO_WritePin(L1_PORT, L1, GPIO_PIN_SET);
			break;
		case L2:
			HAL_GPIO_WritePin(L2_PORT, L2, GPIO_PIN_SET);
			break;
		case L3:
			HAL_GPIO_WritePin(L3_PORT, L3, GPIO_PIN_SET);
			break;
		case L4:
			HAL_GPIO_WritePin(L4_PORT, L4, GPIO_PIN_SET);
			break;
		case L5:
			HAL_GPIO_WritePin(L5_PORT, L5, GPIO_PIN_SET);
			break;
		case L6:	
		  HAL_GPIO_WritePin(L6_PORT, L6, GPIO_PIN_SET);
		  break;
		default:
			break;
	}
}
/**
  * @brief  Turns Line Switch OFF.
  * @param  LINE:Specifies the Line to be turn OFF. 
  *   This parameter can be one of following parameters:
  * @arg L1,L2,L3,L4,L5,L6
  */
static void LineSwitch_OFF(uint16_t LINE)
{
	switch(LINE)
	{
		case L1:
			HAL_GPIO_WritePin(L1_PORT, L1, GPIO_PIN_RESET);
			break;
		case L2:
			HAL_GPIO_WritePin(L2_PORT, L2, GPIO_PIN_RESET);
			break;
		case L3:
			HAL_GPIO_WritePin(L3_PORT, L3, GPIO_PIN_RESET);
			break;
		case L4:
			HAL_GPIO_WritePin(L4_PORT, L4, GPIO_PIN_RESET);
			break;
		case L5:
			HAL_GPIO_WritePin(L5_PORT, L5, GPIO_PIN_RESET);
			break;
		case L6:
		  HAL_GPIO_WritePin(L6_PORT, L6, GPIO_PIN_RESET);
		  break;	
		default:
			break;
	}
}
/**
  * @brief  Turns Line LED ON.
  * @param  LINE:Specifies the LED to be turn ON. 
  *   This parameter can be one of following parameters:
  * @arg LED1,LED2,LED3,LED4,LED5,LED6,ALARM
  */
static void BoxLED_ON(uint16_t LED)
{
	switch(LED)
	{
		case LED1:
			HAL_GPIO_WritePin(LED1_PORT, LED1, GPIO_PIN_SET);
			break;
		case LED2:
			HAL_GPIO_WritePin(LED2_PORT, LED2, GPIO_PIN_SET);
			break;
		case LED3:
			HAL_GPIO_WritePin(LED3_PORT, LED3, GPIO_PIN_SET);
			break;
		case LED4:
			HAL_GPIO_WritePin(LED4_PORT, LED4, GPIO_PIN_SET);
			break;
		case LED5:
			HAL_GPIO_WritePin(LED5_PORT, LED5, GPIO_PIN_SET);
			break;
		case LED6:	
		  HAL_GPIO_WritePin(LED6_PORT, LED6, GPIO_PIN_SET);
		  break;
		case ALARM:	
		  HAL_GPIO_WritePin(ALARM_PORT, ALARM, GPIO_PIN_RESET);
		  break;
		default:
			break;
	}
}
/**
  * @brief  Turns LED Switch OFF.
  * @param  LINE:Specifies the LED to be turn OFF. 
  *   This parameter can be one of following parameters:
  * @arg LED1,LED2,LED3,LED4,LED5,LED6,ALARM
  */
static void BoxLED_OFF(uint16_t LED)
{
	switch(LED)
	{
		case LED1:
			HAL_GPIO_WritePin(LED1_PORT, LED1, GPIO_PIN_RESET);
			break;
		case LED2:
			HAL_GPIO_WritePin(LED2_PORT, LED2, GPIO_PIN_RESET);
			break;
		case LED3:
			HAL_GPIO_WritePin(LED3_PORT, LED3, GPIO_PIN_RESET);
			break;
		case LED4:
			HAL_GPIO_WritePin(LED4_PORT, LED4, GPIO_PIN_RESET);
			break;
		case LED5:
			HAL_GPIO_WritePin(LED5_PORT, LED5, GPIO_PIN_RESET);
			break;
		case LED6:
		  HAL_GPIO_WritePin(LED6_PORT, LED6, GPIO_PIN_RESET);
		  break;
		case ALARM:
		  HAL_GPIO_WritePin(ALARM_PORT, ALARM, GPIO_PIN_SET);		
		  break;	
		default:
			break;
	}
}

/**
  * @brief  Get the State of switch.
  * @param  Swt:Specifies the Switch to be get state. 
  *   This parameter can be one of following parameters:
  * @arg K1,K2,K3,K4,K5,K6
  */
GPIO_PinState BoxGetSwitchState(uint16_t Swt)
{
  switch(Swt)
	{
		case K1:
			return (HAL_GPIO_ReadPin(K1_PORT, K1_Pin));
		case K2:
			return (HAL_GPIO_ReadPin(K2_PORT, K2_Pin));
		case K3:
			return (HAL_GPIO_ReadPin(K3_PORT, K3_Pin));
		case K4:
			return (HAL_GPIO_ReadPin(K4_PORT, K4_Pin));
		case K5:
			return (HAL_GPIO_ReadPin(K5_PORT, K5_Pin));
		case K6:
		  return (HAL_GPIO_ReadPin(K6_PORT, K6_Pin));
		default:
			return GPIO_PIN_RESET;
	}
}
/* USER CODE END 4 */
/**********************************
 * @brief  Send PSMEINT pluse.    *
 * @param  none                   *
 **********************************/
static void NB_PSMEINT(void)
{
	HAL_GPIO_WritePin(NB_PSM_EINT_GPIO_Port, NB_PSM_EINT_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);//delay 10ms
	HAL_GPIO_WritePin(NB_PSM_EINT_GPIO_Port, NB_PSM_EINT_Pin, GPIO_PIN_SET);
	HAL_Delay(10);//delay 5ms
	HAL_GPIO_WritePin(NB_PSM_EINT_GPIO_Port, NB_PSM_EINT_Pin, GPIO_PIN_RESET);
}
static void NB_PsmEintLOW(void)
{
	/*pull PSM_EINT low*/
  HAL_GPIO_WritePin(NB_PSM_EINT_GPIO_Port, NB_PSM_EINT_Pin, GPIO_PIN_SET);
}
static void NB_PsmEintHIGH(void)
{
	/*pull PSM_EINT high*/
	HAL_GPIO_WritePin(NB_PSM_EINT_GPIO_Port, NB_PSM_EINT_Pin,GPIO_PIN_RESET);
}
/*********************************
 * @brief  Send RESET pluse.     *
 * @param  none                  *
 *********************************/
static void NB_RESETB(void)
{
	HAL_GPIO_WritePin(NB_PSM_EINT_GPIO_Port, NB_RESETB_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);//delay 10ms
	HAL_GPIO_WritePin(NB_PSM_EINT_GPIO_Port, NB_RESETB_Pin, GPIO_PIN_SET);
	HAL_Delay(100);//delay 100ms
	HAL_GPIO_WritePin(NB_PSM_EINT_GPIO_Port, NB_RESETB_Pin, GPIO_PIN_RESET);
}
/**
  * @brief  Send PWRKEY pluse.
  * @param  none
  */
static void NB_PKEY(void)
{
  HAL_GPIO_WritePin(NB_PWRKEY_GPIO_Port, NB_PWRKEY_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);//delay 10ms
	HAL_GPIO_WritePin(NB_PWRKEY_GPIO_Port, NB_PWRKEY_Pin, GPIO_PIN_SET);
	HAL_Delay(800);//delay 800ms
	HAL_GPIO_WritePin(NB_PWRKEY_GPIO_Port, NB_PWRKEY_Pin, GPIO_PIN_RESET);
}
static void NB_StartUp(void)
{
	NB_PKEY();
}

/****************************************************************/
/* @function: String copy                                       */
/* @para:DestString,SourceStrine,number of Bytes                */
/****************************************************************/
void StringCpy(uint8_t* String1,uint8_t* String2,uint16_t Stringlength)
{
	while(Stringlength--)
	{
		*String1=*String2;
		String1++;
		String2++;
	}
}
/**********************************************************************************
 * @brief  Get debug command from UART1.                                          *
 * @param  huart: Pointer to a UART_HandleTypeDef structure that contains         *
 *                the configuration information for the specified UART module.    *
 *         ConCmd:The pointer of Debug Command structure.                         *
 **********************************************************************************/
HAL_StatusTypeDef GetDBGCmd(UART_HandleTypeDef *huart,CmdStruct *ConCmd)
{
	uint32_t tickstart = 0U;
	uint16_t NumOfChar;
	char temp;
	const uint8_t TimeOut[]="\nTime out!\n";
	const uint8_t TooLong[]="\nToo long Command!\n";	
	
	uint8_t Cnt=0;
	memset(ConCmd->AtCmd,0,sizeof(ConCmd->AtCmd));//clear array 
	tickstart = HAL_GetTick();
	for(;;)
	{
		if(__HAL_UART_GET_FLAG (huart, UART_FLAG_RXNE) != RESET)
		{
			  temp=(uint8_t)(huart->Instance->DR & 0xff);
			  //ConCmd->AtCmd[Cnt]=((temp>=97)&&(temp<=122))?temp-32:temp;
				ConCmd->AtCmd[Cnt]=temp;
				__HAL_UART_CLEAR_FLAG (huart, UART_FLAG_RXNE);
				if (Cnt>254) 
				{
					StringCpy(ConCmd->AtCmd,(uint8_t *)TooLong,19);
					NumOfChar=sizeof(TooLong);
					break;
				}
				else
					Cnt++;
				tickstart = HAL_GetTick();
		}
		else
		{
			if (Cnt==0) return HAL_ERROR;
			else if ((HAL_GetTick()-tickstart)>20)
			{
				if(Cnt>2)
				{
					NumOfChar=Cnt-1;
					break;
				}	
				else
				{	
					StringCpy(ConCmd->AtCmd,(uint8_t *)TimeOut,11);
					NumOfChar=strlen((const char *)TimeOut);
					break;
				}//else
			}//else if()
		}//if()
	}//for(;;)
	ConCmd->CmdLen=NumOfChar;
	return HAL_OK;
}

/**********************************************************************************
 * @brief  Get debug command from UART3.                                          *
 * @param  huart: Pointer to a UART_HandleTypeDef structure that contains         *
 *                the configuration information for the specified UART module.    *
 *         ConCmd:The pointer of Debug Command structure.                         *
 **********************************************************************************/
HAL_StatusTypeDef GetNBMsg(UART_HandleTypeDef *huart,CmdStruct *NBMsg)
{
	uint32_t tickstart = 0U;
	const char TooLong[]="\nToo long Repond!\n";
	uint8_t Cnt=0;
	
	memset(NBMsg->AtCmd,0,sizeof(NBMsg->AtCmd));//clear array 
	tickstart = HAL_GetTick();
	for(;;)
	{
		if(__HAL_UART_GET_FLAG (huart, UART_FLAG_RXNE) != RESET)
		{
			  NBMsg->AtCmd[Cnt]=(uint8_t)(huart->Instance->DR & 0xff);
				__HAL_UART_CLEAR_FLAG (huart, UART_FLAG_RXNE);
				if (Cnt>254) 
				{
					StringCpy(NBMsg->AtCmd,(uint8_t *)TooLong,18);
					Cnt=18;//force to end of string
					break;
				}
				else
					Cnt++;
				tickstart = HAL_GetTick();
		}
		else
		{
			if (Cnt==0) return HAL_ERROR;
			else if ((HAL_GetTick()-tickstart)>20)
			{
				break;
			}//else if()
		}//if()
	}//for(;;)
	NBMsg->CmdLen=Cnt-1;
	return HAL_OK;
}

/***************************************************************
  * @brief  match Command.                                     *
  * @param  pBuffer: received command line.                    *
  * @param  Cmdlength: Command length                          *
  * @retval Match: CommandID(0-9)                              *
  *         Mismatch:-1;                                       *
****************************************************************/
int8_t LookUpCmd(uint8_t* pBuffer,uint16_t Cmdlength)
{ 
	int8_t ind;
	uint16_t KeyLen;

	for(ind=0;ind<NUMOFCMD;ind++)
	{ 
		if(CmdSize[ind]<=Cmdlength)
		{//String length match
			KeyLen=0;
			while(KeyLen<CmdSize[ind])
			{
				if(pBuffer[KeyLen] != CmdString[ind][KeyLen]) break;//No match this command
				//pBuffer++;
				KeyLen++;
			}
			if(KeyLen==CmdSize[ind]) return ind;
	  }
  }
  return NOFOUND;//No match command
}

void USART_SEND_HEXSTRING(UART_HandleTypeDef *DBGhuart,UART_HandleTypeDef *huart,CmdStruct *TxCmd,CmdStruct *RetString,uint32_t Timeout)//AT Command
{
  const char Toolong[]="\nToo long Repond!\n";
	const char TimeOutString[]="\nRespond Time out!\n";
	//const char LFCR[2]={0x0d,0x0a};
	volatile uint16_t NumOfChar;	
	volatile uint16_t Cnt;
	uint32_t tickstart = 0U;
	
	HAL_StatusTypeDef status = HAL_UART_Transmit(huart,(uint8_t*)TxCmd->AtCmd,TxCmd->CmdLen,200);
	//HAL_UART_Transmit(huart,(uint8_t*)LFCR,2,50);
	
	Cnt=0;
	tickstart = HAL_GetTick();
	memset(RetString->AtCmd,0,sizeof(RetString->AtCmd));//clear array 
	for(;;)
	{
		if(__HAL_UART_GET_FLAG(huart,UART_FLAG_RXNE) != RESET)
		{
			  RetString->AtCmd[Cnt]=(uint8_t)(huart->Instance->DR & 0xff);
				__HAL_UART_CLEAR_FLAG (huart, UART_FLAG_RXNE);
				if (Cnt>254) 
				{
					memset(RetString->AtCmd,0,sizeof(RetString->AtCmd));//clear array 
					StringCpy(RetString->AtCmd,(uint8_t *)Toolong,sizeof(Toolong));
					NumOfChar=sizeof(Toolong);//force to end of string
					break;
				}
				else
					Cnt++;
				//tickstart = HAL_GetTick();
		}
		else
		{
			if ((HAL_GetTick()-tickstart)>Timeout)
			{
			  if (Cnt<2) 
			  {
					
					memset(RetString->AtCmd,0,sizeof(RetString->AtCmd));//clear array 
					StringCpy(RetString->AtCmd,(uint8_t *)TimeOutString,sizeof(TimeOutString));
		      NumOfChar=sizeof(TimeOutString);
					break;
				}
				else
				{
				  NumOfChar=Cnt;
				  break;
			  }//if (Cnt<2)	
			}
		}//if()
	}//for(;;)
	//RetString->AtCmd[NumOfChar]=0;//end of string
	RetString->CmdLen=NumOfChar;
	//HAL_UART_Transmit(DBGhuart,(uint8_t*)RetString->AtCmd,RetString->CmdLen,500);
}

/**********************************************************************************
 * @brief  Send AT command to NB(UART3).                                          *
 * @param  DBGhuart:Pointer to a UART_HandleTypeDef structure that contains       *
 *                  the configuration information for the NB UART module.         *
 * @param  NBhuart:Pointer to a UART_HandleTypeDef structure that contains        *
 *                 the configuration information for the DBG UART module.         *
 *         TxCmd:The pointer of AT Command structure.                             *
 *         RetString:The pointer of NB respond structure.                         *
 **********************************************************************************/
void NB_ATCMD(UART_HandleTypeDef *DBGhuart,UART_HandleTypeDef *NBhuart,CmdStruct *TxCmd,CmdStruct *RetString,uint32_t Timeout)//AT Command
{
  const char Toolong[]="\nToo long Repond!\n";
	const char TimeOutString[]="\nRespond Time out!\n";
	const char LFCR[2]={0x0d,0x0a};
	volatile uint16_t NumOfChar;	
	volatile uint16_t Cnt;
	uint32_t tickstart = 0U;
	
	
	Cnt=0;
	tickstart = HAL_GetTick();
	memset(RetString->AtCmd,0,sizeof(RetString->AtCmd));//clear array 
	HAL_UART_Transmit(DBGhuart,(uint8_t*)TxCmd->AtCmd,TxCmd->CmdLen,100);//display AT Command	
	HAL_UART_Transmit(DBGhuart,(uint8_t*)LFCR,1,50);
	/********transmit AT command to NB:**************/
	HAL_UART_Transmit(NBhuart,(uint8_t*)TxCmd->AtCmd,TxCmd->CmdLen,200);//send ATCMD to NB
	HAL_UART_Transmit(NBhuart,(uint8_t*)LFCR,1,50);
	
	for(;;)
	{
		if(__HAL_UART_GET_FLAG(NBhuart,UART_FLAG_RXNE) != RESET)
		{
			  RetString->AtCmd[Cnt]=(uint8_t)(NBhuart->Instance->DR & 0xff);
				__HAL_UART_CLEAR_FLAG (NBhuart, UART_FLAG_RXNE);
				if (Cnt>254) 
				{
					memset(RetString->AtCmd,0,sizeof(RetString->AtCmd));//clear array 
					StringCpy(RetString->AtCmd,(uint8_t *)Toolong,sizeof(Toolong));
					NumOfChar=sizeof(Toolong);//force to end of string
					break;
				}
				else
					Cnt++;
				//tickstart = HAL_GetTick();
		}
		else
		{
			if ((HAL_GetTick()-tickstart)>Timeout)
			{
			  if (Cnt<2) 
			  {
					memset(RetString->AtCmd,0,sizeof(RetString->AtCmd));//clear array 
					StringCpy(RetString->AtCmd,(uint8_t *)TimeOutString,sizeof(TimeOutString));
		      NumOfChar=sizeof(TimeOutString);
					break;
				}
				else
				{
				  NumOfChar=Cnt-1;
				  break;
			  }//if (Cnt<2)	
			}
		}//if()
	}//for(;;)
	//RetString->AtCmd[NumOfChar]=0;//end of string
	RetString->CmdLen=NumOfChar;
	HAL_UART_Transmit(DBGhuart,(uint8_t*)RetString->AtCmd,RetString->CmdLen,500);
	//DisplayStringHex(USARTy,RetString->AtCmd,RetString->CmdLen);
}

void NB_ATCMD_NO_PRINT(UART_HandleTypeDef *DBGhuart,UART_HandleTypeDef *NBhuart,CmdStruct *TxCmd,CmdStruct *RetString,uint32_t Timeout)//AT Command
{
  const char Toolong[]="\nToo long Repond!\n";
	const char TimeOutString[]="\nRespond Time out!\n";
	const char LFCR[2]={0x0d,0x0a};
	volatile uint16_t NumOfChar;	
	volatile uint16_t Cnt;
	uint32_t tickstart = 0U;
	
	
	Cnt=0;
	tickstart = HAL_GetTick();
	memset(RetString->AtCmd,0,sizeof(RetString->AtCmd));//clear array 
	//HAL_UART_Transmit(DBGhuart,(uint8_t*)TxCmd->AtCmd,TxCmd->CmdLen,500);//display AT Command	
	//HAL_UART_Transmit(DBGhuart,(uint8_t*)LFCR,1,50);
	/********transmit AT command to NB:**************/
	HAL_UART_Transmit(NBhuart,(uint8_t*)TxCmd->AtCmd,TxCmd->CmdLen,200);//send ATCMD to NB
	HAL_UART_Transmit(NBhuart,(uint8_t*)LFCR,1,50);
	
	for(;;)
	{
		if(__HAL_UART_GET_FLAG(NBhuart,UART_FLAG_RXNE) != RESET)
		{
			  RetString->AtCmd[Cnt]=(uint8_t)(NBhuart->Instance->DR & 0xff);
				__HAL_UART_CLEAR_FLAG (NBhuart, UART_FLAG_RXNE);
				if (Cnt>254) 
				{
					memset(RetString->AtCmd,0,sizeof(RetString->AtCmd));//clear array 
					StringCpy(RetString->AtCmd,(uint8_t *)Toolong,sizeof(Toolong));
					NumOfChar=sizeof(Toolong);//force to end of string
					break;
				}
				else
					Cnt++;
				//tickstart = HAL_GetTick();
		}
		else
		{
			if ((HAL_GetTick()-tickstart)>Timeout)
			{
			  if (Cnt<2) 
			  {
					memset(RetString->AtCmd,0,sizeof(RetString->AtCmd));//clear array 
					StringCpy(RetString->AtCmd,(uint8_t *)TimeOutString,sizeof(TimeOutString));
		      NumOfChar=sizeof(TimeOutString);
					break;
				}
				else
				{
				  NumOfChar=Cnt-1;
				  break;
			  }//if (Cnt<2)	
			}
		}//if()
	}//for(;;)
	//RetString->AtCmd[NumOfChar]=0;//end of string
	RetString->CmdLen=NumOfChar;
	//HAL_UART_Transmit(DBGhuart,(uint8_t*)RetString->AtCmd,RetString->CmdLen,500);
	//DisplayStringHex(USARTy,RetString->AtCmd,RetString->CmdLen);
}
void NBMsgUp(UART_HandleTypeDef *DBGhuart,UART_HandleTypeDef *NBhuart,char *NBMsgString,uint32_t Timeout, uint8_t confirm)
{
  char DataPak[256],LenString[5];
	const char SendCmd[]="AT+QLWDATASEND=19,0,0,";
	const char NoConfirm[]=",0x0000";//0x0100
	const char NeedConfirm[]=",0x0100";//0x0100
	const char Sending[]="Sending Data Package......\n";
	const char Sendfail[]="Send Fail......\n";
	const char Sended[]="Data Package Transmited!\n";
  volatile uint16_t NumOfChar,temp,PL,RespLen;
	volatile uint16_t i,j;
	volatile uint32_t MsgPn;
  CmdStruct CDMStruct,RetMsg;
	
	RespLen=0;
	HAL_UART_Transmit(DBGhuart,(uint8_t*)Sending,strlen(Sending),50);
	NumOfChar=NBMsgString[0];//Get the length of message 
	temp=chkcrc(NBMsgString,0,NumOfChar-1);//CRC include Length byte
	NBMsgString[NumOfChar-1]=temp>>8;
	NBMsgString[NumOfChar]=temp&0x00ff;	
	HexToString((uint8_t *)NBMsgString,(unsigned char *)DataPak,NumOfChar+1);//include length byte
	PL=2*NumOfChar+2;
	for(j=0;j<22;j++) CDMStruct.AtCmd[RespLen++]=SendCmd[j];
	//convert length
	if(PL>999)
	{	
		HAL_UART_Transmit(DBGhuart,(uint8_t*)Sendfail,strlen(Sendfail),50);
	  return;
	}
	else
	{
		if(PL>99)
		{
			LenString[0]=HexToAsc(PL/100);
			LenString[1]=HexToAsc((PL%100)/10);
			LenString[2]=HexToAsc(PL%10);
			for(j=0;j<3;j++) CDMStruct.AtCmd[RespLen++]=LenString[j];
		}
		else if(PL>9)
		{
			LenString[0]=HexToAsc(PL/10);
			LenString[1]=HexToAsc(PL%10);
			for(j=0;j<2;j++) CDMStruct.AtCmd[RespLen++]=LenString[j];
		}
		else
		{
			LenString[0]=HexToAsc(PL);
			CDMStruct.AtCmd[RespLen++]=LenString[0];
		}
	}
	CDMStruct.AtCmd[RespLen++]=',';
	for(j=0;j<PL;j++) CDMStruct.AtCmd[RespLen++]=DataPak[j];//Data part
	if(confirm)
	{
		for(j=0;j<7;j++) CDMStruct.AtCmd[RespLen++]=NeedConfirm[j];//Confirm part
	}
	else
	{
		for(j=0;j<7;j++) CDMStruct.AtCmd[RespLen++]=NoConfirm[j];//Confirm part
	}
	
	CDMStruct.CmdLen=RespLen;	
  NB_ATCMD(DBGhuart,NBhuart,&CDMStruct,&RetMsg,Timeout);
	HAL_UART_Transmit(DBGhuart,(uint8_t*)Sended,strlen(Sended),50);
}
/*********************************************************************************
 *函数名B_CommCheck                                                            *
 *功  能：//检查NB模块通信                                                       *
 *参  数：                                                                       *
 *返回值：0---OK   1---Fail                                                      *       
 *********************************************************************************/
uint8_t NB_CommCheck(UART_HandleTypeDef *NBhuart)
{
	char RepChar[16];
	const char AtCmd[]="AT\r";
	uint8_t j;
	uint32_t tickstart = 0U;
	//Synchronize
	memset(RepChar,0,sizeof(RepChar));//clear array 
	//HAL_UART_Transmit(&huart1,(uint8_t*)AtCmd,3,50);//Test
	HAL_UART_Transmit(NBhuart,(uint8_t*)AtCmd,3,50);//send ATCMD to NB
	j=0;
	tickstart = HAL_GetTick();
	while(1)
	{
		if(__HAL_UART_GET_FLAG(NBhuart,UART_FLAG_RXNE) != RESET)
		{
			RepChar[j]=(uint8_t)(NBhuart->Instance->DR & 0xff);
			__HAL_UART_CLEAR_FLAG (NBhuart, UART_FLAG_RXNE);	
	    j++;
			RepChar[j]=0;//end of string
      if (j==15) break;//Sync fail
		}
		else
		{
			if ((HAL_GetTick()-tickstart)>500) 
			{
				if (!NB_ACK_Check(RepChar,j,"OK")) 
					return 0;//NB is OK!
				else 
					return 1;//Sync fail
			}
	  }
  }
	return 1;//Sync fail
}
/*********************************************************************************
 *函数名：NB_ACK_Check                                                           *
 *功  能：//检查NB模块应答是否符合预期                                           *
 *参  数：                                                                       *
 *返回值：0---match   1---mismatch                                               *       
 *********************************************************************************/
uint8_t NB_ACK_Check(char *ackstruct,uint8_t len,char *str)
{
	uint8_t j=0;
	while(j<len)
	{
			if(strstr((const char *)(ackstruct+j),(const char *)str)!=NULL)//match
			{ 
				return 0;
			}
			else //mismatch
			{ 
				j++;
			}                             	
	}
  return 1;//mismatch
}
/**********************************************************************************
 * @brief  NB Attachment.                                                         *
 * @param  None                                                                   *
 **********************************************************************************/
uint8_t NB_Attachment(UART_HandleTypeDef *DBGhuart,UART_HandleTypeDef *NBhuart, uint8_t attachTimes)
{
	//RTC_DateTypeDef  sdate;
  //RTC_TimeTypeDef  stime;
  //RTC_AlarmTypeDef salarm;
	
	enum ATSTATUS ATStatus;
	uint8_t i,TryTimes,RspTag;
	uint8_t RstartCnt;
	char *Index,IMEIstring[20];
	int tick;
//	uint32_t Tmp_HH,Tmp_MM,Tmp_SS,TimeVar;

	CT_PLATFORM_CONNECT_FLAG = 0;
	//NBStatus:NBSTART=1,AT,DISSLP,CPIN,IPADD,CESQ,CFUN,QLWSERV,CEREG,NCDP,QLWCONF,
	//QLWADDOBJ0,QLWADDOBJ1,QLWOPEN,QLWCFG,QLWDATASEND,CONNECTOK,GETIMEI,GETTIME,ENDATT

	NB_OFF();
	HAL_Delay(500);
	NB_ON();//Turn NB Power on
  HAL_Delay(500);
	NB_StartUp();
	HAL_Delay(500);
	
	RstartCnt=0;
	ATStatus=START;	
	
	while(1)
	{
		memset(AttOpMsg.AtCmd,0,sizeof(AttOpMsg.AtCmd));//clear array 
		memset(AttRetMsg.AtCmd,0,sizeof(AttRetMsg.AtCmd));//clear array 
		AttOpMsg.CmdLen=0;
		AttRetMsg.CmdLen=0;
		switch(ATStatus)
		{	
			#if 1
			case START:
				HAL_UART_Transmit(DBGhuart,(uint8_t*)NBSTA,strlen(NBSTA),200);
				RstartCnt++;
			  //if (RstartCnt>5) 
				if (RstartCnt>attachTimes) 
				{
					char tt[]="attach times reached, return!\r\n";
					HAL_UART_Transmit(DBGhuart,(uint8_t*)tt,strlen(tt),200);
					return 1;
				}
				NB_RESETB();
			  HAL_Delay(200);
				NB_StartUp();//NB Power On
			  HAL_Delay(10000);
				ATStatus=AT;
			  TryTimes=0;
				break;
			case AT:
				if (NB_CommCheck(NBhuart))//Test NB Communication Channnel
				{
					TryTimes++;
					if (TryTimes>3)//fail 4 times 
					{
						ATStatus=START;
					}
					else
						ATStatus=AT;
			  }
				else
				{
					//NBStatus=DISSLP;
					ATStatus=ATE;
				}
				break;
			case ATE:
				StringCpy(AttOpMsg.AtCmd,(uint8_t *)ATECLOSE,strlen(ATECLOSE));
				AttOpMsg.CmdLen=strlen(ATECLOSE);
				NB_ATCMD(DBGhuart,NBhuart,&AttOpMsg,&AttRetMsg,500);
				ATStatus=CSQ;
				break;
			case CSQ:
				StringCpy(AttOpMsg.AtCmd,(uint8_t *)ATCSQ,strlen(ATCSQ));
				AttOpMsg.CmdLen=strlen(ATCSQ);
			  memset(AttRetMsg.AtCmd,0,sizeof(AttRetMsg.AtCmd));//clear array 
				NB_ATCMD(DBGhuart,NBhuart,&AttOpMsg,&AttRetMsg,500);
				ATStatus=CPIN;
				break;
			case CPIN:
			  StringCpy(AttOpMsg.AtCmd,(uint8_t *)ATCPIN,strlen(ATCPIN));
				AttOpMsg.CmdLen=strlen(ATCPIN);
				tick = HAL_GetTick();
				while(HAL_GetTick() - tick < (20*1000))
				{
					memset(AttRetMsg.AtCmd,0,sizeof(AttRetMsg.AtCmd));//clear array 
					NB_ATCMD(DBGhuart,NBhuart,&AttOpMsg,&AttRetMsg,500);
					if (0 == NB_ACK_Check((char *)AttRetMsg.AtCmd,AttRetMsg.CmdLen,"+CPIN: READY"))
					{
					  ATStatus=CREG;
						break;
					}
					HAL_Delay(2000);
				}
				
				if(CREG != ATStatus)
				{
					ATStatus = START;
				}
				break;
#if 0
			case IPADD:
				HAL_UART_Transmit(DBGhuart,(uint8_t*)WIATIPADD,strlen(WIATIPADD),200);
			  //RspTag=GetNBMsgAndCheck(NBhuart,&AttRetMsg,"+IP:",22000);
				RspTag=GetNBMsgAndCheck(NBhuart,&AttRetMsg,"+IP:",20000);
				if(RspTag) 
				{
					HAL_UART_Transmit(DBGhuart,(uint8_t*)AttRetMsg.AtCmd,AttRetMsg.CmdLen,500);//display
					NBStatus=CESQ;
					TryTimes=0;
				}
				else 
					NBStatus=NBSTART;
			  break;
#endif

			case CREG:
				StringCpy(AttOpMsg.AtCmd,(uint8_t *)ATCREG,strlen(ATCREG));//UE Functionality Querry
				AttOpMsg.CmdLen=strlen(ATCREG);
				
				for(i = 0; i < 5; i++)
				{
					memset(AttRetMsg.AtCmd,0,sizeof(AttRetMsg.AtCmd));//clear array 
					NB_ATCMD(DBGhuart,NBhuart,&AttOpMsg,&AttRetMsg,500);
					if (0 == NB_ACK_Check((char *)AttRetMsg.AtCmd,AttRetMsg.CmdLen,"+CREG: 0,1") || 
							0 == NB_ACK_Check((char *)AttRetMsg.AtCmd,AttRetMsg.CmdLen,"+CREG: 0,5"))
					{
					  ATStatus=CGREG;
						break;
					}
					HAL_Delay(1000);
				}
				
				if(3 == i)
				{
					ATStatus=START;
				}
				break;
			case CGREG:
				StringCpy(AttOpMsg.AtCmd,(uint8_t *)ATCGREG,strlen(ATCGREG));//EPS Network Registration Status
				AttOpMsg.CmdLen=strlen(ATCGREG);
				for(i = 0; i < 3; i++)
				{
					memset(AttRetMsg.AtCmd,0,sizeof(AttRetMsg.AtCmd));//clear array 
					NB_ATCMD(DBGhuart,NBhuart,&AttOpMsg,&AttRetMsg,500);
					if (0 == NB_ACK_Check((char *)AttRetMsg.AtCmd,AttRetMsg.CmdLen,"+CGREG: 0,1") || 
							0 == NB_ACK_Check((char *)AttRetMsg.AtCmd,AttRetMsg.CmdLen,"+CGREG: 0,5"))
					{
					  ATStatus=CGATT;
						break;
					}
					HAL_Delay(1000);
				}
				
				if(3 == i)
				{
					ATStatus=START;
				}
				break;
			case CGATT:
				StringCpy(AttOpMsg.AtCmd,(uint8_t *)ATCGATT,strlen(ATCGATT));// Query whether the network is activated
				AttOpMsg.CmdLen=strlen(ATCGATT);
				for(i = 0; i < 3; i++)
				{
					memset(AttRetMsg.AtCmd,0,sizeof(AttRetMsg.AtCmd));//clear array 
					NB_ATCMD(DBGhuart,NBhuart,&AttOpMsg,&AttRetMsg,500);
					if (0 == NB_ACK_Check((char *)AttRetMsg.AtCmd,AttRetMsg.CmdLen,"+CGATT: 1"))
					{
					  ATStatus=GETIMEI;
						break;
					}
					HAL_Delay(1000);
				}
				if(3 == i)
				{
					ATStatus=START;
				}
				break;
			case GETIMEI:
				StringCpy(AttOpMsg.AtCmd,(uint8_t *)ATGSN,strlen(ATGSN));//read IMEI	
				AttOpMsg.CmdLen=strlen(ATGSN);
				NB_ATCMD(DBGhuart,NBhuart,&AttOpMsg,&AttRetMsg,300);
				//already get IMEI
				if (0 == NB_ACK_Check((char*)AttRetMsg.AtCmd,AttRetMsg.CmdLen,"OK") &&
						AttRetMsg.CmdLen > NUMBITOFIMEI && 
						'8' == AttRetMsg.AtCmd[2] && '6' == AttRetMsg.AtCmd[3])
				{
						for(i=0;i<NUMBITOFIMEI;i++) deviceId[i]=AttRetMsg.AtCmd[i+2];
						//check imei
						ATStatus=GETTIME;

				}
				else
				{
					ATStatus=START;
				}
			  	
				break;
		#endif
			case GETTIME:
				if(true == IS_FIRST_SYSC_TIME)
				{
					IS_FIRST_SYSC_TIME = false;
					sysc_time_from_network();
				}
				ATStatus=ENDATT;
				break;
			case ENDATT:	
				memset(NBMsg.AtCmd,0,sizeof(NBMsg.AtCmd));//clear array 
				memset(ConMsg.AtCmd,0,sizeof(ConMsg.AtCmd));//clear array 
				memset(ReStr.AtCmd,0,sizeof(ReStr.AtCmd));//clear array
				NBMsg.CmdLen=0;
				ConMsg.CmdLen=0;
				ReStr.CmdLen=0;	
				network_first_unavailable_tick = 0;
				return 0;
			default:
				ATStatus=START;
				break;
		}
	}
}

/**********************************************************************/
/* @Brief:Get Msg from NB and check Keyword                           */
/**********************************************************************/
uint8_t GetNBMsgAndCheck(UART_HandleTypeDef *NBhuart,CmdStruct *NBMsgPt,char *KeyWord,uint32_t Timeout)
{
	uint8_t j = 0,First;
	uint32_t tickstart = 0U;
	
	First=0;
	tickstart = HAL_GetTick();
	if (!Timeout) return 0;//get NBMsg fail
	while(1) //get NB respond within Timeout
	{
		if(__HAL_UART_GET_FLAG(NBhuart,UART_FLAG_RXNE) != RESET)
		{
			NBMsgPt->AtCmd[j]=(uint8_t)(NBhuart->Instance->DR & 0xff);
			__HAL_UART_CLEAR_FLAG (NBhuart, UART_FLAG_RXNE);
			j++;
			NBMsgPt->AtCmd[j]=0;
			if (j==255)//too much char geted
			{
				 return 0;//ERROR!
			}
			tickstart = HAL_GetTick();
			First=1;
		}
		else
		{
			if(First)
			{
				if ((HAL_GetTick()-tickstart)>20) 
				{
					if(strstr((char*)NBMsgPt->AtCmd,KeyWord)!=NULL) 
					{
						NBMsgPt->CmdLen=j-1;
						return 1;
					}
					else
						return 0;
				}
			}
			else
			{
				if ((HAL_GetTick()-tickstart)>Timeout) return 0;//no respond
			}
		}//if()
	}
//	return 0;
}

void turnOnAllLineSwitch(void)
{
	for(int i = 1; i <= SWITCH_NUM; i++)
	{
		TurnOnPowerLineSwitch(i);
	}
}

void turnOffAllLineSwitch(void)
{
	for(int i = 1; i <= SWITCH_NUM; i++)
	{
		TurnOFFPowerLineSwitch(i);
	}
}
/**********************************************************************/
/* @Brief:Turn On Power Line Switch                                   */
/* @Para:LineNumber is the number of line                             */
/**********************************************************************/
uint8_t TurnOnPowerLineSwitch(uint8_t LineNumber)
{
	switch(LineNumber)
	{
		case 1:
			LineSwitch_ON(L1);
			//BoxLED_ON(LED1);
			return 1;
		case 2:
			LineSwitch_ON(L2);
			//BoxLED_ON(LED2);
			return 1;
		case 3:
			LineSwitch_ON(L3);
			//BoxLED_ON(LED3);
			return 1;
		case 4:
			LineSwitch_ON(L4);
			//BoxLED_ON(LED4);
			return 1;
		case 5:
			LineSwitch_ON(L5);
			//BoxLED_ON(LED5);
			return 1;
		case 6:
			LineSwitch_ON(L6);
			//BoxLED_ON(LED6);
			return 1;
		case 7:
			return 1;
		case 8:
			return 1;
		default:
			return 0;
	}
}
/**********************************************************************/
/* @Brief: Turn OFF Power Line Switch                                 */
/* @Para:LineNumber is the number of line                             */
/**********************************************************************/
uint8_t TurnOFFPowerLineSwitch(uint8_t LineNumber)
{
	switch(LineNumber)
	{
		case 1:
			LineSwitch_OFF(L1);
			//BoxLED_OFF(LED1);
			return 1;
		case 2:
			LineSwitch_OFF(L2);
			//BoxLED_OFF(LED2);
			return 1;
		case 3:
			LineSwitch_OFF(L3);
			//BoxLED_OFF(LED3);
			return 1;
		case 4:
			LineSwitch_OFF(L4);
			//BoxLED_OFF(LED4);
			return 1;
		case 5:
			LineSwitch_OFF(L5);
			//BoxLED_OFF(LED5);
			return 1;
		case 6:
			LineSwitch_OFF(L6);
			//BoxLED_OFF(LED6);
			return 1;
		case 7:
			return 1;
		case 8:
			return 1;
		default:
			return 0;
	}
}
/**********************************************************************/
/* @Brief:Check Switch state                                          */
/* @Para:LineNumber is the Serial Number of line                      */
/* @Para:StateToBeSet is the switch state to be set (0=OFF 1=ON)      */
/* @return:0=OK,1=ERROR                                               */
/**********************************************************************/
uint8_t CheckSwitch(uint8_t LineNumber,uint8_t StateToBeSet)
{
	switch(LineNumber)
	{
		case 1:
			if (BoxGetSwitchState(K1)==StateToBeSet) return 0;
			return 1;
		case 2:
			if (BoxGetSwitchState(K2)==StateToBeSet) return 0;
			return 1;
		case 3:
			if (BoxGetSwitchState(K3)==StateToBeSet) return 0;
			return 1;
		case 4:
			if (BoxGetSwitchState(K4)==StateToBeSet) return 0;
			return 1;
		case 5:
			if (BoxGetSwitchState(K5)==StateToBeSet) return 0;
			return 1;
		case 6:
			if (BoxGetSwitchState(K6)==StateToBeSet) return 0;
			return 1;
		case 7:
			return 0;
		case 8:
			return 0;
		default:
			return 0;
	}
}
void GetCurrentData(CurrentState* Ctrl,char *ChangedMsg)
{
	ChangedMsg[4]=Ctrl->Alarm;//Alarm
	ChangedMsg[5]=Ctrl->SwitchState[0];//K1
	ChangedMsg[6]=Ctrl->SwitchState[1];//K2
	ChangedMsg[7]=Ctrl->SwitchState[2];//K3
	ChangedMsg[8]=Ctrl->SwitchState[3];//K4
	ChangedMsg[9]=Ctrl->SwitchState[4];//K5
	ChangedMsg[10]=Ctrl->SwitchState[5];//K6
	ChangedMsg[11]=Ctrl->SwitchState[6];//K7(reserve)
	ChangedMsg[12]=Ctrl->SwitchState[7];//K8(reserve)
	ChangedMsg[13]=(Ctrl->quantity>>24)&0x000000FFU;
	ChangedMsg[14]=(Ctrl->quantity>>16)&0x000000FFU;
	ChangedMsg[15]=(Ctrl->quantity>> 8)&0x000000FFU;
	ChangedMsg[16]=(Ctrl->quantity)    &0x000000FFU;
	ChangedMsg[17]=(Ctrl->Current>>8)&0x00ffU;
	ChangedMsg[18]=(Ctrl->Current)   &0x00ffU;
	ChangedMsg[19]=(Ctrl->Power>>8)  &0x00ffU;
	ChangedMsg[20]=(Ctrl->Power)     &0x00ffU;
	ChangedMsg[21]=(Ctrl->Temperature>>8)&0x00ffU;
	ChangedMsg[22]=(Ctrl->Temperature)   &0x00ffU;
	ChangedMsg[23]= (deviceConfig.reportConfig.period>>8)&0x00ffU;
	ChangedMsg[24]=(deviceConfig.reportConfig.period)   &0x00ffU;
	ChangedMsg[27]= (deviceConfig.reportConfig.tempDelta>>8)&0x00ffU;
	ChangedMsg[28]=(deviceConfig.reportConfig.tempDelta)   &0x00ffU;
	ChangedMsg[29]=(deviceConfig.switchPolicy.autoSwitch)   &0x00ffU;
	for(int i = 0; i < SWITCH_NUM; i++)
	{
		ChangedMsg[30+i*4]= deviceConfig.switchPolicy.SwitchOnTimeHh[i];
		ChangedMsg[31+i*4]= deviceConfig.switchPolicy.SwitchOnTimeMm[i];
		ChangedMsg[32+i*4]= deviceConfig.switchPolicy.SwitchOFFTimeHh[i];
		ChangedMsg[33+i*4]= deviceConfig.switchPolicy.SwitchOFFTimeMm[i];
		
		ChangedMsg[62+i]= deviceConfig.switchPolicy.autoSwitchMode[i];
		
		ChangedMsg[70+i*2]= (Ctrl->SwitchLineCurrent[i] >> 8)&0xff;//line current
		ChangedMsg[71+i*2] = Ctrl->SwitchLineCurrent[i]&0xff;
		
		ChangedMsg[86+i*2]= (Ctrl->SwitchLineVoltage[i] >> 8)&0xff;//line voltage
		ChangedMsg[87+i*2] = Ctrl->SwitchLineVoltage[i]&0xff;
	}
}
/**********************************************************************/
/* @Brief:Get Current Switch State and alarm state                    */
/* @Para:Controller Data Block                                        */
/* @return:none change the Parameter in controller Block              */
/**********************************************************************/
void GetAlarmState(CurrentState* Ctrl)
{
	//fill Switch alarm state(bit:1=ERROR 0=OK)
	Ctrl->Alarm =0;
	Ctrl->Alarm |=CheckSwitch(K1,Ctrl->SwitchState[0]);
	Ctrl->Alarm |=CheckSwitch(K2,Ctrl->SwitchState[1])<<1;
	Ctrl->Alarm |=CheckSwitch(K3,Ctrl->SwitchState[2])<<2;
	Ctrl->Alarm |=CheckSwitch(K4,Ctrl->SwitchState[3])<<3;
	Ctrl->Alarm |=CheckSwitch(K5,Ctrl->SwitchState[4])<<4;
	Ctrl->Alarm |=CheckSwitch(K6,Ctrl->SwitchState[5])<<5;
	Ctrl->Alarm &=0x3F;
//fill Switch Current state
//	Ctrl->SwitchState[0]=BoxGetSwitchState(K1);
//	Ctrl->SwitchState[1]=BoxGetSwitchState(K2);
//	Ctrl->SwitchState[2]=BoxGetSwitchState(K3);
//	Ctrl->SwitchState[3]=BoxGetSwitchState(K4);
//	Ctrl->SwitchState[4]=BoxGetSwitchState(K5);
//	Ctrl->SwitchState[5]=BoxGetSwitchState(K6);
//	Ctrl->SwitchState[6]=0;//BoxGetSwitchState(K7);
//	Ctrl->SwitchState[7]=0;//BoxGetSwitchState(K8);
}
uint32_t GetMeterOFWattMeter(void)//Read The Meter of WattMeter
{
	CmdStruct sendC, recieveC;
	int i=0, cmdLen;
	uint32_t iQuantity = 0;
	float fQuantity = 0;
	cmdLen = sizeof(readQuantityCmd)/sizeof(char);
	
	for(i= 0; i < cmdLen; i++)
	{
		sendC.AtCmd[i] = readQuantityCmd[i];
	}
	sendC.CmdLen = cmdLen;
	
	USART_SEND_HEXSTRING(&huart1,&huart5, &sendC, &recieveC, 400);
	uint16_t calcCrc = chkcrc((char *)recieveC.AtCmd, 0, recieveC.CmdLen-2);
	uint16_t packageCrc = recieveC.AtCmd[recieveC.CmdLen-1]*256+recieveC.AtCmd[recieveC.CmdLen-2];
	
	if(packageCrc == calcCrc)
	{
		uint8_t data[4];
		
		for(int i = 0; i < 4; i++)
		{
			data[i] = recieveC.AtCmd[6-i];
		}
		float *tmp = (float *)data;
		fQuantity = *tmp;
		iQuantity = f2h(fQuantity);
	}
	
	return iQuantity;
}

void GetSwitchLineCurrent(CurrentState* Ctrl)
{
	CmdStruct sendC, recieveC;
	uint16_t sendCRC;
	int i=0, cmdLen;
	uint32_t iQuantity = 0;
	cmdLen = sizeof(readSwitchLineCurrentCmd)/sizeof(char);
	
	for(i= 0; i < cmdLen; i++)
	{
		sendC.AtCmd[i] = readSwitchLineCurrentCmd[i];
	}
	sendC.CmdLen = cmdLen;
	sendCRC = chkcrc((char *)sendC.AtCmd, 0, sendC.CmdLen-2);
	sendC.AtCmd[sendC.CmdLen-2] = sendCRC & 0xff;
	sendC.AtCmd[sendC.CmdLen-1] = (sendCRC >> 8) & 0xff;
	
	
	USART_SEND_HEXSTRING(&huart1,&huart4, &sendC, &recieveC, 500);
	uint16_t calcCrc = chkcrc((char *)recieveC.AtCmd, 0, recieveC.CmdLen-2);
	uint16_t packageCrc = recieveC.AtCmd[recieveC.CmdLen-1]*256+recieveC.AtCmd[recieveC.CmdLen-2];
	
	if(packageCrc == calcCrc)
	{
		if(84 == recieveC.AtCmd[2] && recieveC.CmdLen == 89)
		{
			for(int i = 0; i < SWITCH_NUM; i++)
			{
				Ctrl->SwitchLineCurrent[i] = recieveC.AtCmd[5+i*14]*256+recieveC.AtCmd[6+i*14];
				Ctrl->SwitchLineVoltage[i] = recieveC.AtCmd[3+i*14]*256+recieveC.AtCmd[4+i*14];
			}
		}
	}
	
	return;
}

uint16_t GetCurrentMeter(void)//Read The Current of WattMeter
{
	uint16_t NewCurrent = 0;
	
	return NewCurrent;
	
}

uint16_t GetPowerMeter(void)//Read The Power of WattMeter
{
	uint16_t NewPower = 0;
	
	return NewPower;
	//collect new data
}

uint16_t GetTemperature(ADC_HandleTypeDef* hadc)//Get Temperature from AD channel-1
{
//	uint8_t n;
	uint32_t Value[32],AD_Value,max,min,Temper_val;
	uint16_t temperature;
	double temperate;
	
	AD_Value=0;
	for(char n=0;n<22;n++) // Get 22 Value for filter
	{
			HAL_ADC_Start(hadc);
			HAL_ADC_PollForConversion(hadc,10); //wait for conversion over,the second parameter is timeout,uint is ms        
			if(HAL_IS_BIT_SET(HAL_ADC_GetState(hadc), HAL_ADC_STATE_REG_EOC))
			{
				Value[n]=HAL_ADC_GetValue(hadc);
				AD_Value += Value[n];
			}                
	}
	max=Value[0];
	min=Value[0];
	for(char n=0;n<22;n++)//get max value and min value
	{
		max=(Value[n]<max)?max:Value[n];    
		min=(min<Value[n])?min:Value[n];
	}
	Temper_val=((AD_Value -max-min)/20)&0x0fff;
	temperate = (float)Temper_val*(3.3/4096);
	//temperature=((1.42-Temper_val*3.3/4096)*1000/4.3+25)*10;//temperature UNIT：0.1
	temperate = (1.43- temperate)/0.0043 +25;
	temperature = temperate*10;//temperature UNIT：0.1
	return temperature;
	
}

/**********************************************************************/
/* @Brief:Multiple Event Process                                      */
/*  (Data collection,condition report,Period report,Update LED state) */
/* @Para:Controller Data Block                                        */
/* @return:none                                                       */
/**********************************************************************/
void MultiEventProcess(CurrentState *Ctrl,char *replyMsg)
{
	static const char C5PCH[]="C5(Power Change).....\r";
	static const char C5TCH[]="C5(Temperature Change).....\r";
	static const char C5PERIOD[]="C5(Period report).....\r";
  //char RepChar[32];
	
	if((HAL_GetTick()-(Ctrl->Gapstart))>TIMEGAP)
	{
		//printTimeCounter(0);
		//printAlarmCount(0);
		switch(Ctrl->Phase)
		{
			case 0:
				Ctrl->LastTemperature=Ctrl->Temperature;//save Last Temperature
				Ctrl->Temperature=GetTemperature(&hadc1);//Get new Temperature
				break;
			case 1:
				GetAlarmState(Ctrl);//Get Alarm state
				break;
			case 2:
				Ctrl->Current=GetCurrentMeter();//Get new Current
				break;
			case 3:
				Ctrl->LastPower=Ctrl->Power;//save the last Power
				Ctrl->Power=GetPowerMeter();//Get new POWER
				break;
			case 4:
				Ctrl->quantity=GetMeterOFWattMeter();//Get new quantity Of WattMeter
				GetSwitchLineCurrent(Ctrl);//get line current
				//first report param after get all param
				if(IS_FIRST_REPORT)
				{
					replyMsg[2]=0xc5;
					GetCurrentData(Ctrl,replyMsg);//get data
					//NBMsgUp(&huart1,&huart3,replyMsg,300, 1);//report									 
					MQTT_publish((uint8_t *)replyMsg, replyMsg[0]+1);
					HAL_UART_Transmit(&huart1,(uint8_t*)"first report!\r\n", 15,100);
					Ctrl->ReportGapstart=HAL_GetTick();
					IS_FIRST_REPORT = false;
				}
				break;
			case 5:
				/*-------------------------Report in Power change-----------------------*/
				if  ((Myabs(Ctrl->LastPower-Ctrl->Power)>Ctrl->ReportCondPower)
					&&(Ctrl->ReportCondPower))
				{
					 replyMsg[2]=0xc5;
					 GetCurrentData(Ctrl,replyMsg);//get new data
					 DisplayStringHex(&huart1,replyMsg,25);//display respond message,25 include length byte
					 //NBMsgUp(&huart1,&huart3,replyMsg,300, 0);//report									 
					 MQTT_publish((uint8_t *)replyMsg, replyMsg[0]+1);
					 HAL_UART_Transmit(&huart1,(uint8_t*)C5PCH,strlen(C5PCH),100);
				}
				else
					/*--------------------Report in Temperature Change---------------------*/
					if  ((Myabs(Ctrl->LastTemperature-Ctrl->Temperature)>Ctrl->ReportCondTemp*10)
						&&(Ctrl->ReportCondTemp))
					{
						 replyMsg[2]=0xc5;
						 GetCurrentData(Ctrl,replyMsg);//get data
						 DisplayStringHex(&huart1,replyMsg,25);//display respond message,25 include length byte
						 //NBMsgUp(&huart1,&huart3,replyMsg,300, 0);//report	
						 MQTT_publish((uint8_t *)replyMsg, replyMsg[0]+1);						
						 HAL_UART_Transmit(&huart1,(uint8_t*)C5TCH,strlen(C5TCH),100);
					}
				break;
			default:
				break;
		}
		LEDEventProc(Ctrl);//Update LED state
		Ctrl->Phase=(Ctrl->Phase+1)%6;
		Ctrl->Gapstart=HAL_GetTick();
	}
	
	/*---------------------------Report in Period--------------------------*/
	if(((HAL_GetTick()-Ctrl->ReportGapstart)>Ctrl->ReportGap)&&(Ctrl->ReportCondPeriod))
	{ 
		 //sprintf(RepChar,"Period:0x%08x\r\n",Ctrl->ReportGap);
		 //HAL_UART_Transmit(&huart1,(uint8_t*)RepChar,19,500);
		 replyMsg[1]=0x00;
		 replyMsg[2]=0xc5;
		 GetCurrentData(Ctrl,replyMsg);//get data
		 //DisplayStringHex(&huart1,replyMsg,25);//display respond message,25 include length byte
		 for(int i = 0; i < SWITCH_NUM; i++)
		 {
			 replyMsg[5+i] = BoxGetSwitchState(i+1);
		 }
		 //NBMsgUp(&huart1,&huart3,replyMsg,300, 0);//report
		 MQTT_publish((uint8_t *)replyMsg, replyMsg[0]+1);		 
		 HAL_UART_Transmit(&huart1,(uint8_t*)C5PERIOD,strlen(C5PERIOD),100);
		 Ctrl->ReportGapstart=HAL_GetTick();
	}
	/*---------------------------Period Report END-------------------------*/
}
//@brief Control LED state
void LEDEventProc(CurrentState *Ctrl)
{
	AlarmLED(Ctrl);
	LineLED(K1);	
	LineLED(K2);	
	LineLED(K3);	
	LineLED(K4);	
	LineLED(K5);	
	LineLED(K6);
}
//@brief AlarmLED:Turn on /Turn off Alarm LED
void AlarmLED(CurrentState *Ctrl)
{
//	char RepChar[32];
//	sprintf(RepChar,"Alarm:0x%02x\r\n",Ctrl->Alarm);
//	HAL_UART_Transmit(&huart1,(uint8_t*)RepChar,12,500);
	if(Ctrl->Alarm)
	{		
//		HAL_UART_Transmit(&huart1,(uint8_t*)"Alarm...\r",9,500);
//		sprintf(RepChar,"ALARM:0x%04x\r\n",ALARM);
//	  HAL_UART_Transmit(&huart1,(uint8_t*)RepChar,14,500);
		BoxLED_ON(ALARM);//Alarm:Turn ON ALARM LED
	}
	else 
	{
		//HAL_GPIO_WritePin(ALARM_PORT, ALARM, GPIO_PIN_SET);
		BoxLED_OFF(ALARM);//No Alarm:Turn OFF ALARM LED
	}
}
/**********************************************************/
/*@brief LineLED:Turn on /Turn off line LED               */
/*@Parameter Swt:K1,K2,K3,K4,K5,K6                        */
/*@return:none                                            */
/**********************************************************/
void LineLED(uint16_t Swt)
{
	uint16_t LEDARRY[9]={0xffff,LED1,LED2,LED3,LED4,LED5,LED6,0xffff,0xffff};
	
	if(BoxGetSwitchState(Swt)) 
		BoxLED_ON(LEDARRY[Swt]);//SWITCH ON:Turn ON LEDn
	else 
		BoxLED_OFF(LEDARRY[Swt]);//SWITCH OFF:Turn OFF LEDn
}
/**********************************************************/
/*@brief Switch controle process                          */
/*&Parameter Ctrl----the pointer of control block         */
/**********************************************************/
void SwitchConEventProc(CurrentState* Ctrl)
{ 
	//RTC_DateTypeDef sdateget;
  RTC_TimeTypeDef stimeget;
	uint8_t aShowTime[50] = {0};
	uint8_t ch;
	//RTC_TimeShow(aShowTime);
		
	if (Ctrl->EnableAutoSwitch)
	{//EnableAutoSwitch
		/* Get the RTC current Date */
		//HAL_RTC_GetDate(&hrtc, &sdateget, RTC_FORMAT_BIN);
		/* Get the RTC current Time */
		HAL_RTC_GetTime(&hrtc, &stimeget, RTC_FORMAT_BIN);
		/* Display time Format : hh:mm:ss */
  	//	sprintf((char*)aShowTime,"20%02d/%02d/%02d,%02d:%02d:%02d\r",\
	  //	       sdateget.Year,sdateget.Month,sdateget.Date,\
	  //         stimeget.Hours, stimeget.Minutes, stimeget.Seconds);
		for(ch=0;ch<8;ch++)
		{
			if((Ctrl->SwitchOnTimeHh[ch]==stimeget.Hours)&&(Ctrl->SwitchOnTimeMm[ch]==stimeget.Minutes))
			{
				sprintf((char*)aShowTime,"No%1x ON Time:%02d:%02d:%02d\r",ch,\
							 stimeget.Hours, stimeget.Minutes, stimeget.Seconds);
			}
			if((Ctrl->SwitchOFFTimeHh[ch]==stimeget.Hours)&&(Ctrl->SwitchOFFTimeMm[ch]==stimeget.Minutes))
			{
				sprintf((char*)aShowTime,"No%1x OFF Time:%02d:%02d:%02d\r",ch,\
							 stimeget.Hours, stimeget.Minutes, stimeget.Seconds);
			}
		
		}
	}
	return;
}

/**********************************************************/
/* @Brief:Check CRC                                       */
/* @para: arr is charaecter string                        */
/* @para: fxstat is position of first character           */
/* @para: len is the length of string                     */
/* @Note: CRC1=(byte)(crcc & 0xff)                        */
/*        CRC2=(byte)(crcc / 0x100)                       */
/**********************************************************/
static uint16_t chkcrc(char *arr,char fxstat,char len)
{
	uint16_t crcc,i,j,it;
	crcc = 0xffff;
	it = 0;
	for (j=fxstat;j<=len-1;j++)
	{
		crcc = crcc ^ arr[j];
		for (i = 0; i <= 7; i++)
		{
			it = crcc & 1;
			crcc = crcc >> 1;
			crcc = crcc & 0x7fff;
			if (it == 1)
			  crcc = crcc ^ 0xa001;
			crcc = crcc & 0xffff;
		}
	}
	return crcc;
}
/**********************************************************************/
/* @Brief:ASC convert to Dec num                                      */
/**********************************************************************/
unsigned char DecNum(unsigned char ch)
{
	switch(ch)
	{
		case '0':return 0;
		case '1':return 1;
		case '2':return 2;
		case '3':return 3;
		case '4':return 4;
		case '5':return 5;
		case '6':return 6;
		case '7':return 7;
		case '8':return 8;
		case '9':return 9;
		default:return 0;
	}
}
/***********************************************************************************/
/* @Brief:ch is Hex Character                                                      */
/***********************************************************************************/
unsigned char IsHex(unsigned char ch)
{
	if ((ch>=0x30 && ch<=0x39)||(ch>=0x41 && ch<=0x46)) return 1;
	else return 0;
}
/***********************************************************************************/
/* @Brief:ch is Dec Character                                                      */
/***********************************************************************************/
unsigned char IsDec(unsigned char ch)
{
	if ((ch>=0x30) && (ch<=0x39)) return 1;
	else return 0;
}
/***********************************************************************************/
/* @Brief:hex num convert to ASC                                                   */
/***********************************************************************************/
unsigned char HexToAsc(unsigned char num)
{
	switch(num)
	{
		case 0:return '0';
		case 1:return '1';
		case 2:return '2';
		case 3:return '3';
		case 4:return '4';
		case 5:return '5';
		case 6:return '6';
		case 7:return '7';
		case 8:return '8';
		case 9:return '9';
		case 10:return 'A';
		case 11:return 'B';	
		case 12:return 'C';	
		case 13:return 'D';	
		case 14:return 'E';	
		case 15:return 'F';			
		default:return '0';
	}
}
/***********************************************************************************/
/* @Brief:ASC convert to hex num                                                   */
/***********************************************************************************/
unsigned char HexNum(unsigned char ch)
{
	switch(ch)
	{
		case '0':return 0;
		case '1':return 1;
		case '2':return 2;
		case '3':return 3;
		case '4':return 4;
		case '5':return 5;
		case '6':return 6;
		case '7':return 7;
		case '8':return 8;
		case '9':return 9;
		case 'A':return 10;
		case 'a':return 10;
		case 'B':return 11;	
		case 'b':return 11;
		case 'C':return 12;	
		case 'c':return 12;
		case 'D':return 13;	
		case 'd':return 13;
		case 'E':return 14;	
		case 'e':return 14;
		case 'F':return 15;		
    case 'f':return 15;		
		default:return 0;
	}
}
/***************************************************/
/* @brief:abs                                      */
/* Parameter:num the number                        */
/***************************************************/
int32_t Myabs(int32_t num)
{
	return num*((num>>31<<1)+1);
}
/***************************************************/
/* @brief:display string in HEX                    */
/* Parameter:StringPoint---start address of string */
/*           length--------The length of string    */
/***************************************************/
void DisplayStringHex(UART_HandleTypeDef* USARTx,char *StringPoint,uint8_t length)
{
	uint8_t j;
	char dispCH[]="   ";
	for(j=0;j<length;j++)
  {
	  //sprintf(dispCH,"%02x",StringPoint[j]);
		dispCH[0]=HexToAsc(StringPoint[j]>>4);
		dispCH[1]=HexToAsc(StringPoint[j]&0x0f);
//		USART_PutChar(USARTy,dispCH[0]);
//		USART_PutChar(USARTy,dispCH[1]);
//		USART_PutChar(USARTy,' ');
		HAL_UART_Transmit(&huart1,(uint8_t*)dispCH,3,500);
	}
	//USART_PutChar(USARTy,'\n');
	HAL_UART_Transmit(&huart1,(uint8_t*)'\n',1,2);
}

/***************************************************************************/
/* @brief:String convert to HEX                                            */
/* Parameter:StrPoint---start address of string                            */
/*           Data-------Output data buffer                                 */
/*           length-----the length of string                               */
/***************************************************************************/
uint8_t StringToHex(unsigned char *StrPoint,uint8_t *Data,uint8_t length)
{
	uint8_t temp,j;
	if (StrPoint==NULL) return 1;//fail
	if (length % 2) return 1;//fail
	j=0;
	while(j<length)
	{
		temp=HexNum(StrPoint[j])*16+HexNum(StrPoint[j+1]);
		j=j+2;
		*Data=temp;
		Data++;
	}
	return 0;
}
/***************************************************************************/
/* @brief:Hex array convert to HEXstring                                   */
/* Parameter:HexData---start address of Hex array                          */
/*           ChString-------Output data buffer                             */
/*           length-----the length of Hex array                            */
/***************************************************************************/
uint8_t HexToString(uint8_t *HexData,unsigned char *ChString,uint8_t length)
{
	uint8_t j;
	if (HexData==NULL) return 1;//fail
	if (length==0) return 1;//fail
	j=0;
	for(j=0;j<length;j++)
	{
		*ChString++=HexToAsc(HexData[j]>>4);
		*ChString++=HexToAsc(HexData[j]&0x0f);
	}
	return 0;
}

/**
  * @brief  Display the current Data and time.
  * @param  showtime : pointer to buffer
  * @retval None
  */
static void RTC_TimeShow(uint8_t* showtime)
{
  RTC_DateTypeDef sdateget;
  RTC_TimeTypeDef stimeget;
  
  /* Get the RTC current Time */
  HAL_RTC_GetTime(&hrtc, &stimeget, RTC_FORMAT_BIN);
  /* Get the RTC current Date */
  HAL_RTC_GetDate(&hrtc, &sdateget, RTC_FORMAT_BIN);
  /* Display time Format : hh:mm:ss */
  sprintf((char*)showtime,"DATE:20%02d/%02d/%02d TIME:%02d:%02d:%02d\r",\
		       sdateget.Year,sdateget.Month,sdateget.Date,\
	         stimeget.Hours, stimeget.Minutes, stimeget.Seconds);
} 

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	char str[] = "set clock it error!\r\n";
	HAL_UART_Transmit(&huart1,(uint8_t*)str, strlen(str),2);
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
