#ifndef __KEY_H
#define	__KEY_H

#include "stm32f1xx_hal.h"

//  ���Ŷ���
/*
#define    KEY1_GPIO_CLK     RCC_APB2Periph_GPIOA
#define    KEY1_GPIO_PORT    GPIOA			   
#define    KEY1_GPIO_PIN		 GPIO_Pin_0

#define    KEY2_GPIO_CLK     RCC_APB2Periph_GPIOC
#define    KEY2_GPIO_PORT    GPIOC		   
#define    KEY2_GPIO_PIN		  GPIO_Pin_13
*/

//#define    KEY_PRESS(GPIOx,GPIO_Pin)      GPIO_ReadInputDataBit(GPIOx,GPIO_Pin) == KEY_ON
#define    KEY_PRESS(GPIOx,GPIO_Pin)      HAL_GPIO_ReadPin(GPIOx,GPIO_Pin)//GPIO_ReadInputDataBit(GPIOx,GPIO_Pin)
#define    KEY_LOOSEN(GPIOx,GPIO_Pin)     HAL_GPIO_ReadPin(GPIOx,GPIO_Pin) == KEY_OFF //GPIO_ReadInputDataBit(GPIOx,GPIO_Pin) == KEY_OFF

 /** �������±��ú�
	*  ��������Ϊ�ߵ�ƽ������ KEY_ON=1�� KEY_OFF=0
	*  ����������Ϊ�͵�ƽ���Ѻ����ó�KEY_ON=0 ��KEY_OFF=1 ����
	*/
	

#define KEY_ON	0
#define KEY_OFF	1
#define KEY_NULL 0
#define KEY_SHORT 1
#define KEY_LONG  10
#define SHORT_TIME 200

void Key_GPIO_Config(void);
uint8_t Key_Scan(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin);
//uint8_t Key_state(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin);
unsigned char key_handle(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin);

#endif /* __KEY_H */

