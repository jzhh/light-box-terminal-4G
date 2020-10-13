#include "key.h"  

extern uint32_t time;

/**
  * @brief  ���ð����õ���I/O��
  * @param  ��
  * @retval ��
  */
#if 0 
void Key_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/*���������˿ڵ�ʱ��*/
	RCC_APB2PeriphClockCmd(KEY1_GPIO_CLK|KEY2_GPIO_CLK,ENABLE);
	
	//ѡ�񰴼�������
	GPIO_InitStructure.GPIO_Pin = KEY1_GPIO_PIN; 
	// ���ð���������Ϊ��������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
	//ʹ�ýṹ���ʼ������
	GPIO_Init(KEY1_GPIO_PORT, &GPIO_InitStructure);
	
	//ѡ�񰴼�������
	GPIO_InitStructure.GPIO_Pin = KEY2_GPIO_PIN; 
	//���ð���������Ϊ��������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
	//ʹ�ýṹ���ʼ������
	GPIO_Init(KEY2_GPIO_PORT, &GPIO_InitStructure);	
}
#endif

 /*
 * ��������Key_Scan
 * ����  ������Ƿ��а�������
 * ����  ��GPIOx��x ������ A��B��C��D���� E
 *		     GPIO_Pin������ȡ�Ķ˿�λ 	
 * ���  ��KEY_OFF(û���°���)��KEY_ON�����°�����
 */
uint8_t Key_Scan(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin)
{			
	/*����Ƿ��а������� */
	if(KEY_PRESS(GPIOx,GPIO_Pin))  
	{	 
		/*�ȴ������ͷ� */
		while(KEY_PRESS(GPIOx,GPIO_Pin));   
		return 	KEY_ON;	 
	}
	else
		return KEY_OFF;
}

#if 0
uint8_t Key_state(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin)
{
	 static uint8_t key_value = KEY_NULL;
	 static uint16_t longtime;
	 if( (longtime == 0) && (key_value != KEY_NULL))  // ������״̬Ϊ�������߶̰�ʱ����longtimeΪ0������״̬����
	 { 
	     key_value = KEY_NULL;
	 }
	 if ( time == 5 ) /* 5 * 1 ms = 5ms ʱ�䵽 */
   {
        time = 0;
				if(KEY_PRESS(GPIOx,GPIO_Pin))  //��������
	      {
	         longtime++;
//					 if(longtime > SHORT_TIME)
//					 {
//					     longtime = 0;
//						   key_value = KEY_LONG;
//						   return key_value;
//					 }
	      }
        else  //�����ɿ�
				{
				    if((longtime >= 3) && (longtime <= SHORT_TIME))  //�̰�
						{
						    key_value = KEY_SHORT;
						}
						else if( longtime > SHORT_TIME ) // ����
						{
						    key_value = KEY_LONG;
						}
						else  // ���ţ�����
						{
						    key_value = KEY_NULL;
						}
				    longtime = 0; //����
				}					
   }  
	 return key_value;
}
#endif

/*********************************************END OF FILE**********************/
#define KEY_INPUT(GPIOx,GPIO_Pin)           KEY_PRESS(GPIOx,GPIO_Pin)    // ??IO

#define KEY_STATE_0         0       // ????
#define KEY_STATE_1         1
#define KEY_STATE_2         2
#define KEY_STATE_3         3

#define LONG_KEY_TIME       300     // LONG_KEY_TIME*10MS = 3S
#define SINGLE_KEY_TIME     50       // SINGLE_KEY_TIME*10MS = 30MS

#define N_KEY    0                  // no click
#define S_KEY    1                  // single click
#define L_KEY    10                 // long press

unsigned char key_driver(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin) 
{     
    static unsigned char key_state = 0;         
    static unsigned int key_time = 0;           
    unsigned char key_press, key_return; 

    key_return = N_KEY;                         

    key_press = KEY_INPUT(GPIOx, GPIO_Pin);     

    switch (key_state)     
    {       
        case KEY_STATE_0:                       
            if (key_press == KEY_ON)            
            {
                key_time = 0;                   
                key_state = KEY_STATE_1;        
            }        
            break;

        case KEY_STATE_1:                       
            if (key_press == KEY_ON)                     
            {
                key_time++;                     
                if(key_time>=SINGLE_KEY_TIME)   
                {
                    key_state = KEY_STATE_2;    
                }
            }         
            else key_state = KEY_STATE_0;       
            break; 

        case KEY_STATE_2:                       
            if(key_press == KEY_OFF)            
            { 
                 key_return = S_KEY;            
                 key_state = KEY_STATE_0;       
            } 
            else
            {
                key_time++;                     

                if(key_time >= LONG_KEY_TIME)   
                {
                    key_return = L_KEY;         
                    key_state = KEY_STATE_3;    
                }
            }
            break;

      case KEY_STATE_3:                         
          if (key_press == KEY_OFF) 
          {
              key_state = KEY_STATE_0;          
          }         
          break; 

        default:                                
            key_state = KEY_STATE_0;
            break;
    }

    return key_return;                          
} 

unsigned char key_handle(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin)
{
	unsigned char key_value;
	char tt[32] = {0};
  if ( time >= 10 ) /* 5 * 1 ms = 5ms ʱ�䵽 */
   {
        time = 0;
		    key_value = key_driver(GPIOx,GPIO_Pin); 
	 }
	 return key_value;
}
