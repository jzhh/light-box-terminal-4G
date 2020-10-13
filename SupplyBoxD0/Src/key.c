#include "key.h"  

extern uint32_t time;

/**
  * @brief  配置按键用到的I/O口
  * @param  无
  * @retval 无
  */
#if 0 
void Key_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/*开启按键端口的时钟*/
	RCC_APB2PeriphClockCmd(KEY1_GPIO_CLK|KEY2_GPIO_CLK,ENABLE);
	
	//选择按键的引脚
	GPIO_InitStructure.GPIO_Pin = KEY1_GPIO_PIN; 
	// 设置按键的引脚为浮空输入
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
	//使用结构体初始化按键
	GPIO_Init(KEY1_GPIO_PORT, &GPIO_InitStructure);
	
	//选择按键的引脚
	GPIO_InitStructure.GPIO_Pin = KEY2_GPIO_PIN; 
	//设置按键的引脚为浮空输入
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
	//使用结构体初始化按键
	GPIO_Init(KEY2_GPIO_PORT, &GPIO_InitStructure);	
}
#endif

 /*
 * 函数名：Key_Scan
 * 描述  ：检测是否有按键按下
 * 输入  ：GPIOx：x 可以是 A，B，C，D或者 E
 *		     GPIO_Pin：待读取的端口位 	
 * 输出  ：KEY_OFF(没按下按键)、KEY_ON（按下按键）
 */
uint8_t Key_Scan(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin)
{			
	/*检测是否有按键按下 */
	if(KEY_PRESS(GPIOx,GPIO_Pin))  
	{	 
		/*等待按键释放 */
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
	 if( (longtime == 0) && (key_value != KEY_NULL))  // 当按键状态为长按或者短按时，而longtime为0，按键状态清零
	 { 
	     key_value = KEY_NULL;
	 }
	 if ( time == 5 ) /* 5 * 1 ms = 5ms 时间到 */
   {
        time = 0;
				if(KEY_PRESS(GPIOx,GPIO_Pin))  //按键按下
	      {
	         longtime++;
//					 if(longtime > SHORT_TIME)
//					 {
//					     longtime = 0;
//						   key_value = KEY_LONG;
//						   return key_value;
//					 }
	      }
        else  //按键松开
				{
				    if((longtime >= 3) && (longtime <= SHORT_TIME))  //短按
						{
						    key_value = KEY_SHORT;
						}
						else if( longtime > SHORT_TIME ) // 长按
						{
						    key_value = KEY_LONG;
						}
						else  // 干扰，触动
						{
						    key_value = KEY_NULL;
						}
				    longtime = 0; //清零
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
  if ( time >= 10 ) /* 5 * 1 ms = 5ms 时间到 */
   {
        time = 0;
		    key_value = key_driver(GPIOx,GPIO_Pin); 
	 }
	 return key_value;
}
