#include "board_init.h"
#include "mouse_config.h"
#include "fan.h"
#include "status_led.h"
#include "sensor_init.h"
/*********************************************************************************************************
  定义全局变量
*********************************************************************************************************/
#define SYS_CLK   72000000
/*********************************************************************************************************
** Function name:       SysTick_Configuration
** Descriptions:        系统节拍定时器初始化。
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void SysTick_Configuration(void)
{ 
    if (SysTick_Config(SYS_CLK / TUNE_SYSTICK_HZ))     /* 1 ms */
    {        
        while (1);
    }
    NVIC_SetPriority(SysTick_IRQn, 0x0);
}

/*********************************************************************************************************
** Function name:       __keyInit
** Descriptions:        对 KEY和START键进行初始化
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __keyInit (void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	   
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void Motor_Mode_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruct);

	TIM_DeInit(TIM1);
	
	TIM_TimeBaseInitStruct.TIM_Period = TUNE_PWM_PERIOD - 1; 
	TIM_TimeBaseInitStruct.TIM_Prescaler = 0; 
	TIM_TimeBaseInitStruct.TIM_ClockDivision = 0; 
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStruct); 	
	
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2; 
	TIM_OCInitStruct.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Enable; 
	TIM_OCInitStruct.TIM_OCPolarity=TIM_OCPolarity_High ;
	TIM_OCInitStruct.TIM_OCNPolarity = TIM_OCPolarity_High ; 
	TIM_OCInitStruct.TIM_OCIdleState=TIM_OCIdleState_Set;
	TIM_OCInitStruct.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	
	TIM_OCInitStruct.TIM_Pulse = 0;
		
	TIM_OC1Init(TIM1, &TIM_OCInitStruct); 
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable); 
	
	TIM_OC2Init(TIM1, &TIM_OCInitStruct);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	TIM_OC3Init(TIM1, &TIM_OCInitStruct);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	TIM_OC4Init(TIM1, &TIM_OCInitStruct);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM1, ENABLE); 

	TIM_Cmd(TIM1, ENABLE);
        TIM_SetCompare1(TIM1,0);                                          
        TIM_SetCompare2(TIM1,0);
	TIM_SetCompare4(TIM1,0);                                          
        TIM_SetCompare3(TIM1,0);
	TIM_CtrlPWMOutputs(TIM1, ENABLE); 
}
void ENCL_Init(void)
{
	
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure; 
  TIM_ICInitTypeDef TIM_ICInitStructure;  
  GPIO_InitTypeDef GPIO_InitStructure; 
  NVIC_InitTypeDef NVIC_InitStructure; 
   
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
	
  GPIO_StructInit(&GPIO_InitStructure); 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
    
  NVIC_InitStructure.NVIC_IRQChannel =TIM2_IRQn; 
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
  NVIC_Init(&NVIC_InitStructure); 

  TIM_DeInit(TIM2);  
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
   
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0;             
  TIM_TimeBaseStructure.TIM_Period = 65535;    
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;    
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 
  
  TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); 
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = TUNE_ENCODER_FILTER_L;

  TIM_ICInit(TIM2, &TIM_ICInitStructure); 
   
  TIM_ClearFlag(TIM2,TIM_IT_Update);
  
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); 

   
  TIM_Cmd(TIM2, ENABLE); 
  TIM_SetCounter(TIM2, 0);  
}

void ENCR_Init(void) 
{ 
   TIM_TimeBaseInitTypeDef  TIM_TimeBaseStruct; 
   TIM_ICInitTypeDef TIM_ICInitStruct;  
   GPIO_InitTypeDef GPIO_InitStruct; 
   NVIC_InitTypeDef NVIC_InitStruct; 
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
  GPIO_StructInit(&GPIO_InitStruct); 
	
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; 
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStruct); 
    
  NVIC_InitStruct.NVIC_IRQChannel =TIM3_IRQn; 
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1; 
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0; 
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE; 
  NVIC_Init(&NVIC_InitStruct); 

  TIM_DeInit(TIM3);  
  TIM_TimeBaseStructInit(&TIM_TimeBaseStruct); 
   
  TIM_TimeBaseStruct.TIM_Prescaler =0;          
  TIM_TimeBaseStruct.TIM_Period = 65535-1;   
  TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1; 
  TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;    
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStruct); 
  
  TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); 
  TIM_ICStructInit(&TIM_ICInitStruct);        
  TIM_ICInitStruct.TIM_ICFilter = TUNE_ENCODER_FILTER_R; 
  TIM_ICInit(TIM3, &TIM_ICInitStruct); 
   
  TIM_ClearFlag(TIM3,TIM_IT_Update);
  
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); 

  //TIM_SetCounter(TIM3, 32768);
  TIM_Cmd(TIM3, ENABLE);  
  TIM_SetCounter(TIM3, 0);
} 
void MouseInit(void)
{
	SysTick_Configuration();
        Sensor_GpioInit();
	Motor_Mode_Config();
	ENCL_Init();
	ENCR_Init();
        __keyInit ();
        Sensor_AdcInit();
        Sensor_IrqInit();
        Led_Init();
        Fan_Init();
	//PIDInit();
}


void  __keyIntDisable (void)
{
     EXTI->IMR &= ~(1<<3);
    //GPIOPinIntDisable(GPIO_PORTA_BASE, ZLG7289_KEY);
}

/*********************************************************************************************************
** Function name:       __ir_Get
** Descriptions:        读取E2PROM中的红外频率
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void Clock_Init(void)
{    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | 
                           RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | 
                           RCC_APB2Periph_GPIOE |RCC_APB2Periph_AFIO, ENABLE);
}



void JTAG_Config(u8 mode)
{
    u32 temp;
    temp=mode;
    temp<<=25;
    RCC->APB2ENR|=1<<0;
    AFIO->MAPR&=0XF8FFFFFF;
    AFIO->MAPR|=temp;
}


