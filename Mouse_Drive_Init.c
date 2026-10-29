#include "Mouse_Drive_Init.h"
#include "Mouse_Config.h"
/*********************************************************************************************************
  定义全局变量
*********************************************************************************************************/
#define SYS_CLK   72000000
#define ADC1_DR_Address   ((u32)0x40012400+0x4c)
__IO uint16_t ADC_ConvertedValue[TUNE_ADC_CHANNELS];
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

void IR_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
        
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_5; //IR1
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruct);	
        
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_13;//IR2
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_3; //IR3
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruct);	
	
        GPIO_InitStruct.GPIO_Pin=GPIO_Pin_2;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStruct);	
        
	GPIO_ResetBits(GPIOC,GPIO_Pin_2|GPIO_Pin_13);
        GPIO_ResetBits(GPIOA,GPIO_Pin_3|GPIO_Pin_5);
}

void AD_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin= GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_5;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AIN;
	GPIO_Init(GPIOC,&GPIO_InitStruct);
        
        GPIO_InitStruct.GPIO_Pin= GPIO_Pin_4|GPIO_Pin_2;
        GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AIN;
        GPIO_Init(GPIOA,&GPIO_InitStruct);  
}

void AD_Mode_Config(void)
{
	ADC_InitTypeDef ADC_InitStruct;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	
	ADC_InitStruct.ADC_Mode=ADC_Mode_Independent;//ADC1和ADC2独立工作模式
	ADC_InitStruct.ADC_ScanConvMode=ENABLE; //ENABLE;多通道模式，DISABLE;单通道模式
	ADC_InitStruct.ADC_ContinuousConvMode=ENABLE;  //ENABLE 连续采样 DISABLE;单通次采样
	ADC_InitStruct.ADC_ExternalTrigConv=ADC_ExternalTrigConv_None;//采样由软件驱动
	ADC_InitStruct.ADC_DataAlign=ADC_DataAlign_Right;//数据右对齐
	ADC_InitStruct.ADC_NbrOfChannel=TUNE_ADC_CHANNELS;  //进行规则转换的ADC数目
	
	ADC_Init(ADC1,&ADC_InitStruct);
	RCC_ADCCLKConfig(RCC_PCLK2_Div8);//adc时钟频率=PLCK2/8=9M
	ADC_RegularChannelConfig(ADC1,ADC_Channel_2,1,ADC_SampleTime_71Cycles5);//9M/239.5=37.58k 26.61us进行一次转换
	ADC_RegularChannelConfig(ADC1,ADC_Channel_10,2,ADC_SampleTime_71Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_15,3,ADC_SampleTime_71Cycles5);        
	ADC_RegularChannelConfig(ADC1,ADC_Channel_11,4,ADC_SampleTime_71Cycles5);
        ADC_RegularChannelConfig(ADC1,ADC_Channel_4,5,ADC_SampleTime_71Cycles5);
	ADC_DMACmd(ADC1,ENABLE);//ADC1 DMA通道使能
	ADC_Cmd(ADC1,ENABLE);   //ADC1使能
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);
}

void AD_DMA_Config(void)
{
	DMA_InitTypeDef DMA_InitStruct;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
	
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStruct.DMA_PeripheralBaseAddr=ADC1_DR_Address;
	DMA_InitStruct.DMA_MemoryBaseAddr=(u32)(&ADC_ConvertedValue);
	DMA_InitStruct.DMA_DIR=DMA_DIR_PeripheralSRC;
	DMA_InitStruct.DMA_BufferSize=TUNE_ADC_CHANNELS;
	
	DMA_InitStruct.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc=DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize=DMA_PeripheralDataSize_HalfWord;
	DMA_InitStruct.DMA_MemoryDataSize=DMA_MemoryDataSize_HalfWord;
	
	DMA_InitStruct.DMA_Mode=DMA_Mode_Circular;
	DMA_InitStruct.DMA_Priority=DMA_Priority_High;
	DMA_InitStruct.DMA_M2M=DMA_M2M_Disable;
	
	DMA_Init(DMA1_Channel1,&DMA_InitStruct);
	
	DMA_Cmd(DMA1_Channel1,ENABLE);
}

void AD_Init(void)
{
	AD_GPIO_Config();
	AD_DMA_Config();
	AD_Mode_Config();
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
void TIM6_IRInit(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  													
    NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;	  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
    TIM_DeInit(TIM6);
    
    TIM_TimeBaseStructure.TIM_Period = TUNE_TIM6_PERIOD;		 								
    TIM_TimeBaseStructure.TIM_Prescaler = TUNE_TIM6_PRESCALER;		  
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 		
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
	
    TIM_ClearFlag(TIM6, TIM_FLAG_Update);							    	
    TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);
    TIM_Cmd(TIM6, ENABLE);
	 
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
void LED_Init(void)
{
 
 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_Init(GPIOB, &GPIO_InitStructure);
 GPIO_SetBits(GPIOB,GPIO_Pin_12);
}
 void fan_init(u16 arr,u16 psc)	 //TIM4	PB8
 {		 					 
 	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//使能TIM4时钟
        RCC->APB1ENR|=1<<2; 	//TIM4时钟使能    
 	RCC->APB2ENR|=1<<3;    	//Enable IO PB
       	RCC->APB2ENR|=1<<0;   //Enable AFIO Clock  
     
 	GPIOB->CRH&=0XFFFFFFF0;	//Clear the State of PB8
 	GPIOB->CRH|=0X0000000B;	//Set   the State of PB8  	 

 	AFIO->MAPR&=0XFFFFEFFF; //清除MAPR的12位，tim4   数据手册122页  如果修改IO口这里也需要变
 //	AFIO->MAPR|=1<<11;      //部分重影映像,TIM3_CH2->PB5

        TIM4->CCR3 = 0;  //通道3初始化占空比
 	TIM4->ARR=arr;			//设定计数器的自动重装载值 
 	TIM4->PSC=psc;			//预分频器不分频
 	
 	TIM4->CCMR2|=6<<4;  	//CH3PWM2 模式
 	TIM4->CCMR2|=1<<3; 	  //CH3装载使能	 
 // 	TIM4->CCMR1|=7<<12;  	//CH2 PWM模式	 
 // 	TIM4->CCMR1|=1<<11; 	//CH2	 
 	TIM4->CCER  |= 1<<8;   	//OC3 输出使能
 // 	TIM4->CCER|=1<<4;   	//OC2 输出使能	   
 	TIM4->CR1=0x0080;   	//ARPE使能 
 	TIM4->CR1|=0x01;    	//使能定时器4											  
 }  
void MouseInit(void)
{
	SysTick_Configuration();
        IR_GPIO_Config();
	Motor_Mode_Config();
	ENCL_Init();
	ENCR_Init();
        __keyInit ();
        AD_Init();
        TIM6_IRInit();
        LED_Init();
        fan_init(TUNE_FAN_PWM_PERIOD, 0);
	//PIDInit();
}


void  __keyIntDisable (void)
{
     EXTI->IMR &= ~(1<<3);
    //GPIOPinIntDisable(GPIO_PORTA_BASE, ZLG7289_KEY);
}
