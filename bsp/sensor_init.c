/****************************************Copyright (c)****************************************************
** File Name:           sensor_init.c
** Description:         红外发射/ADC+DMA/TIM6 传感器初始化
*********************************************************************************************************/

#include "sensor_init.h"
#include "stm32f10x.h"
#include "mouse_config.h"

#define ADC1_DR_Address   ((u32)0x40012400 + 0x4c)
__IO uint16_t ADC_ConvertedValue[TUNE_ADC_CHANNELS];

void Sensor_GpioInit(void)
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



static void Sensor_AdcGpioConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin= GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_5;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AIN;
	GPIO_Init(GPIOC,&GPIO_InitStruct);
        
        GPIO_InitStruct.GPIO_Pin= GPIO_Pin_4|GPIO_Pin_2;
        GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AIN;
        GPIO_Init(GPIOA,&GPIO_InitStruct);  
}



static void Sensor_AdcModeConfig(void)
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



static void Sensor_AdcDmaConfig(void)
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



void Sensor_AdcInit(void)
{
	Sensor_AdcGpioConfig();
	Sensor_AdcDmaConfig();
	Sensor_AdcModeConfig();
}



void Sensor_IrqInit(void)
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



void Sensor_Init(void)
{
    Sensor_GpioInit();
    Sensor_AdcInit();
    Sensor_IrqInit();
}
