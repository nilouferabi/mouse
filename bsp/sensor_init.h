/****************************************Copyright (c)****************************************************
** File Name:           sensor_init.h
** Description:         红外发射/ADC+DMA/TIM6 传感器初始化
*********************************************************************************************************/

#ifndef __SENSOR_INIT_H
#define __SENSOR_INIT_H

#include "stm32f10x.h"
#include "mouse_config.h"

extern __IO uint16_t ADC_ConvertedValue[TUNE_ADC_CHANNELS];

void Sensor_GpioInit(void);
void Sensor_AdcInit(void);
void Sensor_IrqInit(void);
void Sensor_Init(void);

#endif
