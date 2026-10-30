/****************************************Copyright (c)****************************************************
** File Name:           Mouse_Led.h
** Description:         状态灯控制接口（PB12）
*********************************************************************************************************/

#ifndef __MOUSE_LED_H
#define __MOUSE_LED_H

#include "Type.h"

typedef enum {
    LED_MODE_OFF = 0,
    LED_MODE_SEARCH,        /* 搜索：慢闪；靠近终点自动快闪 */
    LED_MODE_SPRINT         /* 冲刺：常亮 */
} LED_MODE;

void     Led_Init(void);
void     Led_SetMode(LED_MODE mode);
LED_MODE Led_GetMode(void);
void     Led_Tick(void);    /* 每 1 ms 调用一次 */

#endif
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
