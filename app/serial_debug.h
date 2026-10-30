/****************************************Copyright (c)****************************************************
** File Name:           serial_debug.h
** Description:         调试串口接口
*********************************************************************************************************/

#ifndef __SERIAL_DEBUG_H
#define __SERIAL_DEBUG_H

#include "types.h"

extern uint8 Rece;

void Serial_Init(void);
void Serial_SendChar(uint8 ch);
void Serial_SendStr(const char *s);
void Serial_SendMap(void);
void Serial_SendBlock(void);

#endif
