/****************************************Copyright (c)****************************************************
** File Name:           motion_internal.h
** Description:         motion_control.c 与 motion_8dir.c 的内部共享声明
*********************************************************************************************************/

#ifndef __MOTION_INTERNAL_H
#define __MOTION_INTERNAL_H

#include "motion_control.h"

extern __PID __GmWPID;
extern __MOTOR __GmLeft;
extern __MOTOR __GmRight;
extern volatile uint8  __GucMouseState;
extern volatile int32  __GiMaxSpeed;
extern volatile int32  __GiMinSpeed;
extern volatile uint8  GuiSpeedCtr;
extern volatile uint32 GucSysTickMs;
extern volatile float  jiao;
extern volatile uint16 ucIRCheck[4];
extern volatile uint16 GusDistance_FL_Near;
extern volatile uint16 GusDistance_FR_Near;
extern uint8 GucMouseDir;

void __delay(uint32 uiD);
void Motion_ResetPID(void);
void Motion_Stop(void);

#endif
