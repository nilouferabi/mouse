/****************************************Copyright (c)****************************************************
**                               Guangzhou ZHIYUAN electronics Co.,LTD.
**                                     
**                                 http://www.embedtools.com
**
**--------------File Info---------------------------------------------------------------------------------
** File Name:           maze.h
** Description:         电脑鼠共享数据、状态常量与主程序接口
**
*********************************************************************************************************/


#ifndef __MAZE_H
#define __MAZE_H


/*********************************************************************************************************
  包含头文件
*********************************************************************************************************/
#include "types.h"
#include "micromouse.h"
#include "mouse_config.h"
#include "maze_search.h"

/*********************************************************************************************************
  电脑鼠状态机
*********************************************************************************************************/
#define  WAIT           0                                               /*  等待状态                    */
#define  START          1                                               /*  启动状态                    */
#define  MAZESEARCH     2                                               /*  搜寻状态                    */
#define  SPURT          3                                               /*  冲刺状态                    */
#define  RESTART        4                                               /*  重新启动状态                    */
#define  SPURTL         5                                               /*  冲刺状态                    */
#define  RESTARTL       6                                               /*  重新启动状态                    */
#define  SPURTLL         7                                              /*  冲刺状态                    */
#define  SPURT45         8


/*********************************************************************************************************
  外部变量（定义在 motion_control.c；跨中断共享的变量必须 volatile）
*********************************************************************************************************/
extern MAZECOOR GmcMouse;
extern uint8    GucMouseDir;
extern uint8    GucMapBlock[MAZETYPE][MAZETYPE];
extern uint8    GucMapBlock0[MAZETYPE][MAZETYPE];
extern uint8    GucMapBlock1[MAZETYPE][MAZETYPE];
extern uint8    GucDirTemp;
extern uint8    GucCrossroad;

extern volatile uint8  GucFrontJinju;
extern volatile uint8  GucGoHead;

/* 红外阈值：可由 Flash 覆盖，默认值见 tuning.h */
extern volatile uint16 GusDistance_L_Near;
extern volatile uint16 GusDistance_L_Mid;
extern volatile uint16 GusDistance_L_Far;
extern volatile uint16 GusDistance_R_Near;
extern volatile uint16 GusDistance_R_Mid;
extern volatile uint16 GusDistance_R_Far;
extern volatile uint16 GusDistance_FL_Near;
extern volatile uint16 GusDistance_FR_Near;
extern volatile uint16 GusDistance_FL_Far;
extern volatile uint16 GusDistance_FR_Far;

#endif
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
