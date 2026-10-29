/****************************************Copyright (c)****************************************************
**                               Guangzhou ZHIYUAN electronics Co.,LTD.
**                                     
**                                 http://www.embedtools.com
**
**--------------File Info---------------------------------------------------------------------------------
** File Name:           Maze.h
** Description:         电脑鼠应用层（迷宫算法 + 状态机）头文件
**
*********************************************************************************************************/


#ifndef __Maze_h
#define __Maze_h


/*********************************************************************************************************
  包含头文件
*********************************************************************************************************/
#include "Type.h"
#include "Micromouse.h"
#include "Mouse_Config.h"

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
  应用层内部函数（定义在 main.c）
*********************************************************************************************************/
void  delay(uint32 uiD);
void  mapStepEdit(int8 cX, int8 cY);
void  mapStepEdithong(int8 cX, int8 cY);
void  objectGoTo1(int8 cXdst, int8 cYdst);
void  floodMethodnew(int8 cXdst, int8 cYdst);
void  centralMethodnew(void);
void  goalwall(void);
void  floodwall(void);
void  StartSave(void);
void  wallsave(void);
void  wallget(void);
void  StartGet(void);
void  __ir_Get(void);
void  uart_sendchar(unsigned char ch);
void  SendStr(const char *s);
void  Sendmap(void);

/*********************************************************************************************************
  外部变量（定义在 Mouse_Drive.c；跨中断共享的变量必须 volatile）
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

/* 红外阈值：可由 Flash 覆盖，默认值见 Mouse_Tuning.h */
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
