/****************************************Copyright (c)****************************************************
**                               Guangzhou ZHIYUAN electronics Co.,LTD.
**                                     
**                                 http://www.embedtools.com
**
**--------------File Info---------------------------------------------------------------------------------
** File Name:           Maze.h
** Last modified Date: 
** Last Version: 
** Description:         电脑鼠顶层控制程序头文件
** 
**--------------------------------------------------------------------------------------------------------
** Created By: 
** Created date: 
** Version: 
** Descriptions: 
**
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
** Version:
** Description:
**
*********************************************************************************************************/


#ifndef __Maze_h
#define __Maze_h


/*********************************************************************************************************
  包含头文件
*********************************************************************************************************/
#include "Zlg7289.h"
#include "Type.h"
#include "Micromouse.h"
#include "Mouse_Config.h"
/*********************************************************************************************************
  常量宏定义 -- 定义电脑鼠的四种状态
*********************************************************************************************************/
#define  WAIT           0                                               /*  等待状态                    */
#define  START          1                                               /*  启动状态                    */
#define  MAZESEARCH     2                                               /*  搜寻状态                    */
#define  SPURT          3                                               /*  冲刺状态                    */
#define  RESTART        4                                               /*  重新启动状态                    */
#define  SPURTL         5                                               /*  冲刺状态                    */
#define  RESTARTL       6                                               /*  重新启动状态                    */
#define  SPURTLL         7                                               /*  冲刺状态                    */
#define  SPURT45         8 
/*********************************************************************************************************
  申明需要使用的外部函数
*********************************************************************************************************/
extern void  mouseInit(void);                                           /*  底层驱动程序初始化          */
extern void  mouseGoahead(int8  cNBlock);                               /*  前进N格                     */
extern void  mazeSearch(void);                                          /*  迷宫搜索                    */
extern void  mouseTurnleft(void);                                       /*  向左转90度                  */
extern void  mouseTurnright(void);                                      /*  向右转90度                  */
extern void  mouseTurnback(void);                                       /*  向后转                      */
extern uint8 keyCheck(void);                                            /*  检测按键                    */
extern void  sensorDebug(void);                                         /*  传感器调试                  */
extern void  mouseStop(void);
extern void  __keyIntDisable (void);
extern void  mouseGoahead_Llow (int8  cNBlock);
extern void __UART0Init (void);
extern void mouseTurnleft_C(void);                                               /*  向左转90度                  */
extern void mouseTurnright_C(void);                                              /*  向右转90度                  */
extern void mouseTurnback_Y(void);                                               /*  向后转                      */
extern uint8 startCheck (void);
extern uint32 read_fm24LC16(uint32 *array,uint32 address,uint32 control,uint32 len);
extern void PIDInit(void);
void floodMethodnew(int8 cXdst, int8 cYdst);
void cornerMethodgo(void);
void centralMethodnew(void);
void Sendmap();
void Diagonalway(unsigned char cXdst,unsigned char cYdst);
void objectGoTo_45(uint8 ucStep,uint8  cXdst, uint8  cYdst);
void mouseSpurt_45(void);
void uart_sendchar(unsigned char ch);
/*********************************************************************************************************
  申明需要使用的外部变量
*********************************************************************************************************/
extern MAZECOOR GmcMouse;                                               /*  GmcMouse.x :电脑鼠横坐标    */
                                                                        /*  GmcMouse.y :电脑鼠纵坐标    */
                                                                        
extern uint8    GucMouseDir;                                            /*  电脑鼠的前进方向            */
extern uint8    GucMapBlock[MAZETYPE][MAZETYPE];                        /*  GucMapBlock[x][y]           */
                                                                         /*  x,横坐标;y,纵坐标;          */
                                                       /*  bit3~bit0分别代表左下右上   */
extern uint8    map;                                                                        /*  0:该方向无路，1:该方向有路  */
extern uint8    GucYiBaiBa;                   
extern uint8    GucDirTemp;
extern uint8    GucMapBlock1[MAZETYPE][MAZETYPE];  
extern uint8    GucMapBlock0[MAZETYPE][MAZETYPE];  /*存洪水时墙壁资料*/
extern uint8    GucMouseStart;
extern uint8    GucFrontJinju;
extern uint8    GucCrossroad;

extern uint16   GusFreq_F;  
extern uint16   GusFreq_FJ;      
extern uint16   GusFreq_X;      
extern uint16   GusFreq_LF;           
extern uint16   GusFreq_L;                
//红外的设定值，最后改成数组形式
extern uint16 GusDistance_L_Near; //左红外近
extern uint16 GusDistance_L_Mid;  //左红外中
extern uint16 GusDistance_L_Far;  //左红外远
extern uint16 GusDistance_R_Near; //右红外近
extern uint16 GusDistance_R_Mid;  //右红外中
extern uint16 GusDistance_R_Far;  //右红外近
extern uint16 GusDistance_FL_Near;  //近距，用来判断停止 
extern uint16 GusDistance_FR_Near;
extern uint16 GusDistance_FL_Far;   //远距用来判断墙壁 
extern uint16 GusDistance_FR_Far;
static void  mapStepEdit(int8  cX, int8  cY);
static void  mouseSpurt(void);
static void  objectGoTo(int8  cXdst, int8  cYdst);
static void  objectGoTo_hui(int8  cXdst, int8  cYdst);
static void  objectGoTo_liang(int8  cXdst, int8  cYdst);
static void  objectGoTo_L(int8  cXdst, int8  cYdst);
static uint8 mazeBlockDataGet(uint8  ucDirTemp);
static void  rightMethod(void);
static void  leftMethod(void);
static void  frontRightMethod(void);
static void  frontLeftMethod(void);
static void  centralMethod(void);
void actiongenerate();

#endif
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
