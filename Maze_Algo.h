/****************************************Copyright (c)****************************************************
** File Name:           Maze_Algo.h
** Description:         迷宫/寻路算法接口
**
**   等高图、洪水填充、岔路决策、4 向寻路执行都在 Maze_Algo.c。
*********************************************************************************************************/

#ifndef __MAZE_ALGO_H
#define __MAZE_ALGO_H

#include "Type.h"
#include "Mouse_Config.h"

extern uint8 GucXStart;
extern uint8 GucYStart;

/* 四个终点格组成中心 2x2 */
extern uint8 GucXGoal0;
extern uint8 GucXGoal1;
extern uint8 GucYGoal0;
extern uint8 GucYGoal1;

void  floodwall(void);                         /* 初始化墙图 */
void  goalwall(void);                          /* 目标区域虚拟墙 */
void  centralMethodnew(void);                  /* 中心法则探索到终点 */
void  objectGoTo1(int8 cXdst, int8 cYdst);     /* 按等高图走到目标 */
void  mouseSpurt_CC(void);                     /* 冲刺到最近终点格 */
uint8 crosswayCheck(int8 cX, int8 cY);         /* 统计未走过支路数 */

#endif
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
