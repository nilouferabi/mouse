/****************************************Copyright (c)****************************************************
** File Name:           maze_search.h
** Description:         迷宫/寻路算法接口
**
**   等高图、洪水填充、岔路决策、4 向寻路执行都在 maze_search.c。
*********************************************************************************************************/

#ifndef __MAZE_SEARCH_H
#define __MAZE_SEARCH_H

#include "types.h"
#include "mouse_config.h"
#include "path_4dir.h"

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
uint8 crosswayCheck(int8 cX, int8 cY);         /* 统计未走过支路数 */

#endif
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
