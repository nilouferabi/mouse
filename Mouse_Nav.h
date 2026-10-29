/****************************************Copyright (c)****************************************************
** File Name:           Mouse_Nav.h
** Description:         8 邻域路径规划（冲刺阶段用）
**
**   搜索阶段仍然只走 4 向，保证墙壁探测可靠；
**   地图完整后，冲刺阶段用 8 邻域规划抄斜线。
**   斜向通行的安全规则：目标格与两个正交邻格必须都通，且 2x2 方块内部无墙（不切角）。
**
*********************************************************************************************************/

#ifndef __MOUSE_NAV_H
#define __MOUSE_NAV_H

#include "Type.h"

/* mouseSpurt_8_to_goal()/mouseSpurt_8_to() 返回值 */
#define NAV_OK          1       /* 到达目标，且已对齐到正交方向 */
#define NAV_NO_PATH     0       /* 当前地图没有可达路径，可回退 4 向冲刺 */
#define NAV_ABORT       2       /* 规划有路，但运动超时/中止，需停车检查 */

/* 构建以 (gx,gy) 为终点的 8 邻域距离图；成功返回 1 */
uint8  Nav_BuildDistance(uint8 gx, uint8 gy);

/* 读取某格到终点的距离；不可达返回 0xFFFF */
uint16 Nav_Distance(uint8 x, uint8 y);

/* 从当前格选下一步：输出 dx/dy（-1..1）和 8 向编号（0=N,1=NE,...,7=NW） */
uint8  Nav_NextStep(uint8 x, uint8 y, uint8 curDir,
                    int8 *dx, int8 *dy, uint8 *moveDir);

/* 判断从 (x,y) 沿 (dx,dy) 是否能走一步 */
uint8  Nav_CanMove(uint8 x, uint8 y, int8 dx, int8 dy);

/* 从当前坐标走到四个终点格中最近的一个；成功返回 NAV_OK */
uint8  mouseSpurt_8_to_goal(uint8 gx0, uint8 gx1, uint8 gy0, uint8 gy1);

/* 从当前坐标走到指定格 */
uint8  mouseSpurt_8_to(uint8 gx, uint8 gy);

#endif
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
