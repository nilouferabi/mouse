/****************************************Copyright (c)****************************************************
** File Name:           path_4dir.h
** Description:         4 向等高图路径执行（搜索/回程）
*********************************************************************************************************/

#ifndef __PATH_4DIR_H
#define __PATH_4DIR_H

#include "types.h"

void objectGoTo1(int8 cXdst, int8 cYdst);
uint8 Path4_CanReachGoal(void);
void mouseSpurt_CC(void);

#endif
