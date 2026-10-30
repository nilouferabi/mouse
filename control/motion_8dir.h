/****************************************Copyright (c)****************************************************
** File Name:           motion_8dir.h
** Description:         8 向运动原语：原地转向、沿当前方向走一格
*********************************************************************************************************/

#ifndef __MOTION_8DIR_H
#define __MOTION_8DIR_H

#include "types.h"

uint8 mouseTurnTo(uint8 targetDir);
uint8 mouseGoOneBlock(uint8 dir);

#endif
