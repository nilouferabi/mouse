/****************************************Copyright (c)****************************************************
** File Name:           maze_search_internal.h
** Description:         maze_search.c 与 path_4dir.c 的内部共享声明
*********************************************************************************************************/

#ifndef __MAZE_SEARCH_INTERNAL_H
#define __MAZE_SEARCH_INTERNAL_H

#include "maze_search.h"

extern uint8 GucMapStep[MAZETYPE][MAZETYPE];
void mapStepEdit(int8 cX, int8 cY);

#endif
