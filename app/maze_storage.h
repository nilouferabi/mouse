/****************************************Copyright (c)****************************************************
** File Name:           maze_storage.h
** Description:         迷宫存档接口（Flash）
*********************************************************************************************************/

#ifndef __MAZE_STORAGE_H
#define __MAZE_STORAGE_H

#include "types.h"

void Storage_SaveStart(void);
void Storage_SaveMap(void);
void Storage_LoadMap(void);
void Storage_LoadStart(void);
void Storage_LoadIrThresholds(void);

#endif
