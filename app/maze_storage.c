/****************************************Copyright (c)****************************************************
** File Name:           maze_storage.c
** Description:         迷宫地图/起点/红外阈值的 Flash 存档
*********************************************************************************************************/

#include "maze_storage.h"
#include "maze.h"
#include "maze_search.h"
#include "motion_control.h"
#include "flash.h"
#include "delay.h"

void Storage_SaveStart(void)
{
    Test_Write(START_SAVE_ADDRESS,GucXStart);
}



void Storage_SaveMap(void)
{
   uint8 x = 0;                                                                                            
   uint8 y = 0;
   uint16 GucMapBlock4[16][16]={0};
   for(x=0;x<MAZETYPE;x++) {                                           
      for(y=0;y<MAZETYPE;y++) {                                         
          GucMapBlock4[x][y] =GucMapBlock[x][y]; 
      }
   } 
  STMFLASH_Write(MAPE_SAVE_ADDRESS,(u16*)GucMapBlock4,TUNE_FLASH_MAP_WORDS);

}




void Storage_LoadMap(void)
{
   uint8 x = 0;                                                                                            
   uint8 y = 0;
   uint16 GucMapBlock4[16][16]={0};
   STMFLASH_Read(MAPE_SAVE_ADDRESS,(u16*)GucMapBlock4,TUNE_FLASH_MAP_WORDS);
   delay(TUNE_DELAY_FLASH_READ);
   for(x=0;x<MAZETYPE;x++) {                                           
      for(y=0;y<MAZETYPE;y++) {                                         
          GucMapBlock[x][y] =(uint8)GucMapBlock4[x][y]; 
      }
   }   
}



void Storage_LoadStart(void)
{
  uint16 start = 0;
 
 // read_fm24LC16(&start,0x80,0xa1,1);
  STMFLASH_Read(START_SAVE_ADDRESS,(u16*)&start,1);
  if(start == TUNE_START_MARK)
  {
    GucXStart   = MAZETYPE - 1;                             /*  修改电脑鼠起点的横坐标      */
    GmcMouse.cX = MAZETYPE - 1; 
  }
}




void Storage_LoadIrThresholds(void)
{    
      uint16    DIS1[10] = {0};
      STMFLASH_Read(IR_SAVE_ADDRESS,(u16 *)DIS1,10);
      delay(TUNE_DELAY_POST_ACTION);
      GusDistance_L_Near=DIS1[0];
      GusDistance_R_Near=DIS1[1];
      GusDistance_L_Mid=DIS1[2];
      GusDistance_R_Mid=DIS1[3]; 
      GusDistance_L_Far=DIS1[4];
      GusDistance_R_Far=DIS1[5];
      GusDistance_FL_Far=DIS1[6];
      GusDistance_FR_Far=DIS1[7];
      GusDistance_FL_Near=DIS1[8];
      GusDistance_FR_Near=DIS1[9];
}

