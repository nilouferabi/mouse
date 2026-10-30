/****************************************Copyright (c)****************************************************
**                               
**                                     
**                                 
**
**--------------File Info---------------------------------------------------------------------------------
** File Name:           ZLG7289.h
** Last modified Date:  2014/5/10
** Last Version:        V1.00
** Description:         数码管显示与键盘管理芯片ZLG7289 针对STM32的驱动程序头文件
** 
**--------------------------------------------------------------------------------------------------------
** Created By:          陈杰鸿
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
#ifndef __DISPLAY_H
#define __DISPLAY_H
#include "stm32f10x.h"
#include "bitband.h"
/*********************************************************************************************************
  ZLG7289端口定义
*********************************************************************************************************/       
      
#define       ZLG7289_CS     PBout(4)// PD10
#define       ZLG7289_CLK    PBout(6)// PD11    
#define       ZLG7289_DIO    PBout(5)// PD12    
#define       ZLG7289_KEY    PBout(3)// PD13

/*********************************************************************************************************
  可以供外部使用的函数申明
*********************************************************************************************************/
extern void displayCmd(char  cCmd);                                     /*  执行ZLG7289 纯指令          */
extern void displayCmdDat(u8  cCmd, char  cDat);                     /*  执行ZLG7289 带数据指令      */
extern void displayDownload(u8 ucMod, char cX, char cDp, char cDat); /*  下载数据                    */
extern char displayKey(void);                                           /*  读按键值                    */
extern void ZLG7289Init(void);                                          /*  ZLG7289 初始化              */
extern void ZLG7289_KEY_Init(void);
/*********************************************************************************************************
  ZLG7289用户指令集
*********************************************************************************************************/
#define displayReset()      displayCmd(0xA4)                            /*  复位(清除)指令              */
#define displayTest()       displayCmd(0xBF)                            /*  测试指令                    */
#define displaySHL()        displayCmd(0xA0)                            /*  左移指令                    */
#define displaySHR()        displayCmd(0xA1)                            /*  右移指令                    */
#define displayROL()        displayCmd(0xA2)                            /*  循环左移指令                */
#define displayROR()        displayCmd(0xA3)                            /*  循环右移指令                */

#define displayFlash(cX)    displayCmdDat(0x88, (cX))                   /*  闪烁控制                    */
                                                                        /*  cX的8个位对应数码管的8个位  */
                                                                        /*  0:闪烁  1:不闪烁            */

#define displayHide(cX)     displayCmdDat(0x98, (cX))                   /*  消隐控制                    */
                                                                        /*  cX的8个位对应数码管的8个位  */
                                                                        /*  0:消隐  1:显示              */

#define displaySegOn(cSeg)  displayCmdDat(0xE0, (cSeg))                 /*  段点亮指令                  */
                                                                        /*  8个数码管被看成64个独立LED  */
                                                                        /*  cSeg: 0 ～ 63               */

#define displaySegOff(cSeg) displayCmdDat(0xC0, (cSeg))                 /*  段关闭指令                  */
                                                                        /*  8个数码管被看成64个独立LED  */
                                                                        /*  cSeg: 0 ～ 63               */

#endif
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
