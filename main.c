/****************************************Copyright (c)****************************************************
**                               Guangzhou ZHIYUAN electronics Co.,LTD.
**                                     
**                                 http://www.embedtools.com
**
**--------------File Info---------------------------------------------------------------------------------
** File Name:           maze.c
** Last modified Date:  2007/09/24
** Last Version:        V1.0
** Description:         根据底层程序取得的迷宫信息，经过该智能算法控制电脑鼠的下一状态，并送往底层驱动程
**                      序执行。
** 
**--------------------------------------------------------------------------------------------------------
** Created By:          Liao Maogang
** Created date:        2007/09/08
** Version:             V1.0
** Descriptions: 
**
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
** Version:
** Description:
**                                  在中断中stop
*********************************************************************************************************/


/*********************************************************************************************************
  包含头文件
*********************************************************************************************************/
#include "Maze.h"
#include "stm32f10x.h"
#include "zlg7289.h"
#include "BitBand.h"
#include "Mouse_Drive.h"
#include "stdio.h"
#include "Mouse_Srv.h"
/*********************************************************************************************************
  全局变量定义
*********************************************************************************************************/

extern __PID  __GmSPID;
extern uint8    GucGoHead;
extern uint8    GucGoHead1; 
uint8    GucXStart                           = 0;                /*  起点横坐标                  */
uint8    GucYStart                           = 0;                /*  起点纵坐标                  */
static uint8    GucXGoal0                           = XDST0;            /*  终点X坐标，有两个值         */
static uint8    GucXGoal1                           = XDST1;
static uint8    GucYGoal0                           = YDST0;            /*  终点Y坐标，有两个值         */
static uint8    GucYGoal1                           = YDST1;
static uint8    GucMouseTask                        = WAIT;             /*  状态机，初始状态为等待      */
uint8    GucMapStep[MAZETYPE][MAZETYPE]      = {0xff};           /*  保存各坐标的等高值          */
static uint8    GucMapStep1[MAZETYPE][MAZETYPE]     = {0xff};
static MAZECOOR GmcStack[MAZETYPE * MAZETYPE]       = {0};              /*  在mapStepEdit()中作堆栈使用 */
static MAZECOOR GmcStack1[MAZETYPE * MAZETYPE]      = {0};
static MAZECOOR GmcCrossway[MAZETYPE * MAZETYPE]    = {0};              /*  Main()中暂存未走过支路坐标  */
uint8 Rece=0;

extern uint8 GoHead45_flag;
uint8 turn45_flag=0;

static uint8    GucMouseTurn                        =0;
uint8 ffff=0;
 uint8 LT90L= 1;
 uint8 RT90L= 2;
 uint8 LV90L= 3;
 uint8 RV90L= 4;
 uint8 LT45L= 5;
 uint8 RT45L =6;
 uint8 LV45L =7;
 uint8 RV45L =8;
 uint8 LT135L= 9;
 uint8 RT135L= 10;
 uint8 LV135L =11;
 uint8 RV135L= 12;
 uint8 L180L  =13;
 uint8 R180L  =14;
 uint8 STRAIGHT90L= 15;        //常规直道 标志位
 uint8 STRAIGHT45L =16;        //45度直道 标志位
 uint8 hh=0;
 uint8 zongdianx;
 uint8 zongdiany;
uint8 led_flag;

static uint8  cDirsishiwu;
static   uint8  mmm=0;
static   uint8 nnn=0;
extern unsigned char 	MapDiagonalway[16][16];
extern unsigned char   DiagonalwayCount;
extern unsigned char	DiagonalwayAction[255];
extern unsigned char actionsequence[500];
/*********************************************************************************************************
** Function name:       Delay
** Descriptions:        延时函数
** input parameters:    uiD :延时参数，值越大，延时越久
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void delay (uint32 uiD)
{
    for (; uiD; uiD--);
}

/*********************************************************************************************************
** Function name:       mapStepEdit
** Descriptions:        制作以目标点为起点的等高图
** input parameters:    uiX:    目的地横坐标
**                      uiY:    目的地纵坐标
** output parameters:   GucMapStep[][]:  各坐标上的等高值
** Returned value:      无
*********************************************************************************************************/
void mapStepEdit (int8  cX, int8  cY)
{
    uint8 n         = 0;                              /*  GmcStack[]下标              */
    uint8 ucStep    = 1;                              /*  等高值                      */
    uint8 ucStat    = 0;                              /*  统计可前进的方向数          */
    uint8 i,j;
    
    GmcStack[n].cX  = cX;                              /*  起点X值入栈                 */
    GmcStack[n].cY  = cY;                              /*  起点Y值入栈                 */
    n++;
    /*
     *  初始化各坐标等高值
     */
    for (i = 0; i < MAZETYPE; i++) {
        for (j = 0; j < MAZETYPE; j++) {
            GucMapStep[i][j] = 0xff;
        }
    }
    /*
     *  制作等高图，直到堆栈中所有数据处理完毕
     */
    while (n) {
        GucMapStep[cX][cY] = ucStep++;                                  /*  填入等高值                  */

        /*
         *  对当前坐标格里可前进的方向统计
         */
        ucStat = 0;
        if ((GucMapBlock[cX][cY] & 0x01) &&                    /*  前方有路     存储墙壁信息不变，此为存储墙壁信息               */
            (GucMapStep[cX][cY + 1] > (ucStep))) {             /*  前方等高值大于计划设定值 ，此为存储等高值   */
            ucStat++;                                          /*  可前进方向数加1             */
        }
        if ((GucMapBlock[cX][cY] & 0x02) &&                    /*  右方有路                    */
            (GucMapStep[cX + 1][cY] > (ucStep))) {             /*  右方等高值大于计划设定值    */
            ucStat++;                                          /*  可前进方向数加1             */
        }
        if ((GucMapBlock[cX][cY] & 0x04) &&
            (GucMapStep[cX][cY - 1] > (ucStep))) {
            ucStat++;                                                   /*  可前进方向数加1             */
        }
        if ((GucMapBlock[cX][cY] & 0x08) &&
            (GucMapStep[cX - 1][cY] > (ucStep))) {
            ucStat++;                                                   /*  可前进方向数加1             */
        }
        /*
         *  没有可前进的方向，则跳转到最近保存的分支点
         *  否则任选一可前进方向前进
         */
        if (ucStat == 0) {
            n--;
            cX = GmcStack[n].cX;
            cY = GmcStack[n].cY;
            ucStep = GucMapStep[cX][cY];
        } else {
            if (ucStat > 1) {                                           /*  有多个可前进方向，保存坐标  */
                GmcStack[n].cX = cX;                                    /*  横坐标X值入栈               */
                GmcStack[n].cY = cY;                                    /*  纵坐标Y值入栈               */
                n++;
            }
            /*
             *  任意选择一条可前进的方向前进
             */
            if ((GucMapBlock[cX][cY] & 0x01) &&                         /*  上方有路                    */
                (GucMapStep[cX][cY + 1] > (ucStep))) {                  /*  上方等高值大于计划设定值    */
                cY++;                                                   /*  修改坐标                    */
                continue;
            }
            if ((GucMapBlock[cX][cY] & 0x02) &&                         /*  右方有路                    */
                (GucMapStep[cX + 1][cY] > (ucStep))) {                  /*  右方等高值大于计划设定值    */
                cX++;                                                   /*  修改坐标                    */
                continue;
            }
            if ((GucMapBlock[cX][cY] & 0x04) &&                         /*  下方有路                    */
                (GucMapStep[cX][cY - 1] > (ucStep))) {                  /*  下方等高值大于计划设定值    */
                cY--;                                                   /*  修改坐标                    */
                continue;
            }
            if ((GucMapBlock[cX][cY] & 0x08) &&                         /*  左方有路                    */
                (GucMapStep[cX - 1][cY] > (ucStep))) {                  /*  左方等高值大于计划设定值    */
                cX--;                                                   /*  修改坐标                    */
                continue;
            }
        }
    }
}

void mapStepEdithong (int8  cX, int8  cY)
{
    uint8 n         = 0;                                                /*  GmcStack[]下标              */
    uint8 ucStep    = 1;                                                /*  等高值                      */
    uint8 ucStat    = 0;                                                /*  统计可前进的方向数          */
    uint8 x = 1;
    uint8 i,j;
    
    GmcStack1[n].cX  = cX;                                               /*  起点X值入栈                 */
    GmcStack1[n].cY  = cY;                                               /*  起点Y值入栈                 */
    n++;
    /*
     *  初始化各坐标等高值
     */

    for (i = 0; i < MAZETYPE; i++) {
        for (j = 0; j < MAZETYPE; j++) {
            GucMapStep1[i][j] = 0xff;
        }
    }

    
    /*
     *  制作等高图，直到堆栈中所有数据处理完毕
     */
    while (n) 
    {
        if(x)
        {
            if(DenggaoCheck() == true)
            {
                if(GucFrontJinju)
                {
                    mouseStop();
                    x=0;
                }
            }
        }
        GucMapStep1[cX][cY] = ucStep++;                                  /*  填入等高值                  */

        /*
         *  对当前坐标格里可前进的方向统计
         *  可前进的定义是说所前进方向的步数值比当前坐标上的值大2以上
         */
        ucStat = 0;
        if ((GucMapBlock1[cX][cY] & 0x01) &&               /*  前方有路                    */
            (GucMapStep1[cX][cY + 1] > (ucStep)))          /*  前方等高值大于计划设定值    */
        {                      
            ucStat++;                                      /*  可前进方向数加1             */
        }
        if ((GucMapBlock1[cX][cY] & 0x02) &&               /*  右方有路                    */
            (GucMapStep1[cX + 1][cY] > (ucStep)))          /*  右方等高值大于计划设定值    */
        {                      
            ucStat++;                                      /*  可前进方向数加1             */
        }
        if ((GucMapBlock1[cX][cY] & 0x04) &&
            (GucMapStep1[cX][cY - 1] > (ucStep))) 
        {
            ucStat++;                                                   /*  可前进方向数加1             */
        }
        if ((GucMapBlock1[cX][cY] & 0x08) &&
            (GucMapStep1[cX - 1][cY] > (ucStep))) 
        {
            ucStat++;                                                   /*  可前进方向数加1             */
        }
        /*
         *  没有可前进的方向，则跳转到最近保存的分支点
         *  否则任选一可前进方向前进
         */
        if (ucStat == 0)
        {
            n--;
            cX = GmcStack1[n].cX;
            cY = GmcStack1[n].cY;
            ucStep = GucMapStep1[cX][cY];
        } 
        else 
        {
            if (ucStat > 1)                              /*  有多个可前进方向，保存坐标  */
            {                                           
                GmcStack1[n].cX = cX;                     /*  横坐标X值入栈               */
                GmcStack1[n].cY = cY;                     /*  纵坐标Y值入栈               */
                n++;
            }
            /*
             *  任意选择一条可前进的方向前进
             */
            if ((GucMapBlock1[cX][cY] & 0x01) &&                         /*  上方有路                    */
                (GucMapStep1[cX][cY + 1] > (ucStep)))                    /*  上方等高值大于计划设定值    */
            {                  
                cY++;                                                    /*  修改坐标                    */
                continue;
            }
            if ((GucMapBlock1[cX][cY] & 0x02) &&                         /*  右方有路                    */
                (GucMapStep1[cX + 1][cY] > (ucStep)))                    /*  右方等高值大于计划设定值    */
            {                   
                cX++;                                                    /*  修改坐标                    */
                continue;
            }
            if ((GucMapBlock1[cX][cY] & 0x04) &&                         /*  下方有路                    */
                (GucMapStep1[cX][cY - 1] > (ucStep)))                    /*  下方等高值大于计划设定值    */
            {                  
                cY--;                                                    /*  修改坐标                    */
                continue;
            }
            if ((GucMapBlock1[cX][cY] & 0x08) &&                         /*  左方有路                    */
                (GucMapStep1[cX - 1][cY] > (ucStep)))                    /*  左方等高值大于计划设定值    */
            {                  
                cX--;                                                    /*  修改坐标                    */
                continue;
            }
        }
    }
    GucMouseTurn =1;
}

void mouseSpurt (void)  
{
    uint8 ucTemp = 0xff;
    int8 cXdst = 0,cYdst = 0;
    /*
     *  对终点的四个坐标分别制作等高图
     *  取离起点最近的一个点作为目标点
     */
    if (GucMapBlock[GucXGoal0][GucYGoal0] & 0x0c) {             /*  判断该终点坐标是否有出口    */
        mapStepEdit(GucXGoal0,GucYGoal0);                       /*  制作等高图                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart]) {        /*  保存离起点最近的坐标        */
            cXdst  = GucXGoal0;
            cYdst  = GucYGoal0;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    if (GucMapBlock[GucXGoal0][GucYGoal1] & 0x09) {               /*  判断该终点坐标是否有出口    */
        mapStepEdit(GucXGoal0,GucYGoal1);                         /*  制作等高图                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart]) {          /*  保存离起点最近的坐标        */
            cXdst  = GucXGoal0;
            cYdst  = GucYGoal1;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    if (GucMapBlock[GucXGoal1][GucYGoal0] & 0x06) {                     /*  判断该终点坐标是否有出口    */
        mapStepEdit(GucXGoal1,GucYGoal0);                               /*  制作等高图                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart]) {                /*  保存离起点最近的坐标        */
            cXdst  = GucXGoal1;
            cYdst  = GucYGoal0;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    if (GucMapBlock[GucXGoal1][GucYGoal1] & 0x03) {                     /*  判断该终点坐标是否有出口    */
        mapStepEdit(GucXGoal1,GucYGoal1);                               /*  制作等高图                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart]) {                /*  保存离起点最近的坐标        */
            cXdst  = GucXGoal1;
            cYdst  = GucYGoal1;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    
    objectGoTo1(cXdst,cYdst);                                            /*  运行到指定目标点            */
    
}

void objectGoTo1(int8  cXdst, int8  cYdst)
{
    uint8 ucStep = 1;
    int8  cNBlock = 0, cDirTemp;
    int8 cX,cY;
    GucCrossroad=0;
    cX = GmcMouse.cX;
    cY = GmcMouse.cY;
    mapStepEdit(cXdst,cYdst);                                           /*  制作等高图                  */
    
    /*
     *  根据等高值向目标点运动，直到达到目的地
     */
    while ((cX != cXdst) || (cY != cYdst)) {
        
        ucStep = GucMapStep[cX][cY];
        /*
         *  任选一个等高值比当前自身等高值小的方向前进
         */
        if ((GucMapBlock[cX][cY] & 0x01) &&                             /*  上方有路                    */
            (GucMapStep[cX][cY + 1] < ucStep)) {                        /*  上方等高值较小              */
            cDirTemp = UP;                                              /*  记录方向                    */
            if (cDirTemp == GucMouseDir) {                              /*  优先选择不需要转弯的方向    */
                cNBlock++;                                              /*  前进一个方格                */
                cY++;
                if((GucMapBlock[cX][cY] & 0x0f)==0x0f)
                  GucCrossroad++;
                continue;                                               /*  跳过本次循环                */
            }
        }
        if ((GucMapBlock[cX][cY] & 0x02) &&                             /*  右方有路                    */
            (GucMapStep[cX + 1][cY] < ucStep)) {                        /*  右方等高值较小              */
            cDirTemp = RIGHT;                                           /*  记录方向                    */
            if (cDirTemp == GucMouseDir) {                              /*  优先选择不需要转弯的方向    */
                cNBlock++;                                              /*  前进一个方格                */
                cX++;
                if((GucMapBlock[cX][cY] & 0x0f)==0x0f)
                  GucCrossroad++;
                continue;                                               /*  跳过本次循环                */
            }
        }
        if ((GucMapBlock[cX][cY] & 0x04) &&                             /*  下方有路                    */
            (GucMapStep[cX][cY - 1] < ucStep)) {                        /*  下方等高值较小              */
            cDirTemp = DOWN;                                            /*  记录方向                    */
            if (cDirTemp == GucMouseDir) {                              /*  优先选择不需要转弯的方向    */
                cNBlock++;                                              /*  前进一个方格                */
                cY--;
                if((GucMapBlock[cX][cY] & 0x0f)==0x0f)
                  GucCrossroad++;
                continue;                                               /*  跳过本次循环                */
            }
        }
        if ((GucMapBlock[cX][cY] & 0x08) &&                             /*  左方有路                    */
            (GucMapStep[cX - 1][cY] < ucStep)) {                        /*  左方等高值较小              */
            cDirTemp = LEFT;                                            /*  记录方向                    */
            if (cDirTemp == GucMouseDir) {                              /*  优先选择不需要转弯的方向    */
                cNBlock++;                                              /*  前进一个方格                */
                cX--;
                if((GucMapBlock[cX][cY] & 0x0f)==0x0f)
                  GucCrossroad++;
                continue;                                               /*  跳过本次循环                */
            }
        }
        
        cDirTemp = (cDirTemp + 8 - GucMouseDir)%8;                      /*  计算方向偏移量              */
        GucDirTemp = cDirTemp;
        if (cNBlock) {
          if((GucCrossroad <= 1)&&(cNBlock>1))
              mouseGoahead_L(cNBlock);                                      /*  前进cNBlock步               */
          else{
            mouseGoahead_L(cNBlock);
            GucCrossroad = 0;
          }
          
          
        }        
        cNBlock = 0;  
        
         
       switch (cDirTemp) {

        case 2:
            mouseTurnright_C();
            break;

        case 4:
            mouseTurnback();
            break;

        case 6:
            mouseTurnleft_C();
            break;

        default:
            break;
        }
      GmcMouse.cX=cX;
      GmcMouse.cY=cY;
    }
    /*
     *  判断任务是否完成，否则继续前进
     */
    
      if (cNBlock) {
          if((GucCrossroad <= 1)&&(cNBlock>1))
            mouseGoahead_L(cNBlock);      //                                /*  前进cNBlock步               */
          else{
            mouseGoahead_L(cNBlock);
            GucCrossroad = 0;
          }
          GmcMouse.cX=cX;
          GmcMouse.cY=cY;
      }
}


char chongci=0;
void mouseSpurt_CC (void)
{
    uint8 ucTemp = 0xff;
    int8 cXdst = 0,cYdst = 0;
    //  对终点的四个坐标分别制作等高图
    //  取离起点最近的一个点作为目标点
    if (GucMapBlock[GucXGoal0][GucYGoal0] & 0x0c)                       /*  判断该终点坐标是否有出口    */
    {                     
        mapStepEdit(GucXGoal0,GucYGoal0);                               /*  制作等高图                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart])                  /*  保存离起点最近的坐标        */
        {                
            cXdst  = GucXGoal0;
            cYdst  = GucYGoal0;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    if (GucMapBlock[GucXGoal0][GucYGoal1] & 0x09)                       /*  判断该终点坐标是否有出口    */
    {                     
        mapStepEdit(GucXGoal0,GucYGoal1);                               /*  制作等高图                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart])                  /*  保存离起点最近的坐标        */
        {                
            cXdst  = GucXGoal0;
            cYdst  = GucYGoal1;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    if (GucMapBlock[GucXGoal1][GucYGoal0] & 0x06)                       /*  判断该终点坐标是否有出口    */
    {                     
        mapStepEdit(GucXGoal1,GucYGoal0);                               /*  制作等高图                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart])                  /*  保存离起点最近的坐标        */
        {                
            cXdst  = GucXGoal1;
            cYdst  = GucYGoal0;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    if (GucMapBlock[GucXGoal1][GucYGoal1] & 0x03)                       /*  判断该终点坐标是否有出口    */
    {                     
        mapStepEdit(GucXGoal1,GucYGoal1);                               /*  制作等高图                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart])                  /*  保存离起点最近的坐标        */
        {                
            cXdst  = GucXGoal1;
            cYdst  = GucYGoal1;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    
    chongci=1;
    objectGoTo1(cXdst,cYdst);/*  运行到指定目标点            */
    
}



void mouseSpurt_CCC (void)
{
    uint8 ucTemp = 0xff;
    int8 cXdst = 0,cYdst = 0;
    //  对终点的四个坐标分别制作等高图
    //  取离起点最近的一个点作为目标点
    if (GucMapBlock[GucXGoal0][GucYGoal0] & 0x0c)                       /*  判断该终点坐标是否有出口    */
    {                     
        mapStepEdit(GucXGoal0,GucYGoal0);                               /*  制作等高图                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart])                  /*  保存离起点最近的坐标        */
        {                
            cXdst  = GucXGoal0;
            cYdst  = GucYGoal0;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    if (GucMapBlock[GucXGoal0][GucYGoal1] & 0x09)                       /*  判断该终点坐标是否有出口    */
    {                     
        mapStepEdit(GucXGoal0,GucYGoal1);                               /*  制作等高图                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart])                  /*  保存离起点最近的坐标        */
        {                
            cXdst  = GucXGoal0;
            cYdst  = GucYGoal1;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    if (GucMapBlock[GucXGoal1][GucYGoal0] & 0x06)                       /*  判断该终点坐标是否有出口    */
    {                     
        mapStepEdit(GucXGoal1,GucYGoal0);                               /*  制作等高图                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart])                  /*  保存离起点最近的坐标        */
        {                
            cXdst  = GucXGoal1;
            cYdst  = GucYGoal0;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    if (GucMapBlock[GucXGoal1][GucYGoal1] & 0x03)                       /*  判断该终点坐标是否有出口    */
    {                     
        mapStepEdit(GucXGoal1,GucYGoal1);                               /*  制作等高图                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart])                  /*  保存离起点最近的坐标        */
        {                
            cXdst  = GucXGoal1;
            cYdst  = GucYGoal1;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    
    chongci=1;
    objectGoTo1(cXdst,cYdst);                                            /*  运行到指定目标点            */
    
}




void mouseSpurt_45(void)
{
    uint8 ucTemp = 0xff;
    uint8 cXdst = 0,cYdst = 0;
    /*
     *  对终点的四个坐标分别制作等高图
     *  取离起点最近的一个点作为目标点  
     */
    if (GucMapBlock[GucXGoal0][GucYGoal0] & 0x0c) {                     /*  判断该终点坐标是否有出口    */
        mapStepEdit(GucXGoal0,GucYGoal0);                               /*  制作等高图                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart]) {                /*  保存离起点最近的坐标        */
            cXdst  = GucXGoal0;
            cYdst  = GucYGoal0;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    if (GucMapBlock[GucXGoal0][GucYGoal1] & 0x09) {                     /*  判断该终点坐标是否有出口    */
        mapStepEdit(GucXGoal0,GucYGoal1);                               /*  制作等高图                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart]) {                /*  保存离起点最近的坐标        */
            cXdst  = GucXGoal0;
            cYdst  = GucYGoal1;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    if (GucMapBlock[GucXGoal1][GucYGoal0] & 0x06) {                     /*  判断该终点坐标是否有出口    */
        mapStepEdit(GucXGoal1,GucYGoal0);                               /*  制作等高图                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart]) {                /*  保存离起点最近的坐标        */
            cXdst  = GucXGoal1;
            cYdst  = GucYGoal0;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    if (GucMapBlock[GucXGoal1][GucYGoal1] & 0x03) {                     /*  判断该终点坐标是否有出口    */
        mapStepEdit(GucXGoal1,GucYGoal1);                               /*  制作等高图                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart]) {                /*  保存离起点最近的坐标        */
            cXdst  = GucXGoal1;
            cYdst  = GucYGoal1;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    mapStepEdit(cXdst,cYdst);                                           /*  制作等高图                  */
    Diagonalway(cXdst,cYdst);    
    actiongenerate();
    GucMouseDir=0;
    objectGoTo_45(ucTemp,cXdst,cYdst);
    GmcMouse.cX=cXdst;
    GmcMouse.cY=cYdst;
    
}



void mouseSpurt_C (void)
{
    uint8 ucTemp = 0xff;
    int8 cXdst = 0,cYdst = 0;
    //  对终点的四个坐标分别制作等高图
    //  取离起点最近的一个点作为目标点
    if (GucMapBlock[GucXGoal0][GucYGoal0] & 0x0c)                       /*  判断该终点坐标是否有出口    */
    {                     
        mapStepEdit(GucXGoal0,GucYGoal0);                               /*  制作等高图                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart])                  /*  保存离起点最近的坐标        */
        {                
            cXdst  = GucXGoal0;
            cYdst  = GucYGoal0;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    if (GucMapBlock[GucXGoal0][GucYGoal1] & 0x09)                       /*  判断该终点坐标是否有出口    */
    {                     
        mapStepEdit(GucXGoal0,GucYGoal1);                               /*  制作等高图                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart])                  /*  保存离起点最近的坐标        */
        {                
            cXdst  = GucXGoal0;
            cYdst  = GucYGoal1;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    if (GucMapBlock[GucXGoal1][GucYGoal0] & 0x06)                       /*  判断该终点坐标是否有出口    */
    {                     
        mapStepEdit(GucXGoal1,GucYGoal0);                               /*  制作等高图                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart])                  /*  保存离起点最近的坐标        */
        {                
            cXdst  = GucXGoal1;
            cYdst  = GucYGoal0;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    if (GucMapBlock[GucXGoal1][GucYGoal1] & 0x03)                       /*  判断该终点坐标是否有出口    */
    {                     
        mapStepEdit(GucXGoal1,GucYGoal1);                               /*  制作等高图                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart])                  /*  保存离起点最近的坐标        */
        {                
            cXdst  = GucXGoal1;
            cYdst  = GucYGoal1;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    
    objectGoTo_CHui(cXdst,cYdst);     //objectGoTo_Hui(cXdst,cYdst);                  /*  运行到指定目标点            */
    
}




extern int getsum(int num);

uint8 xiexianyige=0;
int action=0;
void objectGoTo_45(uint8 ucStep,uint8  cXdst, uint8  cYdst)
{  
   int i=0,num=0;
    int a=0;   
 

    /*
     *  根据等高值向目标点运动，直到达到目的地
     */


    		while(actionsequence[action]!=0)
		{
                 // xiexianyige=1;
			if(actionsequence[action]==STRAIGHT45L)
			{	
				num=0;
				while(actionsequence[action]==STRAIGHT45L)
				{
				 	num++;
					action++;
				}
                                 mouseGoaheadhui_45(num);                            
			}
			else if(actionsequence[action]==STRAIGHT90L)
			{	
				num=0;
				while(actionsequence[action]==STRAIGHT90L)
				{
				 	num++;
					action++;
				}
                                
                                mouseGoahead_liang(num);                                 
			}
                       
			else
			{	
                                         switch(actionsequence[action])
                                          {
                     
                                               case 1://LT90L单个左转          
                                                          
                                                         //mouseTurnleft_90_t();  
                                                       mouseTurnleft_90_t();  
                                                       
                                                         ffff=ffff+6;
                                                         //mouseGoaheadhui_kk(1,42000);    
                                                         break;

                                               case 2:// RT90L单个右转            
                                                        ffff=ffff+2;
                                                      
                                                          mouseTurnright_90_t();
                                                           
                                                         //  mouseGoaheadhui_kk(1,42000);   
                                                      
                                                          break;
                                                case 3 ://LV90L   斜线左转90度'       
                                                  ffff=ffff+6;
                                                           GoHead45_flag=2;
                                                           mouseTurnleft_90_v();        
                                                          GoHead45_flag=0;
                                                         break;
                                                case 4://RV90L    斜线右转90度        
                                                 ffff=ffff+2;
                                                         GoHead45_flag=2;
                                                         
                                                         mouseTurnright_90_v(); 
                                                          //mouseTurnright_90_v();        
                                                          GoHead45_flag=0;
                                                         
                                                          break;
                                                 case 5: //LT45L      直线左转45              ok
                                                   ffff=ffff+7;
                                                          GoHead45_flag=2;
                                                          mouseTurnleft_45_t();     
                                                         // mouseTurnleft_45_t();        
                                                          GoHead45_flag=0;
                                                           break;
                                                 case 6://RT45L       直线右转45      ok
                                                   ffff=ffff+1;
                                                   
                                                          GoHead45_flag=2;//关校正
                                                          mouseTurnright_45_t();     
                                                          GoHead45_flag=0;///斜线校正
                                                         
                                                            break;  
                                                 case 7://LV45L        斜线左转45度     
                                                   ffff=ffff+7;
                                                           //mouseTurnleft_45_v();
                                                  // mouseStop();while(1);
                                                  mouseTurnleft_45_v();
                                                          
                                                                                                       ;
                                                          break;  
                                                 case  8://RV45L         斜线右转45度   ok
                                                          ffff=ffff+1; 
                                                         mouseTurnright_45_v(); 
                                                          
                                                         break;

                                               case  9:  //LT135L       直线左转135          
                                                 ffff=ffff+5;
                                                          GoHead45_flag=2;
                                                          //mouseTurnleft_135_t(); 
                                                         mouseTurnleft_135_t(); 
                                                          
                                                          
                                                          GoHead45_flag=0;
                                                          break;
                                                case 10://RT135L        直线右转135      
                                                  ffff=ffff+3;
                                                          GoHead45_flag=2;
                                                          mouseTurnright_135_t();          
                                                          GoHead45_flag=0;
                                                         break;
                                                case 11://LV135L        斜线左转135      
                                                ffff=ffff+5;
                                                          
                                                          mouseTurnleft_135_v();
                                                         
                                                          break;
                                                 case 12: //RV135L      斜线右转135                                                 
                                                         
                                                         ffff=ffff+3;
                                                          //mouseTurnright_135_v();  
                                                        mouseTurnright_135_v();       
                                                         
                                                           break;
                                                           
                                                 case 13:       //左转180          
                                                         ffff=ffff+4;
                                                           mouseTurnleft_180();      
                                                         //mouseTurnleft_180();        
                                                            break;  
                                                 case  14:      //右转180            
                                                        ffff=ffff+4;
                                                        mouseTurnright_180();      
                                                        //  mouseTurnright_180();        
                                                                                                                   
                                                            break;   
                                             
                                                default:
                                                  mouseStop();
                                                  while(1);
                                                             break;
                                              }                                                     
                                        action++;                               
			}
		}
  
}

void mouseSpurtbucang() 
{
   GmcMouse.cX=zongdianx;
  GmcMouse.cY=zongdiany;
 //  GmcMouse.cX=7;
 //  GmcMouse.cY=8;
   GucXStart= 0;              
   GucYStart= 0; 
}
int8 lingshix,lingshiy=0;
uint8 lingshixyflag=1;

extern uint8 final;
void objectGoTo_Hui(int8 cXdst, int8 cYdst)//if的顺序可以修改，第一个if放返回起始点概率最大的方向
{
    uint8 ucStep = 1;
    int8  cNBlock = 0, cDirTemp;
    int8  cX,cY;
    GucCrossroad = 0;
    cX = GmcMouse.cX;
    cY = GmcMouse.cY;
    mapStepEdit(cXdst, cYdst);                                           /*  制作等高图                  */
    
    while ((cX != cXdst) || (cY != cYdst)) 
    {
        ucStep = GucMapStep[cX][cY];
        if ((GucMapBlock[cX][cY] & 0x01) &&                             /*  上方有路                    */
            (GucMapStep[cX][cY + 1] < ucStep))                          /*  上方等高值较小              */
        {                        
            cDirTemp = UP;                                              /*  记录方向                    */
            if (cDirTemp == GucMouseDir)                                /*  优先选择不需要转弯的方向    */
            {                              
                cNBlock++;                                              /*  前进一个方格                */
                cY++;
                if((GucMapBlock[cX][cY] & 0x0f) == 0x0f)
                    GucCrossroad++;
                continue;                                               /*  跳过本次循环                */
            }
        }
        if ((GucMapBlock[cX][cY] & 0x02) &&                             /*  右方有路                    */
            (GucMapStep[cX + 1][cY] < ucStep))                          /*  右方等高值较小              */
        {                        
            cDirTemp = RIGHT;                                           /*  记录方向                    */
            if (cDirTemp == GucMouseDir)                                /*  优先选择不需要转弯的方向    */
            {                             
                cNBlock++;                                              /*  前进一个方格                */
                cX++;
                if((GucMapBlock[cX][cY] & 0x0f) == 0x0f)
                    GucCrossroad++;
                continue;                                               /*  跳过本次循环                */
            }
        }
        if ((GucMapBlock[cX][cY] & 0x04) &&                             /*  下方有路                    */
            (GucMapStep[cX][cY - 1] < ucStep))                          /*  下方等高值较小              */
        {                        
            cDirTemp = DOWN;                                            /*  记录方向                    */
            if (cDirTemp == GucMouseDir)                                /*  优先选择不需要转弯的方向    */
            {                              
                cNBlock++;                                              /*  前进一个方格                */
                cY--;
                if((GucMapBlock[cX][cY] & 0x0f) == 0x0f)
                    GucCrossroad++;
                continue;                                               /*  跳过本次循环                */
            }
        }
        if ((GucMapBlock[cX][cY] & 0x08) &&                             /*  左方有路                    */
            (GucMapStep[cX - 1][cY] < ucStep))                          /*  左方等高值较小              */
        {                        
            cDirTemp = LEFT;                                            /*  记录方向                    */
            if (cDirTemp == GucMouseDir)                                /*  优先选择不需要转弯的方向    */
            {                              
                cNBlock++;                                              /*  前进一个方格                */
                cX--;
                if((GucMapBlock[cX][cY] & 0x0f) == 0x0f)
                    GucCrossroad++;
                continue;                                               /*  跳过本次循环                */
            }
        }
        cDirTemp = (cDirTemp + 8 - GucMouseDir)%8;                      /*  计算方向偏移量              */
        GucDirTemp = cDirTemp;
        if (cNBlock) 
        {
          
                mouseGoahead(cNBlock);                                      /*  前进cNBlock步               */
         
        }        
        cNBlock = 0;  
        /*  任务清零                    */
        
       switch (cDirTemp) 
       {
           case 1:    
               mouseTurnright_Hui();
               break;

           case 2:
               mouseTurnback();
               break;

           case 3:
               mouseTurnleft_Hui();
               break;

           default:
               break;
        }
        GmcMouse.cX=cX;
        GmcMouse.cY=cY;
    }
    
    if (cNBlock) 
    {
            mouseGoahead(cNBlock);                                  //    前进cNBlock步      
       
        GmcMouse.cX=cX;
        GmcMouse.cY=cY;
    }
}

/*********************************************************************************************************
** Function name:       mazeBlockDataGet
** Descriptions:        根据电脑鼠的相对方向，取出该方向上迷宫格的墙壁资料
** input parameters:    ucDir: 电脑鼠的相对方向
** output parameters:   无
** Returned value:      GucMapBlock[cX][cY] : 墙壁资料
*********************************************************************************************************/
uint8 mazeBlockDataGet (uint8  ucDirTemp)
{
    int8 cX = 0,cY = 0;
    
    /*
     *  把电脑鼠的相对方向转换为绝对方向
     */
    switch (ucDirTemp) {

    case MOUSEFRONT:
        ucDirTemp = GucMouseDir;
        break;

    case MOUSELEFT:
        ucDirTemp = (GucMouseDir + 6) % 8;
        break;

    case MOUSERIGHT:
        ucDirTemp = (GucMouseDir + 2) % 8;
        break;

    default:
        break;
    }
    
    /*
     *  根据绝对方向计算该方向上相邻格的坐标
     */
    switch (ucDirTemp) {

    case 0:
        cX = GmcMouse.cX;
        cY = GmcMouse.cY + 1;
        break;
        
    case 2:
        cX = GmcMouse.cX + 1;
        cY = GmcMouse.cY;
        break;
        
    case 4:
        cX = GmcMouse.cX;
        cY = GmcMouse.cY - 1;
        break;
        
    case 6:
        cX = GmcMouse.cX - 1;
        cY = GmcMouse.cY;
        break;
        
    default:
        break;
    }
    
    return(GucMapBlock0[cX][cY]);                                        /*  返回迷宫格上的资料          */
}
/*********************************************************************************************************
** Function name:       rightMethod
** Descriptions:        右手法则，优先向右前进
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void rightMethod (void)
{
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_R) &&         /*  电脑鼠的右边有路            */
        (mazeBlockDataGet(MOUSERIGHT) == 0x00)) {                       /*  电脑鼠的右边没有走过        */
        mouseTurnright();                                               /*  电脑鼠右转                  */
        //mouseStop();while(1);
        return;
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_F) &&         /*  电脑鼠的前方有路            */
        (mazeBlockDataGet(MOUSEFRONT) == 0x00)) {                       /*  电脑鼠的前方没有走过        */
        return;                                                         /*  电脑鼠不用转弯              */
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_L) &&         /*  电脑鼠的左边有路            */
        (mazeBlockDataGet(MOUSELEFT ) == 0x00)) {                       /*  电脑鼠的左边没有走过        */
        mouseTurnleft();                                                /*  电脑鼠左转                  */
        return;
    }
}
/*********************************************************************************************************
** Function name:       leftMethod
** Descriptions:        左手法则，优先向左运动
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void leftMethod (void)
{
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_L) &&         /*  电脑鼠的左边有路            */
        (mazeBlockDataGet(MOUSELEFT ) == 0x00)) {                       /*  电脑鼠的左边没有走过        */
        mouseTurnleft();                                                /*  电脑鼠左转                  */
        return;
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_F) &&         /*  电脑鼠的前方有路            */
        (mazeBlockDataGet(MOUSEFRONT) == 0x00)) {                       /*  电脑鼠的前方没有走过        */
        return;                                                         /*  电脑鼠不用转弯              */
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_R) &&         /*  电脑鼠的右边有路            */
        (mazeBlockDataGet(MOUSERIGHT) == 0x00)) {                       /*  电脑鼠的右边没有走过        */
        mouseTurnright();                                               /*  电脑鼠右转                  */
        return;
    }
}
/*********************************************************************************************************
** Function name:       frontRightMethod
** Descriptions:        中右法则，优先向前运行，其次向右
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void frontRightMethod (void)
 {
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_F) &&         /*  电脑鼠的前方有路            */
        (mazeBlockDataGet(MOUSEFRONT) == 0x00)) {                       /*  电脑鼠的前方没有走过        */
        
        return;                                                         /*  电脑鼠不用转弯              */
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_R) &&         /*  电脑鼠的右边有路            */
        (mazeBlockDataGet(MOUSERIGHT) == 0x00)) {                       /*  电脑鼠的右边没有走过        */
        mouseTurnright();                                               /*  电脑鼠右转                  */
        return;
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_L) &&         /*  电脑鼠的左边有路            */
        (mazeBlockDataGet(MOUSELEFT ) == 0x00)) {                       /*  电脑鼠的左边没有走过        */
        mouseTurnleft();                                                /*  电脑鼠左转                  */
        return;
    }
}
/*********************************************************************************************************
** Function name:       frontLeftMethod
** Descriptions:        中左法则，优先向前运行，其次向左
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void frontLeftMethod (void)
{
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_F) &&         /*  电脑鼠的前方有路            */
        (mazeBlockDataGet(MOUSEFRONT) == 0x00)) {                       /*  电脑鼠的前方没有走过        */
        return;                                                         /*  电脑鼠不用转弯              */
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_L) &&         /*  电脑鼠的左边有路            */
        (mazeBlockDataGet(MOUSELEFT ) == 0x00)) {                       /*  电脑鼠的左边没有走过        */
        mouseTurnleft();                                                /*  电脑鼠左转                  */
        return;
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_R) &&         /*  电脑鼠的右边有路            */
        (mazeBlockDataGet(MOUSERIGHT) == 0x00)) {                       /*  电脑鼠的右边没有走过        */
        mouseTurnright();                                               /*  电脑鼠右转                  */
        return;
    }
}

/*********************************************************************************************************
** Function name:       centralMethod
** Descriptions:        中心法则，根据电脑鼠目前在迷宫中所处的位置觉定使用何种搜索法则
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void centralMethod (void)
{
    if (GmcMouse.cX & 0x08) {
        if (GmcMouse.cY & 0x08) {

            /*
             *  此时电脑鼠在迷宫的右上角
             */ 
            switch (GucMouseDir) {
                
            case UP:                                                    /*  当前电脑鼠向上              */
                leftMethod();                                           /*  左手法则                    */
                break;

            case RIGHT:                                                 /*  当前电脑鼠向右              */
                rightMethod();                                          /*  右手法则                    */
                break;

            case DOWN:                                                  /*  当前电脑鼠向下              */
                frontRightMethod();                                     /*  中右法则                    */
                break;

            case LEFT:                                                  /*  当前电脑鼠向左              */
                frontLeftMethod();                                      /*  中左法则                    */
                break;

            default:
                break;
            }
        } else {

            /*
             *  此时电脑鼠在迷宫的右下角
             */    
            switch (GucMouseDir) {
                
            case UP:                                                    /*  当前电脑鼠向上              */
                frontLeftMethod();                                      /*  中左法则                    */
                break;

            case RIGHT:                                                 /*  当前电脑鼠向右              */
                leftMethod();                                           /*  左手法则                    */
                break;

            case DOWN:                                                  /*  当前电脑鼠向下              */
                rightMethod();                                          /*  右手法则                    */
                break;

            case LEFT:                                                  /*  当前电脑鼠向左              */
                frontRightMethod();                                     /*  中右法则                    */
                break;

            default:
                break;
            }
        }
    } else {
        if (GmcMouse.cY & 0x08) {

            /*
             *  此时电脑鼠在迷宫的左上角
             */    
            switch (GucMouseDir) {
                
            case UP:                                                    /*  当前电脑鼠向上              */
                rightMethod();                                          /*  右手法则                    */
                break;

            case RIGHT:                                                 /*  当前电脑鼠向右              */
                frontRightMethod();                                     /*  中右法则                    */
                break;

            case DOWN:                                                  /*  当前电脑鼠向下              */
                frontLeftMethod();                                      /*  中左法则                    */
                break;

            case LEFT:                                                  /*  当前电脑鼠向左              */
                leftMethod();                                           /*  左手法则                    */
                break;

            default:
                break;
            }
        } else {

            /*
             *  此时电脑鼠在迷宫的左下角
             */    
            switch (GucMouseDir) {
                
            case UP:                                                    /*  当前电脑鼠向上              */
                frontRightMethod();                                     /*  中右法则                    */
                break;

            case RIGHT:                                                 /*  当前电脑鼠向右              */
                frontLeftMethod();                                      /*  中左法则                    */
                break;

            case DOWN:                                                  /*  当前电脑鼠向下              */
                leftMethod();                                           /*  左手法则                    */
                break;

            case LEFT:                                                  /*  当前电脑鼠向左              */
                rightMethod();                                          /*  右手法则                    */
                break;

            default:
                break;
            }
        }
    }
}
/*********************************************************************************************************
** Function name:       crosswayCheck
** Descriptions:        统计某坐标存在还未走过的支路数
** input parameters:    ucX，需要检测点的横坐标
**                      ucY，需要检测点的纵坐标
** output parameters:   无
** Returned value:      ucCt，未走过的支路数
*********************************************************************************************************/
uint8 crosswayCheck (int8  cX, int8  cY)
{
    uint8 ucCt = 0;
    if ((GucMapBlock[cX][cY] & 0x01) &&                                 /*  绝对方向，迷宫上方有路      */
        (GucMapBlock0[cX][cY + 1]) == 0x00) {                            /*  绝对方向，迷宫上方未走过    */
        ucCt++;                                                         /*  可前进方向数加1             */
    }
    if ((GucMapBlock[cX][cY] & 0x02) &&                                 /*  绝对方向，迷宫右方有路      */
        (GucMapBlock0[cX + 1][cY]) == 0x00) {                            /*  绝对方向，迷宫右方没有走过  */
        ucCt++;                                                         /*  可前进方向数加1             */
    }
    if ((GucMapBlock[cX][cY] & 0x04) &&                                 /*  绝对方向，迷宫下方有路      */
        (GucMapBlock0[cX][cY - 1]) == 0x00) {                            /*  绝对方向，迷宫下方未走过    */
        ucCt++;                                                         /*  可前进方向数加1             */
    }
    if ((GucMapBlock[cX][cY] & 0x08) &&                                 /*  绝对方向，迷宫左方有路      */
        (GucMapBlock0[cX - 1][cY]) == 0x00) {                            /*  绝对方向，迷宫左方未走过    */
        ucCt++;                                                         /*  可前进方向数加1             */
    }
    return ucCt;
}
/*********************************************************************************************************
** Function name:       crosswayChoice
** Descriptions:        选择一条支路作为前进方向
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void crosswayChoice (void)
{
    switch (2) {
        
    case RIGHTMETHOD:
        rightMethod();
        break;
    
    case LEFTMETHOD:
        leftMethod();
        break;
    
    case CENTRALMETHOD:
        centralMethod();
        break;

    case FRONTRIGHTMETHOD:
        frontRightMethod();
        break;

    case FRONTLEFTMETHOD:
        frontLeftMethod();
        break;
       

    default:
        break;
    }
}
/*********************************************************************************************************
** Function name:       __ir_Get
** Descriptions:        读取E2PROM中的红外频率
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void RCC_Init(void)
{    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | 
                           RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | 
                           RCC_APB2Periph_GPIOE |RCC_APB2Periph_AFIO, ENABLE);
}

// mode:JTAG,SWD模式设置；00，全使能，01使能SWD，10全关闭
void JTAG_Set(u8 mode)
{
    u32 temp;
    temp=mode;
    temp<<=25;
    RCC->APB2ENR|=1<<0;
    AFIO->MAPR&=0XF8FFFFFF;
    AFIO->MAPR|=temp;
}

void centralMethodnew(void)
{
    int m = 0,flag = 0;                                                /*  统计可前进的支路数          */
    static int ss=0;
    
    while(1)
    {
        if((((GmcMouse.cX==GucXGoal0)&&(GmcMouse.cY==GucYGoal0))|| 
           ((GmcMouse.cX==GucXGoal1)&&(GmcMouse.cY==GucYGoal1)) ||
           ((GmcMouse.cX==GucXGoal0)&&(GmcMouse.cY==GucYGoal1)) ||
           ((GmcMouse.cX==GucXGoal1)&&(GmcMouse.cY==GucYGoal0))))
        {
             goto End2;
        }
        m = crosswayCheck(GmcMouse.cX,GmcMouse.cY);
        if (m>0)                                                       /*  有可前进方向                */
        {  if(m>1){ ss++; if(ss=0){mouseStop();while(1);}};
            crosswayChoice();                                          /*  用右手法则搜索选择前进方向  */
            mazeSearch();
        }
        else if (m==0)
        {
            flag=1;      
            goto End2;
        }
        else;
      }
End2: if(flag)
          floodMethodnew(GucXGoal0, GucYGoal0);
      else;
}

void floodMethodnew(int8 cXdst, int8 cYdst)
{


    uint8 flag = 0;
    uint8 n = 0;
    uint8 ucStep = 0;
    int8  cDirTemp;
    int8  cX,cY;
    int8  t=0;
    cX = GmcMouse.cX;
    cY = GmcMouse.cY;
    mapStepEdithong(cXdst,cYdst);
    if(((cXdst!=GucXGoal0)&&(cXdst!=GucXGoal1))||
       ((cYdst!=GucYGoal0)&&(cYdst!=GucYGoal1)))             
    {           
        t=1;                                                         
    }
    while ((cX!=cXdst)||(cY!=cYdst)) 
    {
        if((((GmcMouse.cX==GucXGoal0)&&(GmcMouse.cY==GucYGoal0)) || 
            ((GmcMouse.cX==GucXGoal1)&&(GmcMouse.cY==GucYGoal1)) ||
            ((GmcMouse.cX==GucXGoal0)&&(GmcMouse.cY==GucYGoal1)) ||
            ((GmcMouse.cX==GucXGoal1)&&(GmcMouse.cY==GucYGoal0)))
            &&(t==0))
        { 
            cDirTemp=GucMouseDir;
            break;
        }
    
        n=0;
        ucStep = GucMapStep1[cX][cY]; 
        /*
         *  任选一个等高值比当前自身等高值小的方向前进
         */
        if ((GucMapBlock1[cX][cY] & 0x01) &&                             /*  上方有路                    */
            (GucMapStep1[cX][cY + 1] < ucStep)&&                         /*  上方等高值较小              */
            (GucMapBlock0[cX][cY + 1]> 0x00))
        {   
            cDirTemp = UP;
            n++;
            if (cDirTemp == GucMouseDir)                                 /*  优先选择不需要转弯的方向    */
            {                             
                goto T;                                     
            }
        }
        if ((GucMapBlock1[cX][cY] & 0x02) &&                             /*  右方有路                    */
            (GucMapStep1[cX + 1][cY] < ucStep)&&                         /*  右方等高值较小              */
            (GucMapBlock0[cX + 1][cY] >0x00)) 
        {   
            cDirTemp = RIGHT;
            n++;
            if (cDirTemp == GucMouseDir)                                 /*  优先选择不需要转弯的方向    */ 
            {                                         
                goto T; 
            }
        }
        if ((GucMapBlock1[cX][cY] & 0x04) &&                             /*  下方有路                    */
            (GucMapStep1[cX][cY - 1] < ucStep)&&                         /*  下方等高值较小              */
            (GucMapBlock0[cX][cY - 1]>0x00)) 
        {   
            cDirTemp = DOWN;
            n++;
            if (cDirTemp == GucMouseDir)                                 /*  优先选择不需要转弯的方向    */ 
            {                                           
                goto T; 
            }
        }
        if ((GucMapBlock1[cX][cY] & 0x08) &&                             /*  左方有路                    */
            (GucMapStep1[cX - 1][cY] < ucStep)&&                         /*  左方等高值较小              */
            (GucMapBlock0[cX - 1][cY]>0x00)) 
        {
            cDirTemp = LEFT;
            n++;
            if (cDirTemp == GucMouseDir)                                 /*  优先选择不需要转弯的方向    */
            {                              
                goto T;                
            }
        }      
        if(n==0)
        {
            flag=1;
            goto End1;
        }
        cDirTemp = (cDirTemp + 8 - GucMouseDir)%8;                       /*  计算方向偏移量              */
           
        switch (cDirTemp) 
        {

        case 2:
          if(GucMouseTurn){
            if(DenggaoCheck ()== false){
               while(PulseCtrCheck ()== false);
               mouseStop();
               mouseTurnright_Y();
            }
            GucMouseTurn =0;
          }
          else
            mouseTurnright();
           cX = GmcMouse.cX;
           cY = GmcMouse.cY;
             break;
         case 4:
            GucMouseTurn =0;
            mouseTurnback_H();
            cX = GmcMouse.cX;
            cY = GmcMouse.cY;
         
            break;

        case 6:
          if(GucMouseTurn){
            if(DenggaoCheck ()== false){
               while(PulseCtrCheck ()== false);
               mouseStop();
               mouseTurnleft_Y();
            }
            GucMouseTurn =0;
          }
          else
            
            mouseTurnleft();
            cX = GmcMouse.cX;
            cY = GmcMouse.cY;
            break;

        default:
            break;
        }
    
T:   mazeSearch();
     cX = GmcMouse.cX;
     cY = GmcMouse.cY;
      
    }

End1: if(flag)
        centralMethodnew();
      else;
}

/*********************************************************************************************************
** Function name:       goalwall
** Descriptions:       
** input parameters:    无
** output parameters:   无
** Returned value:     
*********************************************************************************************************/
void goalwall(void)
{
 
    if ((GmcMouse.cX==7)&&(GmcMouse.cY==7))                   /* 终点墙壁资料赋值*/
    {                               
        if(GucMouseDir==UP) 
        {       
            GucMapBlock1[7][8]=0x16;
            GucMapBlock1[8][7]=0x19;
            GucMapBlock1[8][8]=0x1c;
            GucMapBlock1[6][7]=(GucMapBlock1[6][7]&0xfd);
            GucMapBlock1[6][8]=(GucMapBlock1[6][8]&0xfd);
            GucMapBlock1[7][9]=(GucMapBlock1[7][9]&0xfb);
            GucMapBlock1[8][9]=(GucMapBlock1[8][9]&0xfb);
            GucMapBlock1[9][8]=(GucMapBlock1[9][8]&0xf7);
            GucMapBlock1[9][7]=(GucMapBlock1[9][7]&0xf7);
            GucMapBlock1[8][6]=(GucMapBlock1[8][6]&0xfe);
            GucMapBlock0[7][8]=0x16;
            GucMapBlock0[8][7]=0x19;
            GucMapBlock0[8][8]=0x1c;
       
        }
        if(GucMouseDir==RIGHT) 
        {
            GucMapBlock1[7][8]=0x16;
            GucMapBlock1[8][7]=0x19;
            GucMapBlock1[8][8]=0x1c;
            GucMapBlock1[6][8]=(GucMapBlock1[6][8]&0xfd);
            GucMapBlock1[7][9]=(GucMapBlock1[7][9]&0xfb);
            GucMapBlock1[8][9]=(GucMapBlock1[8][9]&0xfb);
            GucMapBlock1[9][8]=(GucMapBlock1[9][8]&0xf7);
            GucMapBlock1[9][7]=(GucMapBlock1[9][7]&0xf7);
            GucMapBlock1[8][6]=(GucMapBlock1[8][6]&0xfe);
            GucMapBlock1[7][6]=(GucMapBlock1[7][6]&0xfe);
            GucMapBlock0[7][8]=0x16;
            GucMapBlock0[8][7]=0x19;
            GucMapBlock0[8][8]=0x1c;
        }
    }
    if ((GmcMouse.cX==7)&&(GmcMouse.cY==8)) 
    {
        if(GucMouseDir==DOWN) 
        {
            GucMapBlock1[7][7]=0x13;
            GucMapBlock1[8][7]=0x19;
            GucMapBlock1[8][8]=0x1c;
            GucMapBlock1[6][7]=(GucMapBlock1[6][7]&0xfd);
            GucMapBlock1[6][8]=(GucMapBlock1[6][8]&0xfd);
            GucMapBlock1[8][9]=(GucMapBlock1[8][9]&0xfb);
            GucMapBlock1[9][8]=(GucMapBlock1[9][8]&0xf7);
            GucMapBlock1[9][7]=(GucMapBlock1[9][7]&0xf7);
            GucMapBlock1[8][6]=(GucMapBlock1[8][6]&0xfe);
            GucMapBlock1[7][6]=(GucMapBlock1[7][6]&0xfe);
            GucMapBlock0[7][7]=0x13;
            GucMapBlock0[8][7]=0x19;
            GucMapBlock0[8][8]=0x1c;
        }
        if(GucMouseDir==RIGHT) 
        {
            GucMapBlock1[7][7]=0x13;
            GucMapBlock1[8][7]=0x19;
            GucMapBlock1[8][8]=0x1c;
            GucMapBlock1[6][7]=(GucMapBlock1[6][7]&0xfd);
            GucMapBlock1[7][9]=(GucMapBlock1[7][9]&0xfb);
            GucMapBlock1[8][9]=(GucMapBlock1[8][9]&0xfb);
            GucMapBlock1[9][8]=(GucMapBlock1[9][8]&0xf7);
            GucMapBlock1[9][7]=(GucMapBlock1[9][7]&0xf7);
            GucMapBlock1[8][6]=(GucMapBlock1[8][6]&0xfe);
            GucMapBlock1[7][6]=(GucMapBlock1[7][6]&0xfe);
            GucMapBlock0[7][7]=0x13;
            GucMapBlock0[8][7]=0x19;
            GucMapBlock0[8][8]=0x1c;
        }
    }
    if ((GmcMouse.cX==8)&&(GmcMouse.cY==7)) 
    {
        if(GucMouseDir==UP) 
        {
            GucMapBlock1[7][7]=0x13;
            GucMapBlock1[7][8]=0x16;
            GucMapBlock1[8][8]=0x1c;
            GucMapBlock1[6][7]=(GucMapBlock1[6][7]&0xfd);
            GucMapBlock1[6][8]=(GucMapBlock1[6][8]&0xfd);
            GucMapBlock1[7][9]=(GucMapBlock1[7][9]&0xfb);
            GucMapBlock1[8][9]=(GucMapBlock1[8][9]&0xfb);
            GucMapBlock1[9][8]=(GucMapBlock1[9][8]&0xf7);
            GucMapBlock1[9][7]=(GucMapBlock1[9][7]&0xf7);
            GucMapBlock1[7][6]=(GucMapBlock1[7][6]&0xfe);
            GucMapBlock0[7][7]=0x13;
            GucMapBlock0[7][8]=0x16;
            GucMapBlock0[8][8]=0x1c;
        }
        if(GucMouseDir==LEFT) 
        {
            GucMapBlock1[7][7]=0x13;
            GucMapBlock1[7][8]=0x16;
            GucMapBlock1[8][8]=0x1c;
            GucMapBlock1[6][7]=(GucMapBlock1[6][7]&0xfd);
            GucMapBlock1[6][8]=(GucMapBlock1[6][8]&0xfd);
            GucMapBlock1[7][9]=(GucMapBlock1[7][9]&0xfb);
            GucMapBlock1[8][9]=(GucMapBlock1[8][9]&0xfb);
            GucMapBlock1[9][8]=(GucMapBlock1[9][8]&0xf7);
            GucMapBlock1[8][6]=(GucMapBlock1[8][6]&0xfe);
            GucMapBlock1[7][6]=(GucMapBlock1[7][6]&0xfe);
            GucMapBlock0[7][7]=0x13;
            GucMapBlock0[7][8]=0x16;
            GucMapBlock0[8][8]=0x1c;
        }
    }
    if ((GmcMouse.cX==8)&&(GmcMouse.cY==8)) 
    {
        if(GucMouseDir==DOWN) 
        {
            GucMapBlock1[7][7]=0x13;
            GucMapBlock1[7][8]=0x16;
            GucMapBlock1[8][7]=0x19;
            GucMapBlock1[6][7]=(GucMapBlock1[6][7]&0xfd);
            GucMapBlock1[6][8]=(GucMapBlock1[6][8]&0xfd);
            GucMapBlock1[7][9]=(GucMapBlock1[7][9]&0xfb);
            GucMapBlock1[9][8]=(GucMapBlock1[9][8]&0xf7);
            GucMapBlock1[9][7]=(GucMapBlock1[9][7]&0xf7);
            GucMapBlock1[8][6]=(GucMapBlock1[8][6]&0xfe);
            GucMapBlock1[7][6]=(GucMapBlock1[7][6]&0xfe);
            GucMapBlock0[7][7]=0x13;
            GucMapBlock0[7][8]=0x16;
            GucMapBlock0[8][7]=0x19;
        }
        if(GucMouseDir==LEFT) 
        {
            GucMapBlock1[7][7]=0x13;
            GucMapBlock1[7][8]=0x16;
            GucMapBlock1[8][7]=0x19;
            GucMapBlock1[6][7]=(GucMapBlock1[6][7]&0xfd);
            GucMapBlock1[6][8]=(GucMapBlock1[6][8]&0xfd);
            GucMapBlock1[7][9]=(GucMapBlock1[7][9]&0xfb);
            GucMapBlock1[8][9]=(GucMapBlock1[8][9]&0xfb);
            GucMapBlock1[9][7]=(GucMapBlock1[9][7]&0xf7);
            GucMapBlock1[8][6]=(GucMapBlock1[8][6]&0xfe);
            GucMapBlock1[7][6]=(GucMapBlock1[7][6]&0xfe);
            GucMapBlock0[7][7]=0x13;
            GucMapBlock0[7][8]=0x16;
            GucMapBlock0[8][7]=0x19;
        }
    }
}

void floodwall(void)
{     uint8 n          = 0;                                                                                            
      uint8 ucTemp     = 0;
        for(n=0;n<MAZETYPE;n++) {                                           /*墙壁初值无法写入，所以在此从新赋值*/
          for(ucTemp=0;ucTemp<MAZETYPE;ucTemp++) {                                         /* 洪水算法所用*/
          GucMapBlock1[n][ucTemp] =0x1f; }}
           n=0;    ucTemp     = 0;
       for(n=0;n<MAZETYPE;n++) {
         GucMapBlock1[0][n] =0x17;          
         GucMapBlock1[MAZETYPE-1][n] =0x1d; }
            n=0;
       for(n=0;n<MAZETYPE;n++) 
       {   GucMapBlock1[n][0] =0x1b;          
         GucMapBlock1[n][MAZETYPE-1] =0x1e; }
               n=0;
       GucMapBlock1[MAZETYPE-1][MAZETYPE-1] =0x1c;
        GucMapBlock1[0][MAZETYPE-1] =0x16;
         GucMapBlock1[0][0] =0x11;
         GucMapBlock1[MAZETYPE-1][0] =0x19;
    
             for(n=0;n<MAZETYPE;n++)                                            /*墙壁初值无法写入，所以在此从新赋值*/
       { for(ucTemp=0;ucTemp<MAZETYPE;ucTemp++)
     {  GucMapBlock[n][ucTemp] =0x10; }}
           n=0;    ucTemp     = 0;
}

void StartSave()
{
    Test_Write(START_SAVE_ADDRESS,GucXStart);
}

void wallsave(void)
{
   uint8 x = 0;                                                                                            
   uint8 y = 0;
   uint16 GucMapBlock4[16][16]={0};
   for(x=0;x<MAZETYPE;x++) {                                           
      for(y=0;y<MAZETYPE;y++) {                                         
          GucMapBlock4[x][y] =GucMapBlock[x][y]; 
      }
   } 
  STMFLASH_Write(MAPE_SAVE_ADDRESS,(u16*)GucMapBlock4,256);

}


void wallget(void)
{
   uint8 x = 0;                                                                                            
   uint8 y = 0;
   uint16 GucMapBlock4[16][16]={0};
   STMFLASH_Read(MAPE_SAVE_ADDRESS,(u16*)GucMapBlock4,256);
   delay(10000);
   for(x=0;x<MAZETYPE;x++) {                                           
      for(y=0;y<MAZETYPE;y++) {                                         
          GucMapBlock[x][y] =(uint8)GucMapBlock4[x][y]; 
      }
   }   
}

void StartGet(void)
{
  uint16 start = 0;
 
 // read_fm24LC16(&start,0x80,0xa1,1);
  STMFLASH_Read(START_SAVE_ADDRESS,(u16*)&start,1);
  if(start == 15)
  {
    GucXStart   = MAZETYPE - 1;                             /*  修改电脑鼠起点的横坐标      */
    GmcMouse.cX = MAZETYPE - 1; 
  }
}


void __ir_Get(void)
{    
      uint16    DIS1[10] = {0};
      STMFLASH_Read(IR_SAVE_ADDRESS,(u16 *)DIS1,10);
      delay(1000000);
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
void GPIO_Config1()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3 | RCC_APB2Periph_GPIOB |RCC_APB2Periph_AFIO, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	         
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
    GPIO_Init(GPIOB, &GPIO_InitStructure);		   

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;	        
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}	




void USART1_Config1(void)
{
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure); 
    USART_Cmd(USART3, ENABLE);
}

void NVIC_Config1(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 		   
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;  	           
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; 	    
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; 		    
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);		
    USART_Cmd(USART3, ENABLE);
}

void uart_sendchar(unsigned char ch)
{
  while(USART_GetFlagStatus(USART3,USART_FLAG_TXE) == RESET);
  USART_SendData(USART3,ch);
}


int fputc(int ch,FILE *f)
{
  USART_SendData(USART3,(unsigned char)ch);
  while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
  return (ch);
}


void USART3_IRQHandler(void)
{
    int RX_status;
    RX_status = USART_GetFlagStatus(USART3, USART_FLAG_RXNE);
    if(RX_status == SET) 
    {
        //USART_SendData(USART3 , USART_ReceiveData(USART3));
        Rece=USART_ReceiveData(USART3);
        while(USART_GetFlagStatus(USART3, USART_FLAG_TC)==RESET);
    }
}
void SendBlook()
{
    uint8 String[]="0123456789abcdef";
    //uint8 shi,ge;
    SendStr("GucMapBlock[");
    uart_sendchar(String[GmcMouse.cY]);
    SendStr("]= ");
    uart_sendchar(String[(GucMapBlock[GmcMouse.cX][GmcMouse.cY] & 0X08) >> 3]);
    uart_sendchar(String[(GucMapBlock[GmcMouse.cX][GmcMouse.cY] & 0X04) >> 2]);
    uart_sendchar(String[(GucMapBlock[GmcMouse.cX][GmcMouse.cY] & 0X02) >> 1]);
    uart_sendchar(String[GucMapBlock[GmcMouse.cX][GmcMouse.cY] & 0X01]);
    SendStr("         ");
}
void Sendmap()
{
    uint8 String[]="0123456789abcdef";
  /*  if(map == 0)
       uart_sendchar(String[15]);
    else*/
    uart_sendchar(String[GmcMouse.cX]);
    SendStr("  ");
    uart_sendchar(String[GmcMouse.cY]);
    SendStr("  ");
    uart_sendchar(String[GucMapBlock[GmcMouse.cX][GmcMouse.cY]]);
    SendStr("\r\n");
}
void SendStr(uint8 *s)
{
  while(*s){    
  // USART_SendData(USART3, *s++);
    uart_sendchar(*s++);
  }
}
void objectGoTo_liang(int8 cXdst, int8 cYdst)
{
    uint8 ucStep = 1;
    int8  cNBlock = 0, cDirTemp;
    int8  cX,cY;
    GucCrossroad = 0;
    cX = GmcMouse.cX;
    cY = GmcMouse.cY;
    mapStepEdit(cXdst, cYdst);                                           /*  制作等高图                  */
    
    // 根据等高值向目标点运动，直到达到目的地
    while ((cX != cXdst) || (cY != cYdst)) 
    {
        ucStep = GucMapStep[cX][cY];
        // 任选一个等高值比当前自身等高值小的方向前进
        if ((GucMapBlock[cX][cY] & 0x01) &&                             /*  上方有路                    */
            (GucMapStep[cX][cY + 1] < ucStep))                          /*  上方等高值较小              */
        {                        
            cDirTemp = UP;                                              /*  记录方向                    */
            if (cDirTemp == GucMouseDir)                                /*  优先选择不需要转弯的方向    */
            {                              
                cNBlock++;                                              /*  前进一个方格                */
                cY++;
                if((GucMapBlock[cX][cY] & 0x0f) == 0x0f)
                    GucCrossroad++;
                continue;                                               /*  跳过本次循环                */
            }
        }
        if ((GucMapBlock[cX][cY] & 0x02) &&                             /*  右方有路                    */
            (GucMapStep[cX + 1][cY] < ucStep))                          /*  右方等高值较小              */
        {                        
            cDirTemp = RIGHT;                                           /*  记录方向                    */
            if (cDirTemp == GucMouseDir)                                /*  优先选择不需要转弯的方向    */
            {                             
                cNBlock++;                                              /*  前进一个方格                */
                cX++;
                if((GucMapBlock[cX][cY] & 0x0f) == 0x0f)
                    GucCrossroad++;
                continue;                                               /*  跳过本次循环                */
            }
        }
        if ((GucMapBlock[cX][cY] & 0x04) &&                             /*  下方有路                    */
            (GucMapStep[cX][cY - 1] < ucStep))                          /*  下方等高值较小              */
        {                        
            cDirTemp = DOWN;                                            /*  记录方向                    */
            if (cDirTemp == GucMouseDir)                                /*  优先选择不需要转弯的方向    */
            {                              
                cNBlock++;                                              /*  前进一个方格                */
                cY--;
                if((GucMapBlock[cX][cY] & 0x0f) == 0x0f)
                    GucCrossroad++;
                continue;                                               /*  跳过本次循环                */
            }
        }
        if ((GucMapBlock[cX][cY] & 0x08) &&                             /*  左方有路                    */
            (GucMapStep[cX - 1][cY] < ucStep))                          /*  左方等高值较小              */
        {                        
            cDirTemp = LEFT;                                            /*  记录方向                    */
            if (cDirTemp == GucMouseDir)                                /*  优先选择不需要转弯的方向    */
            {                              
                cNBlock++;                                              /*  前进一个方格                */
                cX--;
                if((GucMapBlock[cX][cY] & 0x0f) == 0x0f)
                    GucCrossroad++;
                continue;                                               /*  跳过本次循环                */
            }
        }
        cDirTemp = (cDirTemp + 8 - GucMouseDir)%8;                      /*  计算方向偏移量              */
        GucDirTemp = cDirTemp;
        if (cNBlock) 
        {
            if((GucCrossroad <= 1)&&(cNBlock >=1))
                mouseGoahead(cNBlock);                                      /*  前进cNBlock步               */
            else
            {
                mouseGoahead(cNBlock);
                GucCrossroad = 0;
            }                 
        }        
        cNBlock = 0;  
        /*  任务清零                    */
        
        //  控制电脑鼠转弯         
       switch (cDirTemp) 
       {
           case 1:    
               mouseTurnright();
               //while(1);
               break;

           case 2:
               mouseTurnback();
               break;

           case 3:
               mouseTurnleft();
               //while(1);
               break;

           default:
               break;
        }
        GmcMouse.cX=cX;
        GmcMouse.cY=cY;
    }
    //  判断任务是否完成，否则继续前进
    
    if (cNBlock) 
    {
        if((GucCrossroad <= 1) && (cNBlock >= 1))
            mouseGoahead(cNBlock);                                     /*  前进cNBlock步               */
        else
        {
            mouseGoahead(cNBlock);
            GucCrossroad = 0;
        }
        GmcMouse.cX=cX;
        GmcMouse.cY=cY;
    }
}


uint8 zijiaozheng0=0;
  uint8 usepiancha =0;
main (void)       
{
    uint8 n          = 0;                                               /*  GmcCrossway[]下标           */
    uint8 ucRoadStat = 0;                                               /*  统计某一坐标可前进的支路数  */
    uint8 ucTemp     = 0;                                               /*  用于START状态中坐标转换 */
    uint8 start=0;
    uint8 start_maxspeed=0;
    uint8 start_led=0;
    SystemInit();
    RCC_Init();
    JTAG_Set(1);
    MouseInit();   //电脑鼠初始化
    PIDInit(); 
    ZLG7289Init();
    delay(100000);
    //__ir_Get();
    delay(100000);
    GPIO_Config1();
    USART1_Config1();
    NVIC_Config1();
    floodwall();
    GPIO_SetBits(GPIOB,GPIO_Pin_12);

    while (1) {
        switch (GucMouseTask) {                                         /*  状态机处理                  */     
            case WAIT:
               sensorDebug();  //检查红外
               delay(10000);   
            
             if (startCheck() == true) 
               {       
                  start++;
                }
               if((start==1)&&GucGoHead)//&&GucGoHead
               {
                  start=0;
                  zijiaozheng0=1;
                  GPIO_ResetBits(GPIOB,GPIO_Pin_12);
                  delay(10000000); 
                  GucMouseTask = START;
                  delay(1000000);
                  GPIO_SetBits(GPIOB,GPIO_Pin_12);
                  zijiaozheng0=1;
               }
             
             if(start==8)//&&GucGoHead
               {
                  start=0;
                  zijiaozheng0=1;
                  GPIO_ResetBits(GPIOB,GPIO_Pin_12);
                  delay(10000000); 
                  TIM4->CCR3 = 600;
                  delay(1000000);
                  GPIO_SetBits(GPIOB,GPIO_Pin_12);
                  zijiaozheng0=1;
               }
             
               if((start<5)&&(start>=3)&&GucGoHead)
               {
                 GPIO_ResetBits(GPIOB,GPIO_Pin_12);
                 zijiaozheng0=1;
                 start=0;
                 wallget();
                 delay(10000000);
                 StartGet();
                 delay(10000000);
                 delay(1000000);
                 delay(1000000);
                 delay(1000000);
                 delay(1000000);
                 GucMouseTask = SPURTL; 
                 delay(1000000);
               }           
                 break;
        case START:                                                     /*  判断电脑鼠起点的横坐标      */
                
                delay(10000000);
                GPIO_ResetBits(GPIOB,GPIO_Pin_12);
                mazeSearch();                                  /*  向前搜索                    */
                if (GucMapBlock[GmcMouse.cX][GmcMouse.cY] & 0x08) 
                {          
                    if (MAZETYPE == 16)                           
                    {                                    
                        GucXGoal0 = 8;
                        GucXGoal1 = 7;
                    }
                    GucXStart   = MAZETYPE - 1;                       
                    GmcMouse.cX = MAZETYPE - 1;                      
                    ucTemp = GmcMouse.cY;
                    do {
                           GucMapBlock[MAZETYPE - 1][ucTemp] = GucMapBlock[0][ucTemp];
                           GucMapBlock0[MAZETYPE - 1][ucTemp]= GucMapBlock0[0][ucTemp];
                           GucMapBlock0[0][ucTemp] = 0;
                           GucMapBlock1[MAZETYPE - 1][ucTemp]= GucMapBlock1[0][ucTemp];    
                           if(ucTemp > 0)
                           {
                               GucMapBlock1[MAZETYPE - 2][ucTemp-1] = 0x1d;
                           }
                           GucMapBlock1[0][ucTemp+1] =0x17;
                           GucMapBlock1[1][ucTemp] =0x1f;
                           GucMapBlock[0][ucTemp] =0x10;
                           GucMapBlock[1][ucTemp] =0x10;                                                
                         
                       }while (ucTemp--);
                    GucMapBlock1[0][0] =0x13;
                    GucMapBlock1[1][0] =0x1b;
                    GucMapBlock1[MAZETYPE - 2][0] =0x19;
                    // 在OFFSHOOT[0]中保存起点坐标
                    GmcCrossway[n].cX = MAZETYPE - 1;
                    GmcCrossway[n].cY = 0;
                    n++;
                    GucMouseTask = MAZESEARCH;                              
                }
                if (GucMapBlock[GmcMouse.cX][GmcMouse.cY] & 0x02)          
                {        
                    // 在OFFSHOOT[0]中保存起点坐标
                    GmcCrossway[n].cX = 0;
                    GmcCrossway[n].cY = 0;
                    n++;
                    GucMouseTask = MAZESEARCH;                              
                }
            break;       
        case MAZESEARCH: 
              
              centralMethodnew();
              goalwall();
              StartSave();
              wallsave();
              TIM4->CCR3 = 600;
              mouseTurnback();
              objectGoTo1(GucXStart,GucYStart);
              mouseTurnbackqi(); 
              delay(10000000);
              GucMouseTask = SPURTL;
            break;  
            
           case  SPURTL:    //一冲（普通冲刺）
             StartGet();
             wallget();
             goalwall();
             mouseSpurt_CC();
             mouseTurnback();
             objectGoTo1(GucXStart,GucYStart);
             onestep3();
             mouseTurnbackqidian();
             
             GucMouseTask = WAIT;
             delay(10000000);
      
        
        default:
            break;
        }
    }
}






     
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
