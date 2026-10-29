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
#include "stmflash.h"
#include "stdio.h"
/*********************************************************************************************************
  全局变量定义
*********************************************************************************************************/

uint8    GucXStart                           = 0;                /*  起点横坐标                  */
uint8    GucYStart                           = 0;                /*  起点纵坐标                  */
static uint8    GucXGoal0                    = XDST0;            /*  终点X坐标，有两个值         */
static uint8    GucXGoal1                    = XDST1;
static uint8    GucYGoal0                    = YDST0;            /*  终点Y坐标，有两个值         */
static uint8    GucYGoal1                    = YDST1;
static uint8    GucMouseTask                 = WAIT;             /*  状态机，初始状态为等待      */

static uint8    GucMapStep[MAZETYPE][MAZETYPE]       = {{0xff}};   /*  保存各坐标的等高值          */
static uint8    GucMapStep1[MAZETYPE][MAZETYPE]      = {{0xff}};
static MAZECOOR GmcStack[MAZETYPE * MAZETYPE]        = {0};      /*  在mapStepEdit()中作堆栈使用 */
static MAZECOOR GmcStack1[MAZETYPE * MAZETYPE]       = {0};

uint8 Rece = 0;                                                  /*  USART3 收到的字节           */
static uint8 GucMouseTurn = 0;                                   /*  等高图后是否需要原地转向    */
/*********************************************************************************************************
** Function name:       Delay
** Descriptions:        延时函数
** input parameters:    uiD :延时参数，值越大，延时越久
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void delay(uint32 uiD)
{
    volatile uint32 i;
    for (i = uiD; i; i--);
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

void objectGoTo1(int8  cXdst, int8  cYdst)
{
    uint8 ucStep = 1;
    int8  cNBlock = 0, cDirTemp = 0;
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
    
    objectGoTo1(cXdst,cYdst);/*  运行到指定目标点            */
    
}




uint8 xiexianyige=0;
int action=0;
int8 lingshix,lingshiy=0;
uint8 lingshixyflag=1;

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
    switch (SEARCHMETHOD) {
        
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
    int m = 0, flag = 0;                                            /*  统计可前进的支路数          */
    
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
        {
            crosswayChoice();                                          /*  用右手法则搜索选择前进方向  */
            mazeSearch();
        }
        else if (m==0)
        {
            flag=1;      
            goto End2;
        }
      }
End2: if(flag)
          floodMethodnew(GucXGoal0, GucYGoal0);
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
          {
            mouseTurnright();
          }
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
          {
            mouseTurnleft();
          }
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
  STMFLASH_Write(MAPE_SAVE_ADDRESS,(u16*)GucMapBlock4,TUNE_FLASH_MAP_WORDS);

}


void wallget(void)
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

void StartGet(void)
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


void __ir_Get(void)
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
void USART3_GPIO_Config(void)
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




void USART3_Config(void)
{
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = TUNE_UART_BAUD;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure); 
    USART_Cmd(USART3, ENABLE);
}

void USART3_NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 		   
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;  	           
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TUNE_UART_IRQ_PREEMPT; 	    
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = TUNE_UART_IRQ_SUB; 		    
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
void SendStr(const char *s)
{
  while(*s){    
  // USART_SendData(USART3, *s++);
    uart_sendchar(*s++);
  }
}
uint8 zijiaozheng0 = 0;                                         /*  陀螺零漂校准标志（SysTick 使用）*/

int main(void)
{
    uint8 ucTemp = 0;                                           /*  START 状态中坐标转换         */
    uint8 start  = 0;                                           /*  START 按键次数               */

    SystemInit();
    RCC_Init();
    JTAG_Set(1);
    MouseInit();                                                /*  电脑鼠初始化                 */
    PIDInit();
    ZLG7289Init();
    delay(TUNE_DELAY_BOOT);
    //__ir_Get();                                               /*  需要时打开：从 Flash 载入红外阈值 */
    delay(TUNE_DELAY_BOOT);
    USART3_GPIO_Config();
    USART3_Config();
    USART3_NVIC_Config();
    floodwall();
    GPIO_SetBits(GPIOB, GPIO_Pin_12);

    while (1) {
        switch (GucMouseTask) {                                 /*  状态机处理                   */
        case WAIT:
            sensorDebug();                                      /*  检查红外                     */
            delay(TUNE_DELAY_SENSOR_REFRESH);

            if (startCheck() == true) {
                start++;
            }
            if ((start == TUNE_START_PRESS_SEARCH) && GucGoHead) {
                start = 0;
                zijiaozheng0 = 1;
                GPIO_ResetBits(GPIOB, GPIO_Pin_12);
                delay(TUNE_DELAY_SETTLE);
                GucMouseTask = START;
                delay(TUNE_DELAY_POST_ACTION);
                GPIO_SetBits(GPIOB, GPIO_Pin_12);
                zijiaozheng0 = 1;
            }
            if (start == TUNE_START_PRESS_CALIB) {
                start = 0;
                zijiaozheng0 = 1;
                GPIO_ResetBits(GPIOB, GPIO_Pin_12);
                delay(TUNE_DELAY_SETTLE);
                TIM4->CCR3 = TUNE_FAN_DUTY;
                delay(TUNE_DELAY_POST_ACTION);
                GPIO_SetBits(GPIOB, GPIO_Pin_12);
                zijiaozheng0 = 1;
            }
            if ((start >= TUNE_START_PRESS_SPURT_MIN) &&
                (start < TUNE_START_PRESS_SPURT_MAX) && GucGoHead) {
                GPIO_ResetBits(GPIOB, GPIO_Pin_12);
                zijiaozheng0 = 1;
                start = 0;
                wallget();
                delay(TUNE_DELAY_SETTLE);
                StartGet();
                delay(TUNE_DELAY_SETTLE);
                delay(TUNE_DELAY_POST_ACTION);
                delay(TUNE_DELAY_POST_ACTION);
                delay(TUNE_DELAY_POST_ACTION);
                delay(TUNE_DELAY_POST_ACTION);
                GucMouseTask = SPURTL;
                delay(TUNE_DELAY_POST_ACTION);
            }
            break;

        case START:                                             /*  判断电脑鼠起点的横坐标       */
            delay(TUNE_DELAY_SETTLE);
            GPIO_ResetBits(GPIOB, GPIO_Pin_12);
            mazeSearch();                                       /*  向前搜索                     */
            if (GucMapBlock[GmcMouse.cX][GmcMouse.cY] & 0x08) {
                if (MAZETYPE == 16) {
                    GucXGoal0 = TUNE_GOAL_X0;
                    GucXGoal1 = TUNE_GOAL_X1;
                }
                GucXStart   = MAZETYPE - 1;
                GmcMouse.cX = MAZETYPE - 1;
                ucTemp = GmcMouse.cY;
                do {
                    GucMapBlock[MAZETYPE - 1][ucTemp]  = GucMapBlock[0][ucTemp];
                    GucMapBlock0[MAZETYPE - 1][ucTemp] = GucMapBlock0[0][ucTemp];
                    GucMapBlock0[0][ucTemp] = 0;
                    GucMapBlock1[MAZETYPE - 1][ucTemp] = GucMapBlock1[0][ucTemp];
                    if (ucTemp > 0) {
                        GucMapBlock1[MAZETYPE - 2][ucTemp - 1] = 0x1d;
                    }
                    GucMapBlock1[0][ucTemp + 1] = 0x17;
                    GucMapBlock1[1][ucTemp]     = 0x1f;
                    GucMapBlock[0][ucTemp]      = 0x10;
                    GucMapBlock[1][ucTemp]      = 0x10;
                } while (ucTemp--);
                GucMapBlock1[0][0]             = 0x13;
                GucMapBlock1[1][0]             = 0x1b;
                GucMapBlock1[MAZETYPE - 2][0]  = 0x19;
                GucMouseTask = MAZESEARCH;
            }
            if (GucMapBlock[GmcMouse.cX][GmcMouse.cY] & 0x02) {
                GucMouseTask = MAZESEARCH;
            }
            break;

        case MAZESEARCH:
            centralMethodnew();
            goalwall();
            StartSave();
            wallsave();
            TIM4->CCR3 = TUNE_FAN_DUTY;
            mouseTurnback();
            objectGoTo1(GucXStart, GucYStart);
            mouseTurnbackqi();
            delay(TUNE_DELAY_SETTLE);
            GucMouseTask = SPURTL;
            break;

        case SPURTL:                                            /*  一冲（普通冲刺）             */
            StartGet();
            wallget();
            goalwall();
            mouseSpurt_CC();
            mouseTurnback();
            objectGoTo1(GucXStart, GucYStart);
            onestep3();
            mouseTurnbackqidian();
            GucMouseTask = WAIT;
            delay(TUNE_DELAY_SETTLE);
            break;

        default:
            break;
        }
    }
}

     
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
