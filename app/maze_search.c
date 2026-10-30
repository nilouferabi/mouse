/****************************************Copyright (c)****************************************************
** File Name:           maze_search.c
** Description:         迷宫/寻路算法：等高图、洪水填充、岔路决策、目标选择
**
**   本文件只放“怎么算路”，不放主状态机、Flash 存取和串口。
*********************************************************************************************************/

#include "maze_search.h"
#include "maze.h"
#include "motion_control.h"

/* ---- 起点/终点（状态机和算法共享） ---- */
uint8 GucXStart = 0;
uint8 GucYStart = 0;
uint8 GucXGoal0 = XDST0;
uint8 GucXGoal1 = XDST1;
uint8 GucYGoal0 = YDST0;
uint8 GucYGoal1 = YDST1;

/* ---- 算法内部状态 ---- */
uint8    GucMapStep[MAZETYPE][MAZETYPE]  = {{0xff}};
static uint8    GucMapStep1[MAZETYPE][MAZETYPE] = {{0xff}};
static MAZECOOR GmcStack[MAZETYPE * MAZETYPE]   = {0};
static MAZECOOR GmcStack1[MAZETYPE * MAZETYPE]  = {0};
static uint8    GucMouseTurn = 0;

/* 前向声明：centralMethodnew 在 floodMethodnew 之前调用 */
static void floodMethodnew(int8 cXdst, int8 cYdst);

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



static void mapStepEdithong (int8  cX, int8  cY)
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



/*********************************************************************************************************
** Function name:       mazeBlockDataGet
** Descriptions:        根据电脑鼠的相对方向，取出该方向上迷宫格的墙壁资料
** input parameters:    ucDir: 电脑鼠的相对方向
** output parameters:   无
** Returned value:      GucMapBlock[cX][cY] : 墙壁资料
*********************************************************************************************************/
static uint8 mazeBlockDataGet (uint8  ucDirTemp)
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
static void rightMethod (void)
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
static void leftMethod (void)
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
static void frontRightMethod (void)
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
static void frontLeftMethod (void)
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
static void centralMethod (void)
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
static void crosswayChoice (void)
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



static void floodMethodnew(int8 cXdst, int8 cYdst)
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



/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
