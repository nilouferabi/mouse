/****************************************Copyright (c)****************************************************
**                               Guangzhou ZHIYUAN electronics Co.,LTD.
**                                     
**                                 http://www.embedtools.com
**
**--------------File Info---------------------------------------------------------------------------------
** File Name:           main.c
** Description:         电脑鼠主状态机：WAIT/START/MAZESEARCH/SPURTL
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
#include "maze.h"
#include "maze_search.h"
#include "stm32f10x.h"
#include "display.h"
#include "motion_control.h"
#include "path_8dir.h"
#include "fan.h"
#include "status_led.h"
#include "board_init.h"
#include "maze_storage.h"
#include "serial_debug.h"
#include "delay.h"
/*********************************************************************************************************
  全局变量定义
*********************************************************************************************************/

static uint8 GucMouseTask = WAIT;                                 /*  状态机，初始状态为等待      */

int main(void)
{
    uint8 ucTemp = 0;                                           /*  START 状态中坐标转换         */
    uint8 start  = 0;                                           /*  START 按键次数               */
#if TUNE_8DIR_ENABLE
    uint8 nav    = NAV_NO_PATH;                                 /*  8 向冲刺结果                 */
#endif

    SystemInit();
    Clock_Init();
    JTAG_Config(1);
    MouseInit();                                                /*  电脑鼠初始化                 */
    PIDInit();
    ZLG7289Init();
    Fan_Off();
    Gyro_Calibrate();                                           /* 上电自动陀螺零位校准          */
    delay(TUNE_DELAY_BOOT);
    //Storage_LoadIrThresholds();                                               /*  需要时打开：从 Flash 载入红外阈值 */
    delay(TUNE_DELAY_BOOT);
    Serial_Init();
    floodwall();
    Fan_Off();
    Led_SetMode(LED_MODE_OFF);

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
                Led_SetMode(LED_MODE_SEARCH);
                delay(TUNE_DELAY_SETTLE);
                GucMouseTask = START;
                delay(TUNE_DELAY_POST_ACTION);
            }
            if ((start >= TUNE_START_PRESS_SPURT_MIN) &&
                (start < TUNE_START_PRESS_SPURT_MAX) && GucGoHead) {
                Fan_Off();
                Led_SetMode(LED_MODE_OFF);
                start = 0;
                Storage_LoadMap();
                delay(TUNE_DELAY_SETTLE);
                Storage_LoadStart();
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
            Led_SetMode(LED_MODE_SEARCH);
            delay(TUNE_DELAY_SETTLE);
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
            Led_SetMode(LED_MODE_SEARCH);
            centralMethodnew();
            goalwall();
            Storage_SaveStart();
            Storage_SaveMap();
            mouseTurnback();
            objectGoTo1(GucXStart, GucYStart);
            mouseTurnbackqi();
            delay(TUNE_DELAY_SETTLE);
            GucMouseTask = SPURTL;
            break;

        case SPURTL:                                            /*  一冲（普通冲刺 / 8 向冲刺）  */
            Led_SetMode(LED_MODE_SPRINT);
            Fan_On();
            Storage_LoadStart();
            Storage_LoadMap();
            goalwall();

#if TUNE_8DIR_ENABLE
            nav = mouseSpurt_8_to_goal(GucXGoal0, GucXGoal1, GucYGoal0, GucYGoal1);
            if (nav == NAV_NO_PATH) {
                mouseSpurt_CC();                                /*  地图没有斜向通路，回退 4 向  */
            } else if (nav != NAV_OK) {
                Fan_Off();
                Led_SetMode(LED_MODE_OFF);
                mouseStop();                                    /*  运动超时/中止，停车待查       */
                GucMouseTask = WAIT;
                delay(TUNE_DELAY_SETTLE);
                break;
            }
#else
            mouseSpurt_CC();
#endif

            mouseTurnback();
            objectGoTo1(GucXStart, GucYStart);
            onestep3();
            mouseTurnbackqidian();
            Fan_Off();
            Led_SetMode(LED_MODE_OFF);
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
