/****************************************Copyright (c)****************************************************
**                               Guangzhou ZHIYUAN electronics Co.,LTD.
**                                     
**                                 http://www.embedtools.com
**
**--------------File Info---------------------------------------------------------------------------------
** File Name:           motion_control.h
** Description:         运动控制层接口：状态枚举、PID/电机结构体、对外函数声明
**
*********************************************************************************************************/

#ifndef __MOTION_CONTROL_H
#define __MOTION_CONTROL_H


/*********************************************************************************************************
  包含头文件
*********************************************************************************************************/
#include "stm32f10x.h"
#include "display.h"
#include "types.h"
#include "micromouse.h"
#include "mouse_config.h"
#include "motion_8dir.h"

/*********************************************************************************************************
  引脚定义（仅保留实际使用项；其余硬件映射见 board_init.c）
*********************************************************************************************************/
#define __START               GPIO_Pin_10                                  /* START 键：PC10             */

/*********************************************************************************************************
  传感器槽位（__GucDistance[] 的下标）
*********************************************************************************************************/
#define __LEFT              0                                           /*  左方传感器                  */
#define __FRONTL            1                                           /*  左前方传感器                */
#define __FRONT             2                                           /*  前方传感器（当前硬件未单独使用）*/
#define __FRONTR            3                                           /*  右前方传感器                */
#define __RIGHT             4                                           /*  右方传感器                  */

/* ucIRCheck[] 下标：与物理四路传感器一一对应 */
#define IR_CH_FL             0
#define IR_CH_FR             1
#define IR_CH_L              2
#define IR_CH_R              3


/*********************************************************************************************************
  电脑鼠状态
*********************************************************************************************************/
#define __STOP              0                                           /*  电脑鼠停止                  */
#define __GOAHEAD           1                                           /*  电脑鼠前进                  */
#define __GOAHEAD_45        2                                           /*  斜向直行（8 向）            */
#define __TURNLEFT          3                                           /*  电脑鼠向左转                */
#define __TURNRIGHT         4                                           /*  电脑鼠向右转                */
#define __TURNBACK          5                                           /*  电脑鼠向后转                */
#define __GOBACK            9
/*********************************************************************************************************
  电机加/减速标志
*********************************************************************************************************/
#define __SPEEDUP         0                                           /*  电机加速                    */
#define __SPEEDDOWN       1                                           /*  电机减速                */

/*********************************************************************************************************
  电机状态
*********************************************************************************************************/
#define __MOTORSTOP         0                                           /*  电机停止                    */
#define __MOTORRUN          2                                           /*  电机运行                    */


/*********************************************************************************************************
  电机运行方向
*********************************************************************************************************/
#define __MOTORGOAHEAD      0                                           /*  电机前进                    */
#define __MOTORGOBACK       1                                           /*  电机后退                    */
#define __MOTORGOSTOP       2                                           /*  电机反向制动                */

/*********************************************************************************************************
  结构体定义
  注意: 这些变量会被 1 ms / 5 kHz 中断改写，必须 volatile，否则 -O2 下忙等会被优化成死循环。
*********************************************************************************************************/
struct __motor {
    volatile int8    cState;                    /*  电机运行状态      */
    volatile int8    cDir;                      /*  电机运行方向      */
    volatile int8    cRealDir;                  /*  电机实际方向      */
    volatile uint32  uiPulse;                   /*  电机需要运行的脉冲    */
    volatile uint32  uiPulseCtr;                /*  电机已运行的脉冲       */
    volatile int16   sSpeed;                    /*  当前 PWM 占空比   */
};
typedef struct __motor __MOTOR;

struct __pid {
    volatile int16  usFeedBack;     /*  速度反馈值              */
    volatile uint16 usEncoder_new;  /*  编码器本次读数          */
    volatile uint16 usEncoder_last; /*  编码器上次读数          */

    volatile float sRef;            /*  设定值                  */
    volatile float sFeedBack;       /*  反馈值                  */
    volatile float sPreError;       /*  前一次误差              */
    volatile float sPreDerror;      /*  前一次误差之差          */

    fp32 fKp;                       /*  比例增益                */
    fp32 fKi;                       /*  积分增益                */
    fp32 fKd;                       /*  微分增益                */

    volatile int16 iPreU;           /*  电机控制输出值          */
};
typedef struct __pid __PID;


/*********************************************************************************************************
  对外函数声明
*********************************************************************************************************/
void  PIDInit(void);
void  __SPIDContr(void);
void  __WPIDContr(void);
void  __PIDContr(void);
void  __Encoder(void);
void  GyroscopeFilter(u32 NewDate, u32 *SourceDate, volatile u32 *Average);
void  voltageDetect(void);
void  voltageDetect1(void);

void  sensorDebug(void);
uint8 startCheck(void);
void  mazeSearch(void);
void  mouseStop(void);
void  mouseGoahead_L(int8 cNBlock);
void  mouseTurnleft(void);
void  mouseTurnright(void);
void  mouseTurnleft_C(void);
void  mouseTurnright_C(void);
void  mouseTurnleft_Y(void);
void  mouseTurnright_Y(void);
void  mouseTurnback(void);
void  mouseTurnbackqi(void);
void  mouseTurnbackqidian(void);
void  mouseTurnback_H(void);
void  onestep3(void);

uint8 DenggaoCheck(void);
uint8 PulseCtrCheck(void);

void  MouseInit(void);

extern __PID __GmSPID;
extern volatile uint8 zijiaozheng0;
extern volatile uint32 GucSysTickMs;
uint8 Gyro_Calibrate(void);

#endif
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
