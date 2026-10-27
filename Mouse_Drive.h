/****************************************Copyright (c)****************************************************
**                               Guangzhou ZHIYUAN electronics Co.,LTD.
**                                     
**                                 http://www.embedtools.com
**
**--------------File Info---------------------------------------------------------------------------------
** File Name:           Mouse.Drive.h
** Last modified Date:  
** Last Version: 
** Description:         底层驱动程序头文件
** 
**--------------------------------------------------------------------------------------------------------
** Created By: 
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


#ifndef __Mouse_Drive_h
#define __Mouse_Drive_h


/*********************************************************************************************************
  包含头文件
*********************************************************************************************************/
#include "stm32f10x.h"
#include "Zlg7289.h"
#include "Type.h"
#include "Micromouse.h"
#include "Mouse_Config.h"
#include "stmflash.h"
#include "Mouse_Srv.h"
/*********************************************************************************************************
  PA端口定义
*********************************************************************************************************/                                
#define __D2_1                GPIO_Pin_0                                   /*   电机 M2 PWMH      */
#define __D2_2                GPIO_Pin_1                                   /*   电机 M2 PWML      */
#define __D1_1                GPIO_Pin_2                                   /*   电机 M1 PWMH      */
#define __D1_2                GPIO_Pin_3                                   /*   电机 M1 PWML      */
#define __KEY                 GPIO_Pin_11                                  /* KEY键连接的端口 */
/*********************************************************************************************************
  PB端口定义

*//*********************************************************************************************************
  PC端口定义
*********************************************************************************************************/
#define __FRONT_R             GPIO_Pin_0                                  /*  前方右传感器输出的信号        */
#define __FRONTSIDE_R         GPIO_Pin_2                                  /*  45右传感器输出的信号      */
#define __RIGHTSIDE           GPIO_Pin_1                                  /*  右方传感器输出的信号        */
#define __START               GPIO_Pin_10                                  /* START键连接的端口 */
/*********************************************************************************************************
  PD端口定义
*********************************************************************************************************/

/*********************************************************************************************************
  PE端口定义
*********************************************************************************************************/
#define __DIR1                GPIO_Pin_2                                   /*  编码器 电机 M1 A相      */
#define __DIR2                GPIO_Pin_3                                   /*  编码器 电机 M2 A相      */

#define __FRONT_L             GPIO_Pin_15                                  /*  前方左传感器输出的信号        */
#define __FRONTSIDE_L         GPIO_Pin_13                                  /*  45左传感器输出的信号      */
#define __LEFTSIDE            GPIO_Pin_14                                  /*  左方传感器输出的信号        */
/*********************************************************************************************************
  常量宏定义--传感器
*********************************************************************************************************/
#define __LEFT              0                                           /*  左方传感器                  */
#define __FRONTL            1                                           /*  左前方传感器                */
#define __FRONT             2                                           /*  前方传感器                  */
#define __FRONTR            3                                           /*  右前方传感器                */
#define __RIGHT             4                                           /*  右方传感器                  */


/*********************************************************************************************************
  常量宏定义--电脑鼠状态
*********************************************************************************************************/
#define __STOP              0                                           /*  电脑鼠停止                  */
#define __GOAHEAD           1                                           /*  电脑鼠前进                  */
#define __GOAHEAD_45        2                                           /*  电脑鼠45度后前进            */
#define __TURNLEFT          3                                           /*  电脑鼠向左转                */
#define __TURNRIGHT         4                                           /*  电脑鼠向右转                */
#define __TURNBACK          5                                           /*  电脑鼠向后转                */
#define __TURNLEFTY         7                                           /*  电脑鼠向左转                */
#define __TURNRIGHTY        8                                           /*  电脑鼠向右转                */
#define __GOBACK            9
#define __TURNRIGHT_45      10                                         /*  电脑鼠向右转 */
#define __TURNLEFT_45       11 
#define __RUN45       12 
#define __RUN90VL       13 
#define __RUN90VR       14 
#define __AFTER          15
#define __SPURT45  16
/*********************************************************************************************************
  常量宏定义--电机加减速度
*********************************************************************************************************/
#define __SPEEDUP         0                                           /*  电机加速                    */
#define __SPEEDDOWN       1                                           /*  电机减速                */

/*********************************************************************************************************
  常量宏定义--电机状态
*********************************************************************************************************/
#define __MOTORSTOP         0                                           /*  电机停止                    */
#define __WAITONESTEP       1                                           /*  电机暂停一步                */
#define __MOTORRUN          2                                           /*  电机运行                    */


/*********************************************************************************************************
  常量宏定义--电机运行方向
*********************************************************************************************************/
#define __MOTORGOAHEAD      0                                           /*  电机前进                    */
#define __MOTORGOBACK       1                                           /*  电机后退                    */
#define __MOTORGOSTOP       2                                           /*  电机反向制动                */

/*********************************************************************************************************
  常量宏定义--PID
*********************************************************************************************************/
#define __KP 20 //80  //比例 30
#define __KI 0.1    //积分 0.01
#define __KD 0           //微分

#define U_MAX 1400       //返回的最大值,是pwm的周期值 
#define U_MIN 0 
#define error_IMAX 10     //积分限幅 
#define Deadband 3   //速度PID，设置死区范围


/*********************************************************************************************************
  结构体定义
*********************************************************************************************************/
struct __motor {
    int8    cState;                             /*  电机运行状态      */
    int8    cDir;                               /*  电机运行方向        */
    int8    cRealDir;                           /*  电机运行方向      */
    uint32  uiPulse;                           /*  电机需要运行的脉冲    */
    uint32  uiPulseCtr;                         /*  电机已运行的脉冲       */
    int16   sSpeed;                             /*  当前占空比        */
};
typedef struct __motor __MOTOR;

struct __pid       //定义数法核心数据 
{ 
    //uint16 usRef;      //速度PID，速度设定值 
    int16 usFeedBack;  //速度PID，速度反馈值//int16zzzzzzzzzzzzzzzzz
    uint16 usEncoder_new; //编码器
    uint16 usEncoder_last; //编码器
    
    float sRef;
    float sFeedBack;//int16zzzzzzzzzzzzzzzz
    float sPreError;  //速度PID，前一次，速度误差,,vi_Ref - vi_FeedBack 
    float sPreDerror; //速度PID，前一次，速度误差之差，d_error-PreDerror; 
  
    fp32 fKp;      //速度PID，Ka = Kp 
    fp32 fKi;      //速度PID，Kb = Kp * ( T / Ti ) 
    fp32 fKd;      //速度PID， 
       
    int16 iPreU;    //电机控制输出值      
};
typedef struct __pid __PID;


/*********************************************************************************************************
  常量宏定义
*********************************************************************************************************/


void voltageDetect(void);
void objectGoTo_CC (int8 cXdst, int8 cYdst);
uint8 keyCheck(void);
void mouseInit(void);
void mazeSearch(void);                                                  /*  前进N格                     */
void mouseTurnleft(void);                                               /*  向左转90度                  */
void mouseTurnright(void);                                              /*  向右转90度                  */
void mouseTurnback(void);                                               /*  向后转                      */
void mouseTurnback_H(void);
void sensorDebug(void);                                                 /*  传感器调试                  */
void voltageDetect(void);
void voltageDetect1(void);
//void mouseGoaheadhui(int8  cNBlock);                                       /*  前进N格                     */
//void mouseGoahead_Llow (int8  cNBlock);
void __UART0Init (void);
void SendChar(uint8 dat);
void SendStr(uint8 *s);
void mouseTurnback_Y(void);
void mouseTurnback_Y3(void);
void onestep(void);
void onestep1(void);
static void __delay(uint32  uiD);
static void __rightMotorContr(void);
static void __leftMotorContr(void);
static void __mouseCoorUpdate(void);
static void __irSendFreq(int8  __cNumber);
void __irCheck(void);
void ZEROcorrect(void);
static void __wallCheck(void);                                          /*  墙壁检测                    */
static void __sensorInit(void);
static void __MotorIint(void);
static void __keyInit(void);
static void __sysTickInit(void);
void MouseInit(void);
void __mazeInfDebug (void);
void mouseTurnleft_L(void);                                               /*  向左转90度                  */
void mouseTurnright_L(void);                                              /*  向右转90度                  */
void mouseGoahead_L(int8  cNBlock);                                       /*  前进N格                     */
void mouseTurnlefthui(void);                                               /*  向左转90度                  */
void mouseTurnrighthui(void);
void objectGoTo_CHui(int8 cXdst, int8 cYdst);  
void objectGoTo1(int8  cXdst, int8  cYdst);
void mouseStop(void); 
void mouseGo(void);
void __keyIntDisable (void);
void mouseTurnback_C(void);                                               /*  向后转                      */
void mouseTurnback_Yqidian(void);
void mouseTurnback_C1(void);                                               /*  向后转                      */
void mouseTurnleft_C(void);                                               /*  向左转90度                  */
void mouseTurnright_C(void);                                              /*  向右转90度                  */
void mouseTurnleft_KC(void);                                               /*  向左转90度                  */
void mouseTurnright_KC(void);                                              /*  向右转90度                  */
uint8 DenggaoCheck (void);
uint8 PulseCtrCheck (void);
void mouseTurnleft_Y(void);                                               /*  向左转90度                  */
void mouseTurnright_Y(void);                                              /*  向右转90度                  */
uint8 startCheck (void);
void GYRO_Z_Angle(void);
extern void mouseInit (void);
void AD_Init(void);
void test(void);
void mouseGoaheadhui_45 (uint8_t  cNBlock);
void mouseGoaheadhui_kk (uint8_t  cNBlock,uint16 OneStep);
void mouseTurnleft_45(void);
void mouseTurnright_45(void);
void testEncoder(void);


void mouseTurnleft_135_t(void);
void mouseTurnright_135_t(void);
void mouseTurnleft_135_v(void);
void mouseTurnright_135_v(void);
void mouseGoaheadhui_45(uint8_t  );
void mouseGoahead_liang(int8);
void mouseTurnleft_90_t(void);
void mouseTurnright_90_t(void);
void mouseTurnleft_90_v(void);
void mouseTurnright_90_v(void);
void mouseTurnleft_180(void);
void mouseTurnright_180(void);
void mouseTurnleft_45_t(void);
void mouseTurnright_45_t(void);
void mouseTurnleft_45_v(void);
void mouseTurnright_45_v(void);
#endif
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
