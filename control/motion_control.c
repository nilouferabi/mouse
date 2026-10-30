/*
** File Name:           motion_control.c
** Description:         运动控制层：PID、编码器、红外/陀螺采样、1 ms 控制环、动作函数
**
**   参数统一在 tuning.h；本文件只保留实现。
**   跨中断共享变量已加 volatile，-Os 下忙等不会再被折叠成死循环。
*********************************************************************************************************/
#include "motion_control.h"
#include "status_led.h"
#include "motion_internal.h"

/* 本文件内部函数原型 */
static void  __rightMotorContr(void);
static void  __leftMotorContr(void);
static void  __mouseCoorUpdate(void);
static void  __wallCheck(void);
static void  __mazeInfDebug(void);

/*
  全局变量
  说明：跨中断读写的变量必须 volatile；只在主循环使用的变量保持普通类型以减小开销。
*/

/* ---- 迷宫/坐标（主循环独占） ---- */
MAZECOOR GmcMouse                        = {0, 0};
uint8    GucMouseDir                     = UP;
uint8    GucMapBlock[MAZETYPE][MAZETYPE] = {0};
uint8    GucMapBlock0[MAZETYPE][MAZETYPE] = {0};
uint8    GucMapBlock1[MAZETYPE][MAZETYPE] = {{0x0f}};
uint8    GucCrossroad                    = 0;
uint8    GucDirTemp                      = 0;
uint16   GucFangXiang                    = 0;

/* ---- 电机 / PID ---- */
__MOTOR __GmLeft  = {0, 0, 0, 0, 0, 0};
__MOTOR __GmRight = {0, 0, 0, 0, 0, 0};
static __PID   __GmLPID;
static __PID   __GmRPID;
__PID          __GmSPID;
__PID          __GmWPID;

volatile uint8  __GucMouseState = __STOP;          /* 主循环写，SysTick 读        */
volatile int32  __GiMaxSpeed    = SEARCHSPEED;     /* 主循环写，SysTick 读        */
volatile int32  __GiMinSpeed    = SEARCHSPEED;

/* ---- 编码器/动作共享量 ---- */
static volatile uint32 GW;                                /* SysTick 写，动作函数读      */
static float           GsTpusle_T;                        /* SysTick 内使用              */
volatile uint8  GuiSpeedCtr = 0;                   /* 主循环写，SysTick 读        */
static uint16          GuiTpusle_LR = 0;                  /* 搜索段接弯补偿             */
static uint8           GucFrontNear = 0;                  /* 搜索段置位，转弯函数读      */
volatile float  jiao = 0;                          /* 陀螺积分角（ISR/主循环共享）*/
volatile uint32 GucSysTickMs = 0;                  /* 1 ms 计时，用于动作超时      */
static volatile float  leftdis = 0;
static volatile float  rightdis = 0;
static float           piancha_r;                         /* 转向偏置（主循环）          */
static float           piancha_l;

/* ---- 传感器（TIM6 中断写，主循环读） ---- */
static volatile uint8  __GucDistance[5] = {0};
volatile uint16        ucIRCheck[4];
uint16                 Sunlight[4];
static float           W_l = 0;
static float           W_r = 0;
volatile uint8         GucFrontJinju = 0;                 /* 前方近距，等高图用          */
volatile uint8         GucGoHead     = 0;                 /* START 起跑判据              */

/* ---- 陀螺 ---- */
u32           Angle_TLY[8] = {0};
volatile u32  Angle_TLY_Average = 0;
static u16            voltageDetectRef = TUNE_GYRO_ZERO_DEFAULT;

/* ---- 红外阈值（可由 Flash 覆盖；默认值集中在 tuning.h） ---- */
volatile uint16 GusDistance_L_Near  = TUNE_IR_L_NEAR_DEFAULT;
volatile uint16 GusDistance_L_Mid   = TUNE_IR_L_MID_DEFAULT;
volatile uint16 GusDistance_L_Far   = TUNE_IR_L_FAR_DEFAULT;
volatile uint16 GusDistance_R_Near  = TUNE_IR_R_NEAR_DEFAULT;
volatile uint16 GusDistance_R_Mid   = TUNE_IR_R_MID_DEFAULT;
volatile uint16 GusDistance_R_Far   = TUNE_IR_R_FAR_DEFAULT;
volatile uint16 GusDistance_FL_Near = TUNE_IR_FL_NEAR_DEFAULT;
volatile uint16 GusDistance_FR_Near = TUNE_IR_FR_NEAR_DEFAULT;
volatile uint16 GusDistance_FL_Far  = TUNE_IR_FL_FAR_DEFAULT;
volatile uint16 GusDistance_FR_Far  = TUNE_IR_FR_FAR_DEFAULT;

/* ---- 标定查表（mV → mm，201 点） ---- */
float disCheck3[201]={222.597,215.542,209.024,202.979,197.356,192.109,187.199,182.593,178.262,174.18,170.325,166.678,163.221,159.939,156.818,153.846,151.012,148.305,145.717,143.24,140.866,138.588,136.401,134.299,132.276,130.328,128.451,126.641,124.893,123.204,121.572,119.993,118.465,116.985,115.55,114.159,112.809,111.499,110.226,108.989,107.787,106.617,105.479,104.371,103.291,102.239,101.214,100.214,99.2392,98.2874,97.3583,96.4509,95.5646,94.6984,93.8517,93.0237,92.214,91.4217,90.6463,89.8872,89.1439,88.4159,87.7027,87.0037,86.3186,85.6469,84.9882,84.3421,83.7082,83.0861,82.4756,81.8762,81.2876,80.7096,80.1417,79.5839,79.0357,78.4969,77.9672,77.4464,76.9343,76.4306,75.9352,75.4477,74.9681,74.496,74.0313,73.5739,73.1235,72.6799,72.2431,71.8128,71.3889,70.9712,70.5597,70.154,69.7542,69.3601,68.9715,68.5884,68.2106,67.8379,67.4703,67.1077,66.75,66.397,66.0487,65.7049,65.3656,65.0307,64.7001,64.3736,64.0513,63.733,63.4186,63.1081,62.8014,62.4984,62.1991,61.9034,61.6111,61.3223,61.0369,60.7548,60.476,60.2004,59.9278,59.6584,59.392,59.1286,58.8681,58.6105,58.3556,58.1036,57.8543,57.6076,57.3636,57.1221,56.8832,56.6468,56.4128,56.1813,55.9521,55.7253,55.5008,55.2785,55.0584,54.8406,54.6249,54.4113,54.1998,53.9904,53.783,53.5775,53.3741,53.1726,52.9729,52.7752,52.5793,52.3852,52.1929,52.0024,51.8136,51.6265,51.4411,51.2574,51.0753,50.8949,50.716,50.5387,50.363,50.1887,50.016,49.8448,49.6751,49.5068,49.3399,49.1744,49.0103,48.8476,48.6863,48.5262,48.3675,48.2101,48.054,47.8991,47.7455,47.5931,47.4419,47.292,47.1432,46.9956,46.8491,46.7038,46.5597,46.4166,46.2746,46.1337,45.9939,45.8552,45.7175};
float disCheck2[201]={189.038,184.056,179.427,175.114,171.081,167.3,163.746,160.398,157.236,154.244,151.408,148.715,146.152,143.711,141.381,139.155,137.026,134.986,133.029,131.151,129.346,127.609,125.937,124.325,122.77,121.269,119.819,118.416,117.059,115.745,114.472,113.238,112.041,110.878,109.75,108.653,107.586,106.549,105.539,104.556,103.598,102.665,101.755,100.868,100.002,99.1569,98.3316,97.5254,96.7376,95.9676,95.2146,94.4781,93.7575,93.0522,92.3617,91.6855,91.0231,90.374,89.7379,89.1142,88.5027,87.9029,87.3144,86.7369,86.1701,85.6136,85.0672,84.5305,84.0033,83.4853,82.9762,82.4758,81.9838,81.5,81.0242,80.5562,80.0957,79.6426,79.1967,78.7578,78.3256,77.9001,77.4811,77.0684,76.6618,76.2613,75.8666,75.4776,75.0942,74.7163,74.3436,73.9762,73.6139,73.2566,72.9041,72.5564,72.2133,71.8747,71.5406,71.2109,70.8854,70.5641,70.2469,69.9336,69.6243,69.3189,69.0171,68.7191,68.4247,68.1338,67.8464,67.5624,67.2817,67.0043,66.7301,66.459,66.191,65.9261,65.6642,65.4051,65.1489,64.8956,64.645,64.3971,64.1519,63.9094,63.6693,63.4319,63.1969,62.9643,62.7342,62.5064,62.281,62.0578,61.8368,61.6181,61.4016,61.1872,60.9749,60.7646,60.5564,60.3502,60.146,59.9437,59.7433,59.5449,59.3482,59.1534,58.9604,58.7691,58.5796,58.3919,58.2058,58.0213,57.8385,57.6574,57.4778,57.2998,57.1234,56.9484,56.775,56.6031,56.4326,56.2636,56.096,55.9298,55.765,55.6015,55.4394,55.2786,55.1192,54.961,54.8041,54.6485,54.4941,54.3409,54.189,54.0382,53.8886,53.7402,53.593,53.4468,53.3018,53.1579,53.0151,52.8734,52.7327,52.5931,52.4545,52.317,52.1804,52.0449,51.9104,51.7768,51.6442,51.5126,51.3819,51.2521,51.1233,50.9954,50.8683};
float disCheck0[201]={440.174,422.152,405.657,390.497,376.513,363.572,351.557,340.37,329.928,320.155,310.989,302.374,294.259,286.602,279.364,272.511,266.012,259.84,253.97,248.38,243.05,237.962,233.099,228.446,223.99,219.718,215.619,211.683,207.898,204.258,200.752,197.375,194.118,190.976,187.941,185.01,182.175,179.433,176.779,174.209,171.718,169.303,166.961,164.687,162.48,160.336,158.252,156.226,154.256,152.338,150.472,148.654,146.883,145.157,143.475,141.834,140.234,138.672,137.147,135.658,134.204,132.783,131.394,130.037,128.709,127.411,126.14,124.897,123.68,122.488,121.321,120.178,119.058,117.96,116.884,115.829,114.795,113.78,112.784,111.807,110.849,109.908,108.984,108.077,107.186,106.31,105.45,104.605,103.775,102.959,102.156,101.367,100.591,99.8279,99.0771,98.3385,97.6117,96.8964,96.1924,95.4993,94.8171,94.1452,93.4836,92.8321,92.1902,91.5579,90.935,90.3211,89.7162,89.1199,88.5322,87.9529,87.3816,86.8184,86.263,85.7152,85.1749,84.6419,84.1161,83.5973,83.0854,82.5803,82.0817,81.5897,81.104,80.6245,80.1511,79.6837,79.2221,78.7663,78.3162,77.8716,77.4324,76.9986,76.57,76.1465,75.7281,75.3146,74.906,74.5022,74.103,73.7085,73.3185,72.9329,72.5517,72.1748,71.8021,71.4335,71.069,70.7086,70.352,69.9994,69.6505,69.3054,68.964,68.6262,68.292,67.9613,67.634,67.3101,66.9896,66.6723,66.3583,66.0475,65.7398,65.4352,65.1337,64.8351,64.5395,64.2468,63.957,63.67,63.3857,63.1042,62.8254,62.5493,62.2757,62.0048,61.7364,61.4705,61.207,60.946,60.6874,60.4312,60.1773,59.9256,59.6763,59.4292,59.1843,58.9416,58.701,58.4626,58.2262,57.9919,57.7596,57.5294,57.3011,57.0748,56.8504,56.6279,56.4073};
float disCheck1[201]={349.867,336.341,323.932,312.503,301.938,292.14,283.026,274.525,266.575,259.122,252.119,245.527,239.308,233.43,227.867,222.591,217.581,212.817,208.28,203.953,199.823,195.875,192.098,188.48,185.011,181.681,178.483,175.408,172.449,169.599,166.853,164.204,161.648,159.179,156.792,154.485,152.252,150.089,147.995,145.965,143.996,142.085,140.23,138.429,136.678,134.977,133.322,131.711,130.144,128.618,127.131,125.682,124.269,122.892,121.548,120.237,118.957,117.707,116.486,115.293,114.127,112.987,111.872,110.782,109.715,108.671,107.648,106.647,105.667,104.707,103.765,102.843,101.939,101.052,100.182,99.3294,98.4925,97.6711,96.865,96.0736,95.2965,94.5333,93.7836,93.0472,92.3235,91.6123,90.9133,90.2261,89.5504,88.8859,88.2324,87.5895,86.957,86.3346,85.7221,85.1193,84.5258,83.9415,83.3662,82.7997,82.2416,81.692,81.1504,80.6169,80.0911,79.5729,79.0622,78.5588,78.0624,77.5731,77.0905,76.6146,76.1453,75.6823,75.2256,74.775,74.3304,73.8916,73.4586,73.0313,72.6094,72.193,71.7819,71.376,70.9752,70.5794,70.1885,69.8024,69.421,69.0443,68.6721,68.3044,67.941,67.5819,67.2271,66.8764,66.5298,66.1871,65.8484,65.5135,65.1825,64.8551,64.5314,64.2113,63.8947,63.5816,63.2719,62.9656,62.6626,62.3628,62.0662,61.7727,61.4823,61.195,60.9106,60.6292,60.3507,60.075,59.8021,59.532,59.2646,58.9999,58.7378,58.4783,58.2213,57.9669,57.7149,57.4654,57.2182,56.9735,56.731,56.4909,56.253,56.0174,55.7839,55.5526,55.3235,55.0964,54.8715,54.6485,54.4276,54.2087,53.9918,53.7767,53.5636,53.3524,53.143,52.9354,52.7297,52.5257,52.3235,52.123,51.9242,51.7272,51.5318,51.338,51.1458,50.9553,50.7663,50.5789,50.3931,};

extern __IO uint16_t ADC_ConvertedValue[TUNE_ADC_CHANNELS];
uint8 zijiaozheng0 = 0;
static float W;

float ADCmv(int AD)
{
  return (float)(AD * TUNE_ADC_VREF_MV / TUNE_ADC_FULL_SCALE);
}

/*脉冲测试函数*/
void __delay(uint32 uiD)
{
    volatile uint32 i;
    for (i = uiD; i; i--);
}

void PIDInit(void)
{
    __GmLPID.usEncoder_new = TUNE_ENCODER_CENTER;
    __GmLPID.usFeedBack    = 0;
    __GmLPID.sFeedBack     = 0;

    __GmRPID.usEncoder_new = TUNE_ENCODER_CENTER;
    __GmRPID.usFeedBack    = 0;
    __GmRPID.sFeedBack     = 0;

    __GmSPID.sRef       = 0;
    __GmSPID.sFeedBack  = 0;
    __GmSPID.sPreError  = 0;
    __GmSPID.sPreDerror = 0;
    __GmSPID.fKp        = TUNE_PID_KP;
    __GmSPID.fKi        = TUNE_PID_KI;
    __GmSPID.fKd        = TUNE_PID_KD;
    __GmSPID.iPreU      = 0;

    __GmWPID.sRef       = 0;
    __GmWPID.sFeedBack  = 0;
    __GmWPID.sPreError  = 0;
    __GmWPID.sPreDerror = 0;
    __GmWPID.fKp        = TUNE_PID_KP;
    __GmWPID.fKi        = TUNE_PID_KI;
    __GmWPID.fKd        = TUNE_PID_KD;
    __GmWPID.iPreU      = 0;
}

void __SPIDContr(void)
{
    float  error, d_error, dd_error;
    static uint8 K_I = 1;

    error   = __GmSPID.sRef - __GmSPID.sFeedBack;
    d_error = error - __GmSPID.sPreError;
    dd_error = d_error - __GmSPID.sPreDerror;

    if (error > TUNE_PID_DEADBAND)
        error -= TUNE_PID_DEADBAND;
    else if (error < -TUNE_PID_DEADBAND)
        error += TUNE_PID_DEADBAND;
    else
        error = 0;

    if ((error > TUNE_PID_ERROR_IMAX) || (error < -TUNE_PID_ERROR_IMAX))
        K_I = 0;
    else
        K_I = 1;

    __GmSPID.sPreError  = error;
    __GmSPID.sPreDerror = d_error;

    __GmSPID.iPreU += (int16)(__GmSPID.fKp * d_error
                              + K_I * __GmSPID.fKi * error
                              + __GmSPID.fKd * dd_error);

    /* 只防止 int16 溢出，不改变正常范围 */
    if (__GmSPID.iPreU > TUNE_PID_I_LIMIT)
        __GmSPID.iPreU = TUNE_PID_I_LIMIT;
    else if (__GmSPID.iPreU < -TUNE_PID_I_LIMIT)
        __GmSPID.iPreU = -TUNE_PID_I_LIMIT;
}

void __WPIDContr(void)
{
    float  error, d_error, dd_error;
    static uint8 K_I = 1;

    error = __GmWPID.sRef + TUNE_PID_WALL_FEEDFORWARD * GsTpusle_T
            - __GmWPID.sFeedBack;
    d_error = error - __GmWPID.sPreError;
    dd_error = d_error - __GmWPID.sPreDerror;

    if (error > TUNE_PID_DEADBAND)
        error -= TUNE_PID_DEADBAND;
    else if (error < -TUNE_PID_DEADBAND)
        error += TUNE_PID_DEADBAND;
    else
        error = 0;

    if ((error > TUNE_PID_ERROR_IMAX) || (error < -TUNE_PID_ERROR_IMAX))
        K_I = 0;
    else
        K_I = 1;

    __GmWPID.sPreError  = error;
    __GmWPID.sPreDerror = d_error;

    __GmWPID.iPreU += (int16)(__GmWPID.fKp * d_error
                              + K_I * __GmWPID.fKi * error
                              + __GmWPID.fKd * dd_error);

    if (__GmWPID.iPreU > TUNE_PID_I_LIMIT)
        __GmWPID.iPreU = TUNE_PID_I_LIMIT;
    else if (__GmWPID.iPreU < -TUNE_PID_I_LIMIT)
        __GmWPID.iPreU = -TUNE_PID_I_LIMIT;
}

static void __setMotor(__MOTOR *motor, int16 speed)
{
    if (speed >= 0)
        motor->cDir = __MOTORGOAHEAD;
    else
    {
        motor->cDir = __MOTORGOBACK;
        speed = -speed;
    }

    if (speed > TUNE_PID_U_MAX)
        speed = TUNE_PID_U_MAX;
    if (speed < TUNE_PID_U_MIN)
        speed = TUNE_PID_U_MIN;

    motor->sSpeed = speed;
}

void __PIDContr(void)
{
    __SPIDContr();
    __WPIDContr();

    __setMotor(&__GmLeft,  __GmSPID.iPreU - __GmWPID.iPreU);
    __setMotor(&__GmRight, __GmSPID.iPreU + __GmWPID.iPreU);

    __rightMotorContr();
    __leftMotorContr();
}

void __Encoder(void)
{
    uint16 dir_l, dir_r;

    __GmLPID.usEncoder_new = TIM_GetCounter(TIM2);
    __GmRPID.usEncoder_new = TIM_GetCounter(TIM3);

    dir_r = (TIM3->CR1 & 0x0010) >> 4;
    dir_l = (TIM2->CR1 & 0x0010) >> 4;

    if (dir_l == 1)                             /* 向下计数：后退 */
    {
        __GmLPID.usFeedBack = TUNE_ENCODER_CENTER - __GmLPID.usEncoder_new;
        TIM_SetCounter(TIM2, TUNE_ENCODER_CENTER);
        __GmLeft.uiPulseCtr += __GmLPID.usFeedBack;
        __GmLeft.cRealDir = __MOTORGOBACK;
        __GmLPID.sFeedBack = -1 * __GmLPID.usFeedBack;
    }
    else
    {
        __GmLPID.usFeedBack = __GmLPID.usEncoder_new - TUNE_ENCODER_CENTER;
        TIM_SetCounter(TIM2, TUNE_ENCODER_CENTER);
        __GmLeft.uiPulseCtr += __GmLPID.usFeedBack;
        __GmLeft.cRealDir = __MOTORGOAHEAD;
        __GmLPID.sFeedBack = __GmLPID.usFeedBack;
    }

    if (dir_r == 1)                             /* 向下计数：前进 */
    {
        __GmRPID.usFeedBack = TUNE_ENCODER_CENTER - __GmRPID.usEncoder_new;
        TIM_SetCounter(TIM3, TUNE_ENCODER_CENTER);
        __GmRight.uiPulseCtr += __GmRPID.usFeedBack;
        __GmRight.cRealDir = __MOTORGOAHEAD;
        __GmRPID.sFeedBack = __GmRPID.usFeedBack;
    }
    else
    {
        __GmRPID.usFeedBack = __GmRPID.usEncoder_new - TUNE_ENCODER_CENTER;
        TIM_SetCounter(TIM3, TUNE_ENCODER_CENTER);
        __GmRight.uiPulseCtr += __GmRPID.usFeedBack;
        __GmRight.cRealDir = __MOTORGOBACK;
        __GmRPID.sFeedBack = -1 * __GmRPID.usFeedBack;
    }

    __GmSPID.sFeedBack = (__GmRPID.sFeedBack + __GmLPID.sFeedBack) / 2;
    __GmWPID.sFeedBack = (__GmRPID.sFeedBack - __GmLPID.sFeedBack) / 2;
}

void __irSendFreq(int8 __cNumber)
{
    switch (__cNumber)
    {
      case 1:  GPIO_SetBits(GPIOA, GPIO_Pin_5);  break;   /* 前左发射 */
      case 2:  GPIO_SetBits(GPIOC, GPIO_Pin_13); break;   /* 前右发射 */
      case 3:  GPIO_SetBits(GPIOA, GPIO_Pin_3);  break;   /* 左发射   */
      case 4:  GPIO_SetBits(GPIOC, GPIO_Pin_2);  break;   /* 右发射   */
      default: break;
    }
}

void sensorDebug(void)
{
    displayDownload(2, 0, 0, __GucDistance[__LEFT  ]);
    displayDownload(2, 1, 0, __GucDistance[__FRONTL]);
    displayDownload(2, 2, 0, __GucDistance[__FRONTR]);
    displayDownload(2, 3, 0, __GucDistance[__RIGHT ]);
}

static void __rightMotorContr(void)
{
    switch (__GmRight.cDir)
    {
    case __MOTORGOAHEAD:
        TIM_SetCompare1(TIM1, __GmRight.sSpeed);
        TIM_SetCompare2(TIM1, 0);
        break;

    case __MOTORGOBACK:
        TIM_SetCompare1(TIM1, 0);
        TIM_SetCompare2(TIM1, __GmRight.sSpeed);
        break;

    case __MOTORGOSTOP:
        TIM_SetCompare1(TIM1, 0);
        TIM_SetCompare2(TIM1, 0);
        break;

    default:
        break;
    }
}

static void __leftMotorContr(void)
{
    switch (__GmLeft.cDir)
    {
    case __MOTORGOAHEAD:
        TIM_SetCompare4(TIM1, 0);
        TIM_SetCompare3(TIM1, __GmLeft.sSpeed);
        break;

    case __MOTORGOBACK:
        TIM_SetCompare4(TIM1, __GmLeft.sSpeed);
        TIM_SetCompare3(TIM1, 0);
        break;

    case __MOTORGOSTOP:
        TIM_SetCompare3(TIM1, 0);
        TIM_SetCompare4(TIM1, 0);
        break;

    default:
        break;
    }
}

void __SpeedUp(void)
{
    uint16 speed = __GmSPID.sFeedBack;
    if (__GmSPID.sRef < __GiMaxSpeed)
    {
        if (speed >= __GmSPID.sRef)
            __GmSPID.sRef += TUNE_SPEED_ACCEL_STEP;
    }
}

void __SpeedDown(void)
{
    uint16 speed = __GmSPID.sFeedBack;
    if (__GmSPID.sRef >= MINSPEED)
    {
        if (speed <= __GmSPID.sRef + TUNE_SPEED_DECEL_MARGIN)
            __GmSPID.sRef -= TUNE_SPEED_DECEL_STEP;
    }
}

uint8 startCheck(void)
{
    if (GPIO_ReadInputDataBit(GPIOC, __START) == 0)
    {
        __delay(50);                                /* 简单消抖 */
        while (GPIO_ReadInputDataBit(GPIOC, __START) == 0);
        return true;
    }

    return false;
}

void TIM3_IRQHandler(void)
{
    TIM_ClearFlag(TIM3, TIM_FLAG_Update);
}

u32 GyroscopeDebug[8] = {0};
/*
SourceDate : 采样值存储地址
NewDate : 新采样值
Average ：返回值
*/
void GyroscopeFilter(u32 NewDate, u32 *SourceDate, volatile u32 *Average)
{
    uint8 i;
    u32 NumMax, NumMin;
    uint8 MaxCount = 0, MinCount = 0;
    u32 Sum;

    for (i = 0; i < TUNE_IR_FILTER_SIZE - 1; i++)
        SourceDate[i] = SourceDate[i + 1];
    SourceDate[TUNE_IR_FILTER_SIZE - 1] = NewDate;

    NumMax = SourceDate[0];
    NumMin = SourceDate[0];

    for (i = 1; i < TUNE_IR_FILTER_SIZE; i++)
    {
        if (SourceDate[i] > NumMax)
        {
            NumMax = SourceDate[i];
            MaxCount = i;
        }
        else if (SourceDate[i] < NumMin)
        {
            NumMin = SourceDate[i];
            MinCount = i;
        }
    }

    if (NumMax == NumMin)
    {
        *Average = NumMax;
        return;
    }

    Sum = 0;
    for (i = 0; i < TUNE_IR_FILTER_SIZE; i++)
    {
        if ((i != MaxCount) && (i != MinCount))
            Sum += SourceDate[i];
    }
    *Average = Sum / TUNE_IR_FILTER_SUM_COUNT;
}

static float __irDistance(float mv, const float *table,
                          uint16 offset, uint16 step,
                          uint16 *index, float *frac)
{
    if (mv < offset)
        mv = offset;

    *index = (uint16)((mv - offset) / step);
    *frac  = (float)(((int)mv - offset) % step) / step;

    return (table[*index] - table[*index + 1]) * (*frac) + table[*index + 1];
}

void TIM6_IRQHandler(void)
{
    static uint8  ucState = 0;
    static uint16 integerr = 0, integer = 0;
    static float  decimalr = 0, decimal = 0;
    float mv;

    TIM_ClearFlag(TIM6, TIM_FLAG_Update);
    GyroscopeFilter(ADC_ConvertedValue[4], Angle_TLY, &Angle_TLY_Average);

    switch (ucState)
    {
    case 0:                                             /* 右红外：先记环境光 */
        Sunlight[3] = ADC_ConvertedValue[3];
        ucState++;
        __irSendFreq(4);
        break;

    case 1:                                             /* 右红外：测反射光 */
        ucIRCheck[3] = ADC_ConvertedValue[3] - Sunlight[3];
        if (ucIRCheck[3] > TUNE_IR_R_SAT)
            ucIRCheck[3] = 0;

        mv = ADCmv(ucIRCheck[3]);
        rightdis = __irDistance(mv, disCheck3, TUNE_IR_R_OFFSET_MV,
                                TUNE_IR_R_STEP_MV, &integer, &decimal);

        GPIO_ResetBits(GPIOC, GPIO_Pin_2);

        if (ucIRCheck[3] > GusDistance_R_Far)
            __GucDistance[__RIGHT] |= 0x01;
        else
            __GucDistance[__RIGHT] &= 0xfe;

        if (ucIRCheck[3] > GusDistance_R_Mid)
            __GucDistance[__RIGHT] |= 0x02;
        else
            __GucDistance[__RIGHT] &= 0xfd;

        if (ucIRCheck[3] > GusDistance_R_Near)
            __GucDistance[__RIGHT] |= 0x04;
        else
            __GucDistance[__RIGHT] &= 0xfb;

        ucState++;
        break;

    case 2:                                             /* 前左：记环境光 */
        Sunlight[0] = ADC_ConvertedValue[0];
        ucState++;
        __irSendFreq(1);
        break;

    case 3:                                             /* 前左：测反射光 */
        ucIRCheck[0] = ADC_ConvertedValue[0] - Sunlight[0];
        if (ucIRCheck[0] > TUNE_IR_FRONT_SAT)
            ucIRCheck[0] = 0;

        GPIO_ResetBits(GPIOA, GPIO_Pin_5);

        if (ucIRCheck[0] > GusDistance_FL_Far)
            __GucDistance[__FRONTL] |= 0x01;
        else
            __GucDistance[__FRONTL] &= 0xfe;

        if (ucIRCheck[0] > GusDistance_FL_Near)
            __GucDistance[__FRONTL] |= 0x02;
        else
            __GucDistance[__FRONTL] &= 0xfd;

        ucState++;
        break;

    case 4:                                             /* 前右：记环境光 */
        Sunlight[1] = ADC_ConvertedValue[1];
        ucState++;
        __irSendFreq(2);
        break;

    case 5:                                             /* 前右：测反射光 */
        ucIRCheck[1] = ADC_ConvertedValue[1] - Sunlight[1];
        if (ucIRCheck[1] > TUNE_IR_FRONT_SAT)
            ucIRCheck[1] = 0;

        GPIO_ResetBits(GPIOC, GPIO_Pin_13);

        if (ucIRCheck[1] > GusDistance_FR_Far)
            __GucDistance[__FRONTR] |= 0x01;
        else
            __GucDistance[__FRONTR] &= 0xfe;

        if (ucIRCheck[1] > GusDistance_FR_Near)
            __GucDistance[__FRONTR] |= 0x02;
        else
            __GucDistance[__FRONTR] &= 0xfd;

        /* START 起跑判据：左前有墙且右前无墙 */
        if ((ucIRCheck[0] > GusDistance_FL_Far) &&
            (ucIRCheck[1] < GusDistance_FR_Far))
            GucGoHead = 1;
        else
            GucGoHead = 0;

        ucState++;
        break;

    case 6:                                             /* 左红外：记环境光 */
        Sunlight[2] = ADC_ConvertedValue[2];
        ucState++;
        __irSendFreq(3);
        break;

    case 7:                                             /* 左红外：测反射光 */
        ucIRCheck[2] = ADC_ConvertedValue[2] - Sunlight[2];
        if (ucIRCheck[2] > TUNE_IR_FRONT_SAT)
            ucIRCheck[2] = 0;

        mv = ADCmv(ucIRCheck[2]);
        leftdis = __irDistance(mv, disCheck2, TUNE_IR_R_OFFSET_MV,
                               TUNE_IR_R_STEP_MV, &integerr, &decimalr);

        GPIO_ResetBits(GPIOA, GPIO_Pin_3);

        if (ucIRCheck[2] > GusDistance_L_Far)
            __GucDistance[__LEFT] |= 0x01;
        else
            __GucDistance[__LEFT] &= 0xfe;

        if (ucIRCheck[2] > GusDistance_L_Mid)
            __GucDistance[__LEFT] |= 0x02;
        else
            __GucDistance[__LEFT] &= 0xfd;

        if (ucIRCheck[2] > GusDistance_L_Near)
            __GucDistance[__LEFT] |= 0x04;
        else
            __GucDistance[__LEFT] &= 0xfb;

        GucFrontJinju = ((__GucDistance[__FRONTR] & 0x02) &&
                         (__GucDistance[__FRONTL] & 0x02)) ? 1 : 0;

        ucState++;
        break;

    default:
        break;
    }

    ucState %= TUNE_IR_STATE_COUNT;
}

/*********************************************************************************************************
** Function name:       SysTick_Handler
** Descriptions:        定时中断扫描。
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
uint8 end_speedr=0;
uint8 end_speedl=0;
uint8 end_RT45V=0;
uint8 end_LT45V=0;
uint8 end_RT135L=0;
uint8 end_LT135L=0;
uint8 end_LT90V=0;
uint8 end_RT90V=0;
uint8 end_LT180=0;
uint8 end_RT180=0;

uint16 start_flag=0;
uint16 start_flag1=0;
uint16 start_RT45Vflag=0;
uint16 start_LT45Vflag=0;
uint16 start_RT135Lflag=0;
uint16 start_LT135Lflag=0;
uint16 start_LT90Vflag=0;
uint16 start_RT90Vflag=0;
uint16 start_LT180flag=0;
uint16 start_RT180flag=0;

static float zijiaozheng=0;
uint16 zijiaozheng_time=0;
static float sum_zijiaozheng=0;
uint8 zijiaozheng_flag=1;

void SysTick_Handler(void)
{
    static uint8 k = 0;

    GucSysTickMs++;
    Led_Tick();

    if (zijiaozheng0 == 1)
    {
        if (zijiaozheng_flag)
        {
            sum_zijiaozheng += Angle_TLY_Average;
            zijiaozheng_time++;
            if (zijiaozheng_time == TUNE_GYRO_CAL_SAMPLES)
            {
                zijiaozheng = sum_zijiaozheng / TUNE_GYRO_CAL_SAMPLES;
                zijiaozheng_flag = 0;
                jiao = 0;
            }
        }
    }

    __Encoder();
    TIM6_IRQHandler();                  /* 保留原行为：与 TIM6 中断构成双触发 */

    switch (__GmRight.cState)
    {
    case __MOTORSTOP:
        __GmRight.uiPulse    = 0;
        __GmRight.uiPulseCtr = 0;
        __GmLeft.uiPulse     = 0;
        __GmLeft.uiPulseCtr  = 0;
        break;

    case __MOTORRUN:
        if (__GucMouseState == __GOAHEAD)
        {
            if (leftdis < TUNE_WALL_L_ACTIVE_DIST)
            {
                GsTpusle_T = TUNE_WALL_L_GAIN * (leftdis - TUNE_WALL_L_TARGET_DIST);
            }
            else if (rightdis < TUNE_WALL_R_ACTIVE_DIST)
            {
                GsTpusle_T = -TUNE_WALL_R_GAIN * (rightdis - TUNE_WALL_R_TARGET_DIST);
            }
            else
            {
                GsTpusle_T = 0;
            }

            __PIDContr();               /* 原代码在本状态会算两次 PID，勿合并 */

            if (GuiSpeedCtr == __SPEEDUP)
            {
                k = (k + 1) % TUNE_SPEED_ACCEL_DIV;
                if (k == TUNE_SPEED_ACCEL_DIV - 1)
                    __SpeedUp();
            }
            else if (GuiSpeedCtr == __SPEEDDOWN)
            {
                k = (k + 1) % TUNE_SPEED_DECEL_DIV;
                if (k == TUNE_SPEED_DECEL_DIV / 2)
                    __SpeedDown();
            }
        }
        else if (__GucMouseState == __GOAHEAD_45)
        {
            /* 斜向直行：不用侧墙找正，靠陀螺积分角保持航向 */
            GsTpusle_T = TUNE_8DIR_GYRO_FF * jiao;
            voltageDetect1();
            voltageDetect();

            if (GuiSpeedCtr == __SPEEDUP)
            {
                k = (k + 1) % TUNE_SPEED_ACCEL_DIV;
                if (k == TUNE_SPEED_ACCEL_DIV - 1)
                    __SpeedUp();
            }
            else if (GuiSpeedCtr == __SPEEDDOWN)
            {
                k = (k + 1) % TUNE_SPEED_DECEL_DIV;
                if (k == TUNE_SPEED_DECEL_DIV / 2)
                    __SpeedDown();
            }
        }
        else
        {
            GsTpusle_T = 0;
            voltageDetect1();
            voltageDetect();
        }

        __PIDContr();
        break;

    default:
        break;
    }
}

void mouseGoahead_L(int8 cNBlock)
{
    int8  cL = 0, cR = 0, cCoor = 1, cB;
    int16 GuiTpusle_LR_l = 0, GuiTpusle_LR_r = 0;

    GuiSpeedCtr = __SPEEDUP;
    if (__GmLeft.cState)
        cCoor = 0;

    if (cNBlock == 1)
    {
        cL = 1;
        cR = 1;
        if (GucFangXiang == GucDirTemp)
        {
            GuiTpusle_LR = TUNE_SEARCH_GO_ADVANCE_SAME;
            if (GucDirTemp == 2) { GuiTpusle_LR_r = 0; W_l = 0; }
            if (GucDirTemp == 6) { GuiTpusle_LR_l = 0; W_r = 0; }
        }
        else
        {
            GuiTpusle_LR = TUNE_SEARCH_GO_ADVANCE_TURN;
            if (GucDirTemp == 2) { GuiTpusle_LR_r = 0; W_l = 0; }
            if (GucDirTemp == 6) { GuiTpusle_LR_l = 0; W_r = 0; }
        }
        __GiMaxSpeed = TUNE_GO_SPEED_CORNER;
    }
    else
    {
        GuiTpusle_LR = 0;
        __GiMaxSpeed = TUNE_GO_SPEED_IDLE;
    }

    GucFangXiang = GucDirTemp;

    if (((GmcMouse.cX == 7) && (GmcMouse.cY == 7)) ||
        ((GmcMouse.cX == 8) && (GmcMouse.cY == 8)) ||
        ((GmcMouse.cX == 7) && (GmcMouse.cY == 8)) ||
        ((GmcMouse.cX == 8) && (GmcMouse.cY == 7)))
    {
        cL = 0;
        cR = 0;
        GuiTpusle_LR = 0;
    }

    if ((__GucMouseState == __TURNRIGHT) || (__GucMouseState == __TURNLEFT))
    {
        __GmLeft.uiPulseCtr  = TUNE_TURN_HANDOFF_PULSE;
        __GmRight.uiPulseCtr = TUNE_TURN_HANDOFF_PULSE;
    }

    cB = cNBlock;
    __GucMouseState   = __GOAHEAD;
    __GmRight.uiPulse += cNBlock * TUNE_PULSE_PER_BLOCK;
    __GmLeft.uiPulse  += cNBlock * TUNE_PULSE_PER_BLOCK;
    __GmRight.cState   = __MOTORRUN;
    __GmLeft.cState    = __MOTORRUN;

    if (cNBlock >= 2)
        __GiMaxSpeed = TUNE_GO_SPEED_CRUISE;

    while (__GmLeft.cState != __MOTORSTOP)
    {
        if (__GmLeft.uiPulseCtr >= TUNE_PULSE_PER_BLOCK)
        {
            __GmLeft.uiPulse    -= TUNE_PULSE_PER_BLOCK;
            __GmLeft.uiPulseCtr -= TUNE_PULSE_PER_BLOCK;
            if (cCoor)
            {
                cNBlock--;
                if (cNBlock == 0)
                    goto End;
                if (cNBlock < cB - 1)
                    GuiSpeedCtr = __SPEEDUP;
            }
            else
            {
                cCoor = 1;
            }
        }

        if (__GmRight.uiPulseCtr >= TUNE_PULSE_PER_BLOCK)
        {
            __GmRight.uiPulse    -= TUNE_PULSE_PER_BLOCK;
            __GmRight.uiPulseCtr -= TUNE_PULSE_PER_BLOCK;
        }

        if (cNBlock < 2)
        {
            if (__GmSPID.sFeedBack > TUNE_GO_SPEED_SLOW)
            {
                GuiSpeedCtr = TUNE_SPEED_HOLD;
                __GmSPID.sRef = TUNE_GO_SPEED_SLOW;
            }

            if (cL)
            {
                if (ucIRCheck[2] < GusDistance_L_Far)
                {
                    __GmRight.uiPulse = __GmRight.uiPulseCtr
                                        + TUNE_SEARCH_GO_ADVANCE_LEFT - GuiTpusle_LR - GuiTpusle_LR_l;
                    __GmLeft.uiPulse  = __GmLeft.uiPulseCtr
                                        + TUNE_SEARCH_GO_ADVANCE_LEFT - GuiTpusle_LR - GuiTpusle_LR_l;
                    while (ucIRCheck[2] < GusDistance_L_Far)
                    {
                        if ((__GmLeft.uiPulseCtr + TUNE_SEARCH_GO_MARGIN) > __GmLeft.uiPulse)
                            goto End;
                    }
                }
            }
            else
            {
                if (ucIRCheck[2] > GusDistance_L_Far)
                    cL = 1;
            }

            if (cR)
            {
                if (ucIRCheck[3] < GusDistance_R_Far)
                {
                    __GmRight.uiPulse = __GmRight.uiPulseCtr
                                        + TUNE_SEARCH_GO_ADVANCE_RIGHT - GuiTpusle_LR - GuiTpusle_LR_r;
                    __GmLeft.uiPulse  = __GmLeft.uiPulseCtr
                                        + TUNE_SEARCH_GO_ADVANCE_RIGHT - GuiTpusle_LR - GuiTpusle_LR_r;
                    while (ucIRCheck[3] < GusDistance_R_Far)
                    {
                        if ((__GmLeft.uiPulseCtr + TUNE_SEARCH_GO_MARGIN) > __GmLeft.uiPulse)
                            goto End;
                    }
                }
            }
            else
            {
                if (ucIRCheck[3] > GusDistance_R_Far)
                    cR = 1;
            }
        }
    }

End:
    ;
}

void mouseStop(void)
{
    __GmSPID.sRef = 0;
    __GmWPID.sRef = 0;
    GuiSpeedCtr = TUNE_SPEED_STOP;
}

/****************************************************************************************************
** Function name:       mouseTurnright
** Descriptions:        右转
** input parameters:    无
** output parameters:   无
** Returned value:      无         按步数转弯
*********************************************************************************************************/
void mouseTurnright(void)
{
    GW = 0;
    __GucMouseState = __TURNRIGHT;
    __GmRight.cState = __MOTORRUN;
    __GmLeft.cState  = __MOTORRUN;
    GucMouseDir = (GucMouseDir + 2) % 8;

    piancha_r = (leftdis - TUNE_TURN_L_TARGET_DIST) / TUNE_TURN_PIANCHA_DIV;
    if (leftdis > TUNE_TURN_L_CLEAR_DIST)
        piancha_r = 0;
    if (piancha_r > TUNE_TURN_PIANCHA_MAX)
        piancha_r = TUNE_TURN_PIANCHA_MAX;
    if (piancha_r < -TUNE_TURN_PIANCHA_MAX)
        piancha_r = -TUNE_TURN_PIANCHA_MAX;

    __GmSPID.sRef  = TUNE_TURN_SPEED;
    __GmWPID.sRef  = -TUNE_TURN_WPID_R;

    while (1)
    {
        W = (float)GW * TUNE_GYRO_GW_SCALE / TUNE_ADC_FULL_SCALE / 2;
        if (W > TUNE_TURN90_W_SEARCH_R)
            break;
    }

    __GmWPID.sRef = 0;
    __GucMouseState = __GOAHEAD;
    GuiSpeedCtr = TUNE_SPEED_HOLD;

    __GmLeft.uiPulse = TUNE_TURN90_PULSE_SEARCH_R - piancha_r * TUNE_TURN_PULSE_PER_DEG;
    __GmLeft.uiPulseCtr = 0;
    __GmRight.uiPulse = TUNE_TURN90_PULSE_SEARCH_R - piancha_r * TUNE_TURN_PULSE_PER_DEG;
    __GmRight.uiPulseCtr = 0;

    while ((__GmRight.uiPulseCtr + TUNE_TURN_PULSE_MARGIN) <= __GmRight.uiPulse);
    while ((__GmLeft.uiPulseCtr + TUNE_TURN_PULSE_MARGIN) <= __GmLeft.uiPulse);

    __GucMouseState = __TURNRIGHT;

    GuiSpeedCtr = __SPEEDUP;
    __GmRight.cState = __MOTORSTOP;
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
}

/*********************************************************************************************************
** Function name:       mouseTurnleft
** Descriptions:        左转
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/

void mouseTurnleft(void)
{
    piancha_l = (rightdis - TUNE_TURN_R_TARGET_DIST) / TUNE_TURN_PIANCHA_DIV;
    if (rightdis > TUNE_TURN_R_CLEAR_DIST)
        piancha_l = 0;
    if (piancha_l > TUNE_TURN_PIANCHA_MAX)
        piancha_l = TUNE_TURN_PIANCHA_MAX;
    if (piancha_l < -TUNE_TURN_PIANCHA_MAX)
        piancha_l = -TUNE_TURN_PIANCHA_MAX;

    __GucMouseState = __TURNLEFT;
    __GmRight.cState = __MOTORRUN;
    __GmLeft.cState  = __MOTORRUN;
    GucMouseDir = (GucMouseDir + 6) % 8;

    __GmSPID.sRef = TUNE_TURN_SPEED;
    __GmWPID.sRef = TUNE_TURN_WPID_L;
    GW = 0;

    while (1)
    {
        W = (float)GW * TUNE_GYRO_GW_SCALE / TUNE_ADC_FULL_SCALE / 2;
        if (W > TUNE_TURN90_W_SEARCH_L)
            break;
    }

    __GmWPID.sRef = 0;
    __GucMouseState = __GOAHEAD;
    GuiSpeedCtr = TUNE_SPEED_HOLD;

    __GmLeft.uiPulse = TUNE_TURN90_PULSE_SEARCH_L - piancha_l * TUNE_TURN_PULSE_PER_DEG;
    __GmLeft.uiPulseCtr = 0;
    __GmRight.uiPulse = TUNE_TURN90_PULSE_SEARCH_L - piancha_l * TUNE_TURN_PULSE_PER_DEG;
    __GmRight.uiPulseCtr = 0;

    while ((__GmRight.uiPulseCtr + TUNE_TURN_PULSE_MARGIN) <= __GmRight.uiPulse);
    while ((__GmLeft.uiPulseCtr + TUNE_TURN_PULSE_MARGIN) <= __GmLeft.uiPulse);

    __GucMouseState = __TURNLEFT;

    GuiSpeedCtr = __SPEEDUP;
    __GmRight.cState = __MOTORSTOP;
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
}

 

void mouseTurnright_C(void)
{
    while ((__GmRight.uiPulseCtr + TUNE_TURN_PULSE_MARGIN) <= __GmRight.uiPulse);
    while ((__GmLeft.uiPulseCtr + TUNE_TURN_PULSE_MARGIN) <= __GmLeft.uiPulse);

    GW = 0;
    __GucMouseState = __TURNRIGHT;
    __GmRight.cState = __MOTORRUN;
    __GmLeft.cState  = __MOTORRUN;
    GucMouseDir = (GucMouseDir + 2) % 8;

    piancha_r = (leftdis - TUNE_TURN_L_TARGET_DIST) / TUNE_TURN_PIANCHA_DIV;
    if (leftdis > TUNE_TURN_L_CLEAR_DIST)
        piancha_r = 0;
    if (piancha_r > TUNE_TURN_PIANCHA_MAX)
        piancha_r = TUNE_TURN_PIANCHA_MAX;
    if (piancha_r < -TUNE_TURN_PIANCHA_MAX)
        piancha_r = -TUNE_TURN_PIANCHA_MAX;

    __GmSPID.sRef = TUNE_TURN_SPEED;
    __GmWPID.sRef = -TUNE_TURN_WPID_R;

    while (1)
    {
        W = (float)GW * TUNE_GYRO_GW_SCALE / TUNE_ADC_FULL_SCALE / 2;
        if (W > TUNE_TURN90_W_C_R)
            break;
    }

    __GmWPID.sRef = 0;
    __GucMouseState = __GOAHEAD;
    GuiSpeedCtr = TUNE_SPEED_HOLD;

    __GmLeft.uiPulse = TUNE_TURN90_PULSE_C_R - piancha_r * TUNE_TURN_PULSE_PER_DEG;
    __GmLeft.uiPulseCtr = 0;
    __GmRight.uiPulse = TUNE_TURN90_PULSE_C_R - piancha_r * TUNE_TURN_PULSE_PER_DEG;
    __GmRight.uiPulseCtr = 0;

    while ((__GmRight.uiPulseCtr + TUNE_TURN_PULSE_MARGIN) <= __GmRight.uiPulse);
    while ((__GmLeft.uiPulseCtr + TUNE_TURN_PULSE_MARGIN) <= __GmLeft.uiPulse);

    __GucMouseState = __TURNRIGHT;

    GuiSpeedCtr = __SPEEDUP;
    __GmRight.cState = __MOTORSTOP;
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
}

void mouseTurnleft_C(void)
{
    while ((__GmRight.uiPulseCtr + TUNE_TURN_PULSE_MARGIN) <= __GmRight.uiPulse);
    while ((__GmLeft.uiPulseCtr + TUNE_TURN_PULSE_MARGIN) <= __GmLeft.uiPulse);

    piancha_l = (rightdis - TUNE_TURN_R_TARGET_DIST) / TUNE_TURN_PIANCHA_DIV;
    if (rightdis > TUNE_TURN_R_CLEAR_DIST)
        piancha_l = 0;
    if (piancha_l > TUNE_TURN_PIANCHA_MAX)
        piancha_l = TUNE_TURN_PIANCHA_MAX;
    if (piancha_l < -TUNE_TURN_PIANCHA_MAX)
        piancha_l = -TUNE_TURN_PIANCHA_MAX;

    __GucMouseState = __TURNLEFT;
    __GmRight.cState = __MOTORRUN;
    __GmLeft.cState  = __MOTORRUN;
    GucMouseDir = (GucMouseDir + 6) % 8;

    __GmSPID.sRef = TUNE_TURN_SPEED;
    __GmWPID.sRef = TUNE_TURN_WPID_L;
    GW = 0;

    while (1)
    {
        W = (float)GW * TUNE_GYRO_GW_SCALE / TUNE_ADC_FULL_SCALE / 2;
        if (W > TUNE_TURN90_W_C_L)
            break;
    }

    __GmWPID.sRef = 0;
    __GucMouseState = __GOAHEAD;
    GuiSpeedCtr = TUNE_SPEED_HOLD;

    /* 原代码此处系数是 286.67（其余位置为 2886.67），先保留并标记 */
    __GmLeft.uiPulse = TUNE_TURN90_PULSE_C_L - piancha_l * TUNE_TURN_PULSE_PER_DEG_L;
    __GmLeft.uiPulseCtr = 0;
    __GmRight.uiPulse = TUNE_TURN90_PULSE_C_L - piancha_l * TUNE_TURN_PULSE_PER_DEG_L;
    __GmRight.uiPulseCtr = 0;

    while ((__GmRight.uiPulseCtr + TUNE_TURN_PULSE_MARGIN) <= __GmRight.uiPulse);
    while ((__GmLeft.uiPulseCtr + TUNE_TURN_PULSE_MARGIN) <= __GmLeft.uiPulse);

    __GucMouseState = __TURNLEFT;

    GuiSpeedCtr = __SPEEDUP;
    __GmRight.cState = __MOTORSTOP;
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
}

void mouseTurnright_Y(void)
{
    GW = 0;
    __GucMouseState = __TURNRIGHT;
    __GmRight.cState = __MOTORRUN;
    __GmLeft.cState  = __MOTORRUN;
    GucMouseDir = (GucMouseDir + 2) % 8;

    __GmSPID.sRef = 0;
    __GmWPID.sRef = -TUNE_TURN_WPID_Y;

    while (1)
    {
        W = (float)GW * TUNE_GYRO_GW_SCALE / TUNE_ADC_FULL_SCALE / 2;
        if (W > TUNE_TURN_Y_W_R)
            break;
    }

    __GmWPID.sRef = 0;
    __GmSPID.sRef = TUNE_TURN_Y_SPEED;
    __GucMouseState = __GOAHEAD;
    GuiSpeedCtr = TUNE_SPEED_HOLD;

    __GmLeft.uiPulse = TUNE_TURN_Y_PULSE_R;
    __GmLeft.uiPulseCtr = 0;
    __GmRight.uiPulse = TUNE_TURN_Y_PULSE_R;
    __GmRight.uiPulseCtr = 0;

    while ((__GmRight.uiPulseCtr + TUNE_TURN_PULSE_MARGIN) <= __GmRight.uiPulse);
    while ((__GmLeft.uiPulseCtr + TUNE_TURN_PULSE_MARGIN) <= __GmLeft.uiPulse);

    __GucMouseState = __TURNRIGHT;
    GuiSpeedCtr = __SPEEDUP;
    __GmRight.cState = __MOTORSTOP;
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
}

void mouseTurnleft_Y(void)
{
    __GucMouseState = __TURNLEFT;
    __GmRight.cState = __MOTORRUN;
    __GmLeft.cState  = __MOTORRUN;
    GucMouseDir = (GucMouseDir + 6) % 8;

    __GmWPID.sRef = TUNE_TURN_WPID_Y;
    GW = 0;

    while (1)
    {
        W = (float)GW * TUNE_GYRO_GW_SCALE / TUNE_ADC_FULL_SCALE / 2;
        if (W > TUNE_TURN_Y_W_L)
            break;
    }

    __GmWPID.sRef = 0;
    __GmSPID.sRef = TUNE_TURN_Y_SPEED;
    __GucMouseState = __GOAHEAD;
    GuiSpeedCtr = TUNE_SPEED_HOLD;

    __GmLeft.uiPulse = TUNE_TURN_Y_PULSE_L;
    __GmLeft.uiPulseCtr = 0;
    __GmRight.uiPulse = TUNE_TURN_Y_PULSE_L;
    __GmRight.uiPulseCtr = 0;

    while ((__GmRight.uiPulseCtr + TUNE_TURN_PULSE_MARGIN) <= __GmRight.uiPulse);
    while ((__GmLeft.uiPulseCtr + TUNE_TURN_PULSE_MARGIN) <= __GmLeft.uiPulse);

    __GucMouseState = __TURNLEFT;
    GuiSpeedCtr = __SPEEDUP;
    __GmRight.cState = __MOTORSTOP;
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
}

void onestep3(void)
{
    __GmRight.uiPulse = TUNE_ONESTEP3_PULSE;
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulse = TUNE_ONESTEP3_PULSE;
    __GmLeft.uiPulseCtr = 0;
    while ((__GmRight.uiPulseCtr + TUNE_TURN_PULSE_MARGIN) <= __GmRight.uiPulse);
    while ((__GmLeft.uiPulseCtr + TUNE_TURN_PULSE_MARGIN) <= __GmLeft.uiPulse);
}

void mouseTurnback(void)
{
    if (GucFrontNear)
    {
        __GmSPID.sRef = TUNE_TURNBACK_SPEED;
        while ((ucIRCheck[0] < GusDistance_FL_Near) ||
               (ucIRCheck[1] < GusDistance_FR_Near));

        __GmRight.uiPulse = TUNE_TURNBACK_APPROACH_PULSE;
        __GmRight.uiPulseCtr = 0;
        __GmLeft.uiPulse = TUNE_TURNBACK_APPROACH_PULSE;
        __GmLeft.uiPulseCtr = 0;
        while ((__GmRight.uiPulseCtr + TUNE_TURN_PULSE_MARGIN) <= __GmRight.uiPulse);
        while ((__GmLeft.uiPulseCtr + TUNE_TURN_PULSE_MARGIN) <= __GmLeft.uiPulse);
    }

    GucFrontNear = 0;
    __GmSPID.sRef = 0;
    GW = 0;
    __GucMouseState = __TURNBACK;
    __GmLeft.cState  = __MOTORRUN;
    __GmRight.cState = __MOTORRUN;
    GucMouseDir = (GucMouseDir + 4) % 8;
    __GmWPID.sRef = TUNE_TURNBACK_WPID;

    while (1)
    {
        if (GW > TUNE_TURNBACK_GW)
            break;
    }

    __GmWPID.sRef = 0;
    __delay(TUNE_TURNBACK_SETTLE_LOOPS);
    __GucMouseState = __GOAHEAD;

    GuiSpeedCtr = __SPEEDUP;
    __GmSPID.sRef = TUNE_TURNBACK_RETURN_SPEED;

    __GmRight.uiPulse = TUNE_TURNBACK_RETURN_PULSE;
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulse = TUNE_TURNBACK_RETURN_PULSE;
    __GmLeft.uiPulseCtr = 0;
    while ((__GmRight.uiPulseCtr + TUNE_TURN_PULSE_MARGIN) <= __GmRight.uiPulse);
    while ((__GmLeft.uiPulseCtr + TUNE_TURN_PULSE_MARGIN) <= __GmLeft.uiPulse);

    __GmRight.cState = __MOTORSTOP;
    __GmLeft.cState  = __MOTORSTOP;

    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = TUNE_TURNBACK_END_MARK;
    __GmLeft.uiPulseCtr  = TUNE_TURNBACK_END_MARK;
}

 
void mouseTurnbackqidian(void)
{
    if (GucFrontNear)
    {
        __GmSPID.sRef = TUNE_TURNBACK_SPEED_QIDIAN;
        while ((ucIRCheck[0] < GusDistance_FL_Near) ||
               (ucIRCheck[1] < GusDistance_FR_Near));

        __GmRight.uiPulse = TUNE_TURNBACK_APPROACH_PULSE;
        __GmRight.uiPulseCtr = 0;
        __GmLeft.uiPulse = TUNE_TURNBACK_APPROACH_PULSE;
        __GmLeft.uiPulseCtr = 0;
        while ((__GmRight.uiPulseCtr + TUNE_TURN_PULSE_MARGIN) <= __GmRight.uiPulse);
        while ((__GmLeft.uiPulseCtr + TUNE_TURN_PULSE_MARGIN) <= __GmLeft.uiPulse);
    }

    GucFrontNear = 0;
    __GmSPID.sRef = 0;
    mouseStop();
    GW = 0;
    __GucMouseState = __TURNBACK;
    __GmLeft.cState  = __MOTORRUN;
    __GmRight.cState = __MOTORRUN;
    GucMouseDir = (GucMouseDir + 2) % 4;           /* 原代码如此，保留 */
    __GmWPID.sRef = TUNE_TURNBACK_WPID;

    while (1)
    {
        if (GW > TUNE_TURNBACK_GW_QIDIAN)
            break;
    }

    __GmWPID.sRef = 0;
    __delay(TUNE_TURNBACK_SETTLE_LOOPS);
    __GucMouseState = __GOAHEAD;

    GuiSpeedCtr = __SPEEDUP;
    __GmSPID.sRef = TUNE_TURNBACK_RETURN_SPEED;

    __GmRight.uiPulse = TUNE_TURNBACK_RETURN_PULSE;
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulse = TUNE_TURNBACK_RETURN_PULSE;
    __GmLeft.uiPulseCtr = 0;
    while ((__GmRight.uiPulseCtr + TUNE_TURN_PULSE_MARGIN) <= __GmRight.uiPulse);
    while ((__GmLeft.uiPulseCtr + TUNE_TURN_PULSE_MARGIN) <= __GmLeft.uiPulse);

    __GmRight.cState = __MOTORSTOP;
    __GmLeft.cState  = __MOTORSTOP;

    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = TUNE_TURNBACK_END_MARK;
    __GmLeft.uiPulseCtr  = TUNE_TURNBACK_END_MARK;
}

void mouseTurnbackqi(void)
{
    if (GucFrontNear)
    {
        __GmSPID.sRef = 0;
        while ((ucIRCheck[0] < GusDistance_FL_Near) ||
               (ucIRCheck[1] < GusDistance_FR_Near));

        __GmRight.uiPulse = TUNE_TURNBACK_APPROACH_PULSE;
        __GmRight.uiPulseCtr = 0;
        __GmLeft.uiPulse = TUNE_TURNBACK_APPROACH_PULSE;
        __GmLeft.uiPulseCtr = 0;
        while ((__GmRight.uiPulseCtr + TUNE_TURN_PULSE_MARGIN) <= __GmRight.uiPulse);
        while ((__GmLeft.uiPulseCtr + TUNE_TURN_PULSE_MARGIN) <= __GmLeft.uiPulse);
    }

    GucFrontNear = 0;
    __GmSPID.sRef = 0;
    GW = 0;
    __GucMouseState = __TURNBACK;
    __GmLeft.cState  = __MOTORRUN;
    __GmRight.cState = __MOTORRUN;
    GucMouseDir = (GucMouseDir + 4) % 8;
    __GmWPID.sRef = TUNE_TURNBACK_WPID_QI;

    while (1)
    {
        if (GW > TUNE_TURNBACK_GW_QI)
            break;
    }

    __GmWPID.sRef = 0;
    mouseStop();
    __GucMouseState = __GOBACK;
    __GmSPID.sRef = TUNE_TURNBACK_BACK_SPEED;

    __GmRight.uiPulse = TUNE_TURNBACK_RETURN_PULSE_QI;
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulse = TUNE_TURNBACK_RETURN_PULSE_QI;
    __GmLeft.uiPulseCtr = 0;
    while ((__GmRight.uiPulseCtr + TUNE_TURN_PULSE_MARGIN) <= __GmRight.uiPulse);
    while ((__GmLeft.uiPulseCtr + TUNE_TURN_PULSE_MARGIN) <= __GmLeft.uiPulse);

    mouseStop();
    __GmRight.cState = __MOTORSTOP;
    __GmLeft.cState  = __MOTORSTOP;

    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr  = 0;
}

void mouseTurnback_H(void)
{
    __GmSPID.sRef = 0;
    mouseStop();
    GW = 0;
    __GucMouseState = __TURNBACK;

    __GmLeft.cState  = __MOTORRUN;
    __GmRight.cState = __MOTORRUN;
    GucMouseDir = (GucMouseDir + 4) % 8;
    __GmWPID.sRef = TUNE_TURNBACK_WPID_QI;

    while (1)
    {
        if (GW > TUNE_TURNBACK_GW)
            break;
    }

    __GmWPID.sRef = 0;
    __GmSPID.sRef = TUNE_TURNBACK_BACK_SPEED_H;

    __GmRight.uiPulse = TUNE_TURNBACK_BACK_PULSE_H;
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulse = TUNE_TURNBACK_BACK_PULSE_H;
    __GmLeft.uiPulseCtr = 0;
    while ((__GmRight.uiPulseCtr + TUNE_TURN_PULSE_MARGIN) <= __GmRight.uiPulse);
    while ((__GmLeft.uiPulseCtr + TUNE_TURN_PULSE_MARGIN) <= __GmLeft.uiPulse);

    __delay(TUNE_TURNBACK_SETTLE_LOOPS);

    __GmSPID.sRef = TUNE_TURNBACK_RETURN_SPEED_H;
    __GucMouseState = __GOAHEAD;
    __GmRight.uiPulse = TUNE_TURNBACK_RETURN_PULSE_H;
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulse = TUNE_TURNBACK_RETURN_PULSE_H;
    __GmLeft.uiPulseCtr = 0;
    while ((__GmRight.uiPulseCtr + TUNE_TURN_PULSE_MARGIN) <= __GmRight.uiPulse);
    while ((__GmLeft.uiPulseCtr + TUNE_TURN_PULSE_MARGIN) <= __GmLeft.uiPulse);

    __GmRight.cState = __MOTORSTOP;
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr  = 0;
}

void voltageDetect(void)
{
    static uint8 first = 1;
    static float gyro_0 = 0, w = 0, w1 = 0;
    float avg;

    if (first)
    {
        gyro_0 = zijiaozheng;
        Angle_TLY_Average = zijiaozheng;
        first = 0;
    }

    avg = (float)Angle_TLY_Average;
    w = ((avg - gyro_0) + w1) * TUNE_GYRO_TRAPEZOID
        * TUNE_GYRO_SENSITIVITY_MV * TUNE_ADC_VREF_MV
        / (TUNE_ADC_FULL_SCALE * TUNE_GYRO_SYSTEM_PARAM * TUNE_GYRO_ANGLE_RANGE);
    jiao += w * TUNE_CONTROL_DT_S;
    w1 = avg - gyro_0;
}

void voltageDetect1(void)
{
    u16 w;

    if (Angle_TLY_Average >= voltageDetectRef)
        w = Angle_TLY_Average - voltageDetectRef;
    else
        w = voltageDetectRef - Angle_TLY_Average;

    GW += w;
}

/*********************************************************************************************************
** Function name:       mazeSearch
** Descriptions:        前进N格
** input parameters:    iNblock: 前进的格数
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
static int16   GuiTpusle_LR_l                     = 0;
static int16   GuiTpusle_LR_r                      = 0; 
 
 
void mazeSearch(void)
{
    int8 cL = 0, cR = 0, cCoor = 1;

    if (__GmLeft.cState)
        cCoor = 0;

    if ((__GucMouseState == __TURNRIGHT) || (__GucMouseState == __TURNLEFT))
    {
        __GmLeft.uiPulseCtr  = TUNE_SEARCH_HANDOFF_PULSE;
        __GmRight.uiPulseCtr = TUNE_SEARCH_HANDOFF_PULSE;
        cL = 1;
        cR = 1;

        if (((__GucDistance[__FRONTR] != 0) && (__GucDistance[__FRONTL] != 0)) ||
            ((__GucDistance[__LEFT] & 0x01) == 0) ||
            ((__GucDistance[__RIGHT] & 0x01) == 0))
        {
            if ((__GucDistance[__FRONTR] != 0) && (__GucDistance[__FRONTL] != 0))
            {
                GuiTpusle_LR = TUNE_SEARCH_TURN_BLEND_BOTH;

                if ((__GucMouseState == __TURNRIGHT) &&
                    ((__GucDistance[__LEFT] & 0x01) == 0))
                {
                    GuiTpusle_LR_l = TUNE_SEARCH_TURN_CORR_L;
                    W_l = 0;
                }
                if ((__GucMouseState == __TURNRIGHT) &&
                    ((__GucDistance[__RIGHT] & 0x01) == 0))
                {
                    GuiTpusle_LR_r = 0;
                    W_r = 0;
                }
                if ((__GucMouseState == __TURNLEFT) &&
                    ((__GucDistance[__LEFT] & 0x01) == 0))
                {
                    GuiTpusle_LR_l = 0;
                    W_l = 0;
                }
                if ((__GucMouseState == __TURNLEFT) &&
                    ((__GucDistance[__RIGHT] & 0x01) == 0))
                {
                    GuiTpusle_LR_r = TUNE_SEARCH_TURN_CORR_R;
                    W_r = 0;
                }
            }
            else
            {
                GuiTpusle_LR = TUNE_SEARCH_TURN_BLEND_ONE;
            }
        }
        else
        {
            GuiTpusle_LR = 0;
        }
    }
    else
    {
        GuiTpusle_LR = 0;
    }

    __GucMouseState = __GOAHEAD;
    __GiMaxSpeed    = SEARCHSPEED;
    __GmRight.uiPulse = MAZETYPE * TUNE_PULSE_PER_BLOCK;
    __GmLeft.uiPulse  = MAZETYPE * TUNE_PULSE_PER_BLOCK;
    __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
    GuiSpeedCtr = __SPEEDUP;

    while (__GmLeft.cState != __MOTORSTOP)
    {
        if (__GmLeft.uiPulseCtr >= TUNE_PULSE_PER_BLOCK)
        {
            __GmLeft.uiPulse    -= TUNE_PULSE_PER_BLOCK;
            __GmLeft.uiPulseCtr -= TUNE_PULSE_PER_BLOCK;

            if (cCoor)
            {
                if (((__GucDistance[__FRONTR] != 0) && (__GucDistance[__FRONTL] != 0)) &&
                    (ucIRCheck[2] > GusDistance_L_Far) &&
                    (ucIRCheck[3] > GusDistance_R_Far))
                {
                    GucFrontNear = 1;
                    goto End;
                }
                __mouseCoorUpdate();
            }
            else
            {
                cCoor = 1;
            }
        }

        if (__GmRight.uiPulseCtr >= TUNE_PULSE_PER_BLOCK)
        {
            __GmRight.uiPulse    -= TUNE_PULSE_PER_BLOCK;
            __GmRight.uiPulseCtr -= TUNE_PULSE_PER_BLOCK;
        }

        if (cL)
        {
            if ((__GucDistance[__LEFT] & 0x01) == 0)
            {
                __GmRight.uiPulse = __GmRight.uiPulseCtr
                                    + TUNE_SEARCH_LEFT_ADVANCE - GuiTpusle_LR - GuiTpusle_LR_l;
                __GmLeft.uiPulse  = __GmLeft.uiPulseCtr
                                    + TUNE_SEARCH_LEFT_ADVANCE - GuiTpusle_LR - GuiTpusle_LR_l;

                while ((__GucDistance[__LEFT] & 0x01) == 0)
                {
                    if ((__GmLeft.uiPulseCtr + TUNE_SEARCH_SIDE_MARGIN) > __GmLeft.uiPulse)
                    {
                        GucFrontNear = 0;
                        goto End;
                    }
                }

                __GmRight.uiPulse = MAZETYPE * TUNE_PULSE_PER_BLOCK;
                __GmLeft.uiPulse  = MAZETYPE * TUNE_PULSE_PER_BLOCK;
                GuiSpeedCtr = __SPEEDUP;
            }
        }
        else
        {
            if (ucIRCheck[2] > GusDistance_L_Far)
                cL = 1;
        }

        if (cR)
        {
            if ((__GucDistance[__RIGHT] & 0x01) == 0)
            {
                __GmRight.uiPulse = __GmRight.uiPulseCtr
                                    + TUNE_SEARCH_RIGHT_ADVANCE - GuiTpusle_LR - GuiTpusle_LR_r;
                __GmLeft.uiPulse  = __GmLeft.uiPulseCtr
                                    + TUNE_SEARCH_RIGHT_ADVANCE - GuiTpusle_LR - GuiTpusle_LR_r;

                while ((__GucDistance[__RIGHT] & 0x01) == 0)
                {
                    if ((__GmLeft.uiPulseCtr + TUNE_SEARCH_SIDE_MARGIN) > __GmLeft.uiPulse)
                    {
                        GucFrontNear = 0;
                        goto End;
                    }
                }

                __GmRight.uiPulse = MAZETYPE * TUNE_PULSE_PER_BLOCK;
                __GmLeft.uiPulse  = MAZETYPE * TUNE_PULSE_PER_BLOCK;
                GuiSpeedCtr = __SPEEDUP;
            }
        }
        else
        {
            if (ucIRCheck[3] > GusDistance_R_Far)
                cR = 1;
        }
    }

End:
    __mouseCoorUpdate();
}

static void __mouseCoorUpdate(void)
{
    switch (GucMouseDir)
    {
    case 0: GmcMouse.cY++; break;
    case 2: GmcMouse.cX++; break;
    case 4: GmcMouse.cY--; break;
    case 6: GmcMouse.cX--; break;
    default: break;
    }

    __wallCheck();
    __mazeInfDebug();
}

static void __mazeInfDebug(void)
{
    switch (GucMouseDir)
    {
    case 0: displayDownload(2, 3, 0, 0x47); break;      /* F */
    case 2: displayDownload(2, 3, 0, 0x77); break;      /* R */
    case 4: displayDownload(2, 3, 0, 0x1f); break;      /* b */
    case 6: displayDownload(2, 3, 0, 0x0e); break;      /* L */
    default: displayDownload(2, 3, 0, 0x4f); break;     /* E */
    }

    displayDownload(1, 0, 0, GmcMouse.cX / 10);
    displayDownload(1, 1, 0, GmcMouse.cX % 10);
    displayDownload(1, 6, 0, GmcMouse.cY / 10);
    displayDownload(1, 7, 0, GmcMouse.cY % 10);
}

static void __wallCheck(void)
{
    uint8 ucMap = 0;
    uint8 uctemp = 0;

    ucMap |= MOUSEWAY_B;

    if (ucIRCheck[2] > GusDistance_L_Far)
    {
        ucMap &= ~MOUSEWAY_L;
        uctemp |= 0x06;
    }
    else
    {
        ucMap |= MOUSEWAY_L;
    }

    if ((ucIRCheck[0] > GusDistance_FL_Far) && (ucIRCheck[1] > GusDistance_FR_Far))
    {
        ucMap &= ~MOUSEWAY_F;
        uctemp |= 0x40;
    }
    else
    {
        ucMap |= MOUSEWAY_F;
    }

    if (ucIRCheck[3] > GusDistance_R_Far)
    {
        ucMap &= ~MOUSEWAY_R;
        uctemp |= 0x30;
    }
    else
    {
        ucMap |= MOUSEWAY_R;
    }

    GucMapBlock0[GmcMouse.cX][GmcMouse.cY] = ucMap;
    GucMapBlock[GmcMouse.cX][GmcMouse.cY]  = ucMap;
    GucMapBlock1[GmcMouse.cX][GmcMouse.cY] = ucMap;

    if (GmcMouse.cY < (MAZETYPE - 1))
        GucMapBlock1[GmcMouse.cX][GmcMouse.cY + 1] &= ~(((~ucMap) & 0x01) * 4);
    if (GmcMouse.cX < (MAZETYPE - 1))
        GucMapBlock1[GmcMouse.cX + 1][GmcMouse.cY] &= ~(((~ucMap) & 0x02) * 4);
    if (GmcMouse.cY > 0)
        GucMapBlock1[GmcMouse.cX][GmcMouse.cY - 1] &= ~(((~ucMap) & 0x04) / 4);
    if (GmcMouse.cX > 0)
        GucMapBlock1[GmcMouse.cX - 1][GmcMouse.cY] &= ~(((~ucMap) & 0x08) / 4);

    if (GmcMouse.cY < (MAZETYPE - 1))
        GucMapBlock[GmcMouse.cX][GmcMouse.cY + 1] |= ((ucMap & 0x01) * 4);
    if (GmcMouse.cX < (MAZETYPE - 1))
        GucMapBlock[GmcMouse.cX + 1][GmcMouse.cY] |= ((ucMap & 0x02) * 4);
    if (GmcMouse.cY > 0)
        GucMapBlock[GmcMouse.cX][GmcMouse.cY - 1] |= ((ucMap & 0x04) / 4);
    if (GmcMouse.cX > 0)
        GucMapBlock[GmcMouse.cX - 1][GmcMouse.cY] |= ((ucMap & 0x08) / 4);

    displayDownload(2, 2, 0, uctemp);
}

uint8 DenggaoCheck(void)
{
    /* 注意：当前硬件没有单独的“正前方”槽位，__GucDistance[__FRONT] 恒为 0，
       因此本函数目前恒返回 false。保留原逻辑，启用前需先填充该槽位。 */
    if ((__GucDistance[__FRONT]) &&
        (__GucDistance[__LEFT] & 0x01) &&
        (__GucDistance[__RIGHT] & 0x01))
        return true;

    return false;
}

uint8 PulseCtrCheck(void)
{
    if (__GmLeft.uiPulseCtr > (__GmLeft.uiPulse + TUNE_PULSE_OVERRUN_MARGIN))
        return true;

    return false;
}


/*======================================================================================================
  8 向运动原语
    - mouseTurnTo()      ：原地转到目标绝对方向（0..7）
    - mouseGoOneBlock()  ：沿当前方向走一格（正交/斜向）
  说明：这两级只负责“一格 + 一次转向”，路径由 path_8dir.c 规划。
======================================================================================================*/

void Motion_ResetPID(void)
{
    __GmSPID.sRef = 0;
    __GmSPID.sPreError = 0;
    __GmSPID.sPreDerror = 0;
    __GmSPID.iPreU = 0;

    __GmWPID.sRef = 0;
    __GmWPID.sPreError = 0;
    __GmWPID.sPreDerror = 0;
    __GmWPID.iPreU = 0;
}

void Motion_Stop(void)
{
    __GmSPID.sRef = 0;
    __GmWPID.sRef = 0;
    GuiSpeedCtr = TUNE_SPEED_STOP;

    __GmRight.cState = __MOTORSTOP;
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __GmLeft.sSpeed  = 0;
    __rightMotorContr();
    __leftMotorContr();

    __GmRight.uiPulse = 0;
    __GmLeft.uiPulse  = 0;
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr  = 0;
}

/* steps45: 正数右转，负数左转，单位 45 度 */
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
