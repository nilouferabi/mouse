#include "Mouse_Drive.h"
#include "math.h"
/*********************************************************************************************************
  定义全局变量
*********************************************************************************************************/
MAZECOOR          GmcMouse                        = {0,0}; //一个MAZECOOR结构的变量 /*  保存电脑鼠当前位置坐标      */
uint8             GucMouseDir                     = UP;                 /*  保存电脑鼠当前方向          */
uint8             GucMapBlock[MAZETYPE][MAZETYPE] = {0};                /*  GucMapBlock[x][y]           */
uint8             GucMapBlock0[MAZETYPE][MAZETYPE] = {0};               /*  中心算法         */
                                                                      /*  0:该方向无路，1:该方向有路  */
uint8             GucMapBlock1[MAZETYPE][MAZETYPE]= {0x0f};
uint8             GucMouseStart                    = 0;                 /* 电脑鼠启动        */
uint8             GucFrontJinju                    = 0;                 /* 前方红外近距在搜索时等高图制作中用   */
uint8             GucCrossroad                     = 0;                 /* 十字路口数，冲刺时用，若十字路口多降低最高冲刺速度   */
static uint32     GW;         //1）变量不释放；2）所修饰只能在本文件使用                                              /*小车转动角度*/
static float      piancha_r;
static __MOTOR  __GmLeft                          = {0, 0, 0, 0, 0, 0};    /*  定义并初始化左电机状态      */
static __MOTOR  __GmRight                         = {0, 0, 0, 0, 0, 0};    /*  定义并初始化右电机状态      */
static __PID    __GmLPID;                                                 /*  定义左电机PID      */
static __PID    __GmRPID;                                                 /*  定义右电机PID     */
 __PID    __GmSPID;      //  straight                                /*  直线PID     */
 __PID    __GmWPID;      //  wheel v.转动、盘旋                      /*  旋转PID     */
static uint8    __GucMouseState                   = __STOP;             /*  保存电脑鼠当前运行状态      */
static int32    __GiMaxSpeed                      = SEARCHSPEED;        /*  保存允许运行的最大速度      */
static int32    __GiMinSpeed                      = SEARCHSPEED;
static uint8    __GucDistance[5]                  = {0};                /*  记录传感器状态              */
uint16   GusFreq_F                         = 36200;   //33.8,33,327        /*  前方红外频率              */
uint16   GusFreq_FJ                        = 19200;   //26.3,266,275              /*  前方近距红外频率              */
uint16   GusFreq_X                         = 30000;   //35,33.8          /*  斜45度红外频率              */
uint16   GusFreq_LF                        = 31700;   //34000           /*  左右红外远距频率              */
uint16   GusFreq_L                         = 18300;              /*  左右红外近距频率              */
static  float   GsTpusle_T                       = 0;    //int16   /*  左轮校正减少的速度值              */
static uint8    GuiSpeedCtr                       = 0;
static uint16   GuiTpusle_LR                      = 0;
static uint16   GuiTpusle_S                       = 0;
static uint8    GucFrontNear                      = 0;
static float jiao = 0;
uint8      map                             = 0;
uint8    GucGoHead                     = 0;
  uint8    GucGoHead1                     = 0;
   uint8    GucGoHead2                     = 0;
uint16    GucFangXiang                      = 0;
uint8    GucDirTemp                        = 0;
uint16    DIS[10] = {0};
uint8 Tab=4;//红外调试选项
uint8  Conturns                               =0;    
uint16 count=0;
uint8      m                             = 0;
uint8      backflag                        =0;
uint8 GoHead45_flag=0;
extern uint8 turn45_flag;
extern u16 voltageDetectRef;
extern __IO uint16_t ADC_ConvertedValue[5];
extern float ADC_ConvertedValueLocal[4]; 
float W;
uint16 Sunlight[4];
uint16 ucIRCheck[4];
extern uint8 usepiancha;
static float W_l;                       //变量作用？？？
static float W_r;
u32 Angle_TLY[8] = {0}; 
u32 Angle_TLY_All = 0;
u32 Angle_TLY_Average = 0;
u16 voltageDetectRef=2041;
float W;
float W_R;

uint16 GusDistance_L_Near=1951;//1412; 2750//左红外近 1250
uint16 GusDistance_L_Mid=1200;  //左红外中              750
uint16 GusDistance_L_Far=450;

uint16 GusDistance_R_Near=1693;//2135; //右红外近      1900
uint16 GusDistance_R_Mid=1000;  //右红外中start_flag   1150
uint16 GusDistance_R_Far=302;

uint16 GusDistance_FL_Near =3076;//1544; 917 //近距，用来判断停止 2500
uint16 GusDistance_FR_Near =2835;

uint16 GusDistance_FL_Far =386;//237;   //远距用来判断墙壁 //380
uint16 GusDistance_FR_Far =484;//611;//345

static uint8 after_turnleft;
static float piancha_l;

 uint16 DisFrX = 500;
uint16 DisFlX = 500;
uint16 DisRX = 200;
uint16 DisLX = 200;

float piancha_vl;
float piancha_vr;


float speed_r[278]={0.00,0.09,0.18,0.26,0.35,0.44,0.53,0.62,0.71,0.79,0.88,0.97,1.06,1.15,1.24,1.32,1.41,1.50,1.59,1.68,1.76,1.85,1.94,2.03,2.12,2.21,2.29,2.38,2.47,2.56,2.65,2.74,2.82,2.91,3.00,3.09,3.18,3.26,3.35,3.44,3.53,3.62,3.71,3.79,3.88,3.97,4.06,4.15,4.24,4.32,4.41,4.50,4.59,4.68,4.76,4.85,4.94,5.03,5.12,5.21,5.29,5.38,5.47,5.56,5.65,5.74,5.82,5.91,6.00,6.09,6.18,6.26,6.35,6.44,6.53,6.62,6.71,6.79,6.88,6.97,7.06,7.15,7.24,7.32,7.41,7.50,7.59,7.68,7.76,7.85,7.94,8.03,8.12,8.21,8.29,8.38,8.47,8.56,8.65,8.74,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.74,8.65,8.56,8.47,8.38,8.29,8.21,8.12,8.03,7.94,7.85,7.76,7.68,7.59,7.50,7.41,7.32,7.24,7.15,7.06,6.97,6.88,6.79,6.71,6.62,6.53,6.44,6.35,6.26,6.18,6.09,6.00,5.91,5.82,5.74,5.65,5.56,5.47,5.38,5.29,5.21,5.12,5.03,4.94,4.85,4.76,4.68,4.59,4.50,4.41,4.32,4.24,4.15,4.06,3.97,3.88,3.79,3.71,3.62,3.53,3.44,3.35,3.26,3.18,3.09,3.00,2.91,2.82,2.74,2.65,2.56,2.47,2.38,2.29,2.21,2.12,2.03,1.94,1.85,1.76,1.68,1.59,1.50,1.41,1.32,1.24,1.15,1.06,0.97,0.88,0.79,0.71,0.62,0.53,0.44,0.35,0.26,0.18,0.09,0.00};
float speed_l[278]={0.00,0.09,0.18,0.26,0.35,0.44,0.53,0.62,0.71,0.79,0.88,0.97,1.06,1.15,1.24,1.32,1.41,1.50,1.59,1.68,1.76,1.85,1.94,2.03,2.12,2.21,2.29,2.38,2.47,2.56,2.65,2.74,2.82,2.91,3.00,3.09,3.18,3.26,3.35,3.44,3.53,3.62,3.71,3.79,3.88,3.97,4.06,4.15,4.24,4.32,4.41,4.50,4.59,4.68,4.76,4.85,4.94,5.03,5.12,5.21,5.29,5.38,5.47,5.56,5.65,5.74,5.82,5.91,6.00,6.09,6.18,6.26,6.35,6.44,6.53,6.62,6.71,6.79,6.88,6.97,7.06,7.15,7.24,7.32,7.41,7.50,7.59,7.68,7.76,7.85,7.94,8.03,8.12,8.21,8.29,8.38,8.47,8.56,8.65,8.74,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.82,8.74,8.65,8.56,8.47,8.38,8.29,8.21,8.12,8.03,7.94,7.85,7.76,7.68,7.59,7.50,7.41,7.32,7.24,7.15,7.06,6.97,6.88,6.79,6.71,6.62,6.53,6.44,6.35,6.26,6.18,6.09,6.00,5.91,5.82,5.74,5.65,5.56,5.47,5.38,5.29,5.21,5.12,5.03,4.94,4.85,4.76,4.68,4.59,4.50,4.41,4.32,4.24,4.15,4.06,3.97,3.88,3.79,3.71,3.62,3.53,3.44,3.35,3.26,3.18,3.09,3.00,2.91,2.82,2.74,2.65,2.56,2.47,2.38,2.29,2.21,2.12,2.03,1.94,1.85,1.76,1.68,1.59,1.50,1.41,1.32,1.24,1.15,1.06,0.97,0.88,0.79,0.71,0.62,0.53,0.44,0.35,0.26,0.18,0.09,0.00};
float gyro_r[250]={0.00,0.00,0.01,0.02,0.03,0.05,0.08,0.11,0.14,0.18,0.23,0.28,0.34,0.40,0.47,0.54,0.62,0.70,0.79,0.88,0.97,1.08,1.19,1.30,1.42,1.54,1.67,1.80,1.94,2.08,2.23,2.39,2.54,2.71,2.88,3.05,3.23,3.42,3.61,3.80,4.00,4.21,4.42,4.63,4.85,5.08,5.31,5.55,5.79,6.03,6.29,6.54,6.80,7.07,7.34,7.62,7.90,8.19,8.48,8.78,9.08,9.39,9.70,10.02,10.34,10.67,11.01,11.34,11.69,12.04,12.39,12.75,13.11,13.48,13.86,14.24,14.62,15.01,15.41,15.81,16.21,16.62,17.04,17.46,17.89,18.32,18.75,19.19,19.64,20.09,20.55,21.01,21.48,21.95,22.43,22.91,23.40,23.89,24.39,24.89,25.40,25.91,26.42,26.94,27.45,27.96,28.48,28.99,29.50,30.02,30.53,31.04,31.56,32.07,32.58,33.09,33.61,34.12,34.63,35.15,35.66,36.17,36.69,37.20,37.71,38.23,38.74,39.25,39.76,40.28,40.79,41.30,41.82,42.33,42.84,43.36,43.87,44.38,44.90,45.41,45.92,46.44,46.95,47.46,47.97,48.49,49.00,49.51,50.03,50.54,51.05,51.57,52.08,52.59,53.11,53.62,54.13,54.64,55.16,55.67,56.18,56.70,57.21,57.72,58.24,58.75,59.26,59.78,60.29,60.80,61.32,61.83,62.34,62.85,63.37,63.88,64.39,64.91,65.41,65.92,66.42,66.91,67.40,67.88,68.35,68.83,69.29,69.76,70.21,70.66,71.11,71.55,71.99,72.42,72.84,73.27,73.68,74.09,74.50,74.90,75.29,75.68,76.07,76.45,76.82,77.19,77.55,77.91,78.27,78.62,78.96,79.30,79.63,79.96,80.28,80.60,80.92,81.22,81.53,81.82,82.12,82.40,82.69,82.96,83.23,83.50,83.76,84.02,84.27,84.52,84.76,84.99,85.23,85.45,85.67,85.89,86.10,86.4,86.8,87.3,87.8,88.0,88.4,88.7,89.1,89.6,90};
float gyro_l[250]={0.00,0.00,0.01,0.02,0.03,0.05,0.08,0.11,0.14,0.18,0.23,0.28,0.34,0.40,0.47,0.54,0.62,0.70,0.79,0.88,0.97,1.08,1.19,1.30,1.42,1.54,1.67,1.80,1.94,2.08,2.23,2.39,2.54,2.71,2.88,3.05,3.23,3.42,3.61,3.80,4.00,4.21,4.42,4.63,4.85,5.08,5.31,5.55,5.79,6.03,6.29,6.54,6.80,7.07,7.34,7.62,7.90,8.19,8.48,8.78,9.08,9.39,9.70,10.02,10.34,10.67,11.01,11.34,11.69,12.04,12.39,12.75,13.11,13.48,13.86,14.24,14.62,15.01,15.41,15.81,16.21,16.62,17.04,17.46,17.89,18.32,18.75,19.19,19.64,20.09,20.55,21.01,21.48,21.95,22.43,22.91,23.40,23.89,24.39,24.89,25.40,25.91,26.42,26.94,27.45,27.96,28.48,28.99,29.50,30.02,30.53,31.04,31.56,32.07,32.58,33.09,33.61,34.12,34.63,35.15,35.66,36.17,36.69,37.20,37.71,38.23,38.74,39.25,39.76,40.28,40.79,41.30,41.82,42.33,42.84,43.36,43.87,44.38,44.90,45.41,45.92,46.44,46.95,47.46,47.97,48.49,49.00,49.51,50.03,50.54,51.05,51.57,52.08,52.59,53.11,53.62,54.13,54.64,55.16,55.67,56.18,56.70,57.21,57.72,58.24,58.75,59.26,59.78,60.29,60.80,61.32,61.83,62.34,62.85,63.37,63.88,64.39,64.91,65.41,65.92,66.42,66.91,67.40,67.88,68.35,68.83,69.29,69.76,70.21,70.66,71.11,71.55,71.99,72.42,72.84,73.27,73.68,74.09,74.50,74.90,75.29,75.68,76.07,76.45,76.82,77.19,77.55,77.91,78.27,78.62,78.96,79.30,79.63,79.96,80.28,80.60,80.92,81.22,81.53,81.82,82.12,82.40,82.69,82.96,83.23,83.50,83.76,84.02,84.27,84.52,84.76,84.99,85.23,85.45,85.67,85.89,86.10,86.4,86.8,87.3,87.8,88.0,88.4,88.7,89.1,89.6,90};
//左转jiao小于零
float disCheck3[201]={222.597,215.542,209.024,202.979,197.356,192.109,187.199,182.593,178.262,174.18,170.325,166.678,163.221,159.939,156.818,153.846,151.012,148.305,145.717,143.24,140.866,138.588,136.401,134.299,132.276,130.328,128.451,126.641,124.893,123.204,121.572,119.993,118.465,116.985,115.55,114.159,112.809,111.499,110.226,108.989,107.787,106.617,105.479,104.371,103.291,102.239,101.214,100.214,99.2392,98.2874,97.3583,96.4509,95.5646,94.6984,93.8517,93.0237,92.214,91.4217,90.6463,89.8872,89.1439,88.4159,87.7027,87.0037,86.3186,85.6469,84.9882,84.3421,83.7082,83.0861,82.4756,81.8762,81.2876,80.7096,80.1417,79.5839,79.0357,78.4969,77.9672,77.4464,76.9343,76.4306,75.9352,75.4477,74.9681,74.496,74.0313,73.5739,73.1235,72.6799,72.2431,71.8128,71.3889,70.9712,70.5597,70.154,69.7542,69.3601,68.9715,68.5884,68.2106,67.8379,67.4703,67.1077,66.75,66.397,66.0487,65.7049,65.3656,65.0307,64.7001,64.3736,64.0513,63.733,63.4186,63.1081,62.8014,62.4984,62.1991,61.9034,61.6111,61.3223,61.0369,60.7548,60.476,60.2004,59.9278,59.6584,59.392,59.1286,58.8681,58.6105,58.3556,58.1036,57.8543,57.6076,57.3636,57.1221,56.8832,56.6468,56.4128,56.1813,55.9521,55.7253,55.5008,55.2785,55.0584,54.8406,54.6249,54.4113,54.1998,53.9904,53.783,53.5775,53.3741,53.1726,52.9729,52.7752,52.5793,52.3852,52.1929,52.0024,51.8136,51.6265,51.4411,51.2574,51.0753,50.8949,50.716,50.5387,50.363,50.1887,50.016,49.8448,49.6751,49.5068,49.3399,49.1744,49.0103,48.8476,48.6863,48.5262,48.3675,48.2101,48.054,47.8991,47.7455,47.5931,47.4419,47.292,47.1432,46.9956,46.8491,46.7038,46.5597,46.4166,46.2746,46.1337,45.9939,45.8552,45.7175};
float disCheck2[201]={189.038,184.056,179.427,175.114,171.081,167.3,163.746,160.398,157.236,154.244,151.408,148.715,146.152,143.711,141.381,139.155,137.026,134.986,133.029,131.151,129.346,127.609,125.937,124.325,122.77,121.269,119.819,118.416,117.059,115.745,114.472,113.238,112.041,110.878,109.75,108.653,107.586,106.549,105.539,104.556,103.598,102.665,101.755,100.868,100.002,99.1569,98.3316,97.5254,96.7376,95.9676,95.2146,94.4781,93.7575,93.0522,92.3617,91.6855,91.0231,90.374,89.7379,89.1142,88.5027,87.9029,87.3144,86.7369,86.1701,85.6136,85.0672,84.5305,84.0033,83.4853,82.9762,82.4758,81.9838,81.5,81.0242,80.5562,80.0957,79.6426,79.1967,78.7578,78.3256,77.9001,77.4811,77.0684,76.6618,76.2613,75.8666,75.4776,75.0942,74.7163,74.3436,73.9762,73.6139,73.2566,72.9041,72.5564,72.2133,71.8747,71.5406,71.2109,70.8854,70.5641,70.2469,69.9336,69.6243,69.3189,69.0171,68.7191,68.4247,68.1338,67.8464,67.5624,67.2817,67.0043,66.7301,66.459,66.191,65.9261,65.6642,65.4051,65.1489,64.8956,64.645,64.3971,64.1519,63.9094,63.6693,63.4319,63.1969,62.9643,62.7342,62.5064,62.281,62.0578,61.8368,61.6181,61.4016,61.1872,60.9749,60.7646,60.5564,60.3502,60.146,59.9437,59.7433,59.5449,59.3482,59.1534,58.9604,58.7691,58.5796,58.3919,58.2058,58.0213,57.8385,57.6574,57.4778,57.2998,57.1234,56.9484,56.775,56.6031,56.4326,56.2636,56.096,55.9298,55.765,55.6015,55.4394,55.2786,55.1192,54.961,54.8041,54.6485,54.4941,54.3409,54.189,54.0382,53.8886,53.7402,53.593,53.4468,53.3018,53.1579,53.0151,52.8734,52.7327,52.5931,52.4545,52.317,52.1804,52.0449,51.9104,51.7768,51.6442,51.5126,51.3819,51.2521,51.1233,50.9954,50.8683};
float disCheck0[201]={440.174,422.152,405.657,390.497,376.513,363.572,351.557,340.37,329.928,320.155,310.989,302.374,294.259,286.602,279.364,272.511,266.012,259.84,253.97,248.38,243.05,237.962,233.099,228.446,223.99,219.718,215.619,211.683,207.898,204.258,200.752,197.375,194.118,190.976,187.941,185.01,182.175,179.433,176.779,174.209,171.718,169.303,166.961,164.687,162.48,160.336,158.252,156.226,154.256,152.338,150.472,148.654,146.883,145.157,143.475,141.834,140.234,138.672,137.147,135.658,134.204,132.783,131.394,130.037,128.709,127.411,126.14,124.897,123.68,122.488,121.321,120.178,119.058,117.96,116.884,115.829,114.795,113.78,112.784,111.807,110.849,109.908,108.984,108.077,107.186,106.31,105.45,104.605,103.775,102.959,102.156,101.367,100.591,99.8279,99.0771,98.3385,97.6117,96.8964,96.1924,95.4993,94.8171,94.1452,93.4836,92.8321,92.1902,91.5579,90.935,90.3211,89.7162,89.1199,88.5322,87.9529,87.3816,86.8184,86.263,85.7152,85.1749,84.6419,84.1161,83.5973,83.0854,82.5803,82.0817,81.5897,81.104,80.6245,80.1511,79.6837,79.2221,78.7663,78.3162,77.8716,77.4324,76.9986,76.57,76.1465,75.7281,75.3146,74.906,74.5022,74.103,73.7085,73.3185,72.9329,72.5517,72.1748,71.8021,71.4335,71.069,70.7086,70.352,69.9994,69.6505,69.3054,68.964,68.6262,68.292,67.9613,67.634,67.3101,66.9896,66.6723,66.3583,66.0475,65.7398,65.4352,65.1337,64.8351,64.5395,64.2468,63.957,63.67,63.3857,63.1042,62.8254,62.5493,62.2757,62.0048,61.7364,61.4705,61.207,60.946,60.6874,60.4312,60.1773,59.9256,59.6763,59.4292,59.1843,58.9416,58.701,58.4626,58.2262,57.9919,57.7596,57.5294,57.3011,57.0748,56.8504,56.6279,56.4073};
float disCheck1[201]={349.867,336.341,323.932,312.503,301.938,292.14,283.026,274.525,266.575,259.122,252.119,245.527,239.308,233.43,227.867,222.591,217.581,212.817,208.28,203.953,199.823,195.875,192.098,188.48,185.011,181.681,178.483,175.408,172.449,169.599,166.853,164.204,161.648,159.179,156.792,154.485,152.252,150.089,147.995,145.965,143.996,142.085,140.23,138.429,136.678,134.977,133.322,131.711,130.144,128.618,127.131,125.682,124.269,122.892,121.548,120.237,118.957,117.707,116.486,115.293,114.127,112.987,111.872,110.782,109.715,108.671,107.648,106.647,105.667,104.707,103.765,102.843,101.939,101.052,100.182,99.3294,98.4925,97.6711,96.865,96.0736,95.2965,94.5333,93.7836,93.0472,92.3235,91.6123,90.9133,90.2261,89.5504,88.8859,88.2324,87.5895,86.957,86.3346,85.7221,85.1193,84.5258,83.9415,83.3662,82.7997,82.2416,81.692,81.1504,80.6169,80.0911,79.5729,79.0622,78.5588,78.0624,77.5731,77.0905,76.6146,76.1453,75.6823,75.2256,74.775,74.3304,73.8916,73.4586,73.0313,72.6094,72.193,71.7819,71.376,70.9752,70.5794,70.1885,69.8024,69.421,69.0443,68.6721,68.3044,67.941,67.5819,67.2271,66.8764,66.5298,66.1871,65.8484,65.5135,65.1825,64.8551,64.5314,64.2113,63.8947,63.5816,63.2719,62.9656,62.6626,62.3628,62.0662,61.7727,61.4823,61.195,60.9106,60.6292,60.3507,60.075,59.8021,59.532,59.2646,58.9999,58.7378,58.4783,58.2213,57.9669,57.7149,57.4654,57.2182,56.9735,56.731,56.4909,56.253,56.0174,55.7839,55.5526,55.3235,55.0964,54.8715,54.6485,54.4276,54.2087,53.9918,53.7767,53.5636,53.3524,53.143,52.9354,52.7297,52.5257,52.3235,52.123,51.9242,51.7272,51.5318,51.338,51.1458,50.9553,50.7663,50.5789,50.3931,};
  
 
uint8 GoHead45_fx=0;

 

u8 flag=0;
u8 ir_flag=0;
u8 mm=0;

u8 backflag2=0;
u16 time;

uint8 flag_left                            =0;
uint8 flag_right                           =0; 
extern uint8 zongdianx;
extern uint8 zongdiany;
uint16 after=0;
extern int action;
extern uint8 STRAIGHT90L;
extern uint8 LV45L;
extern uint8 LT45L;
extern uint8 LV90L;
extern uint8 LV135L;
extern uint8 RV45L;
extern uint8 RT45L;
extern uint8 RV90L;
extern uint8 RV135L;
extern uint8 RT135L;
extern uint8 STRAIGHT45L;
extern uint8 L180L;
extern uint8 R180L;
extern uint8 RT90L;
extern uint8 LT90L;
extern uint8 LT135L;
static int16 angle_temp;   //转弯角度校正基准
uint8 after_turn=0;

extern unsigned char actionsequence[500];

float ADCmv(int AD)
{
  float mvdianya;
  mvdianya=AD*3300/4096;
}

/*脉冲测试函数*/
void testEncoder(void)
{
    __GmLeft.uiPulseCtr =0;   
    __GmRight.uiPulseCtr =0;

 __GucMouseState   = __GOAHEAD;
    __GiMaxSpeed      =   SEARCHSPEED;       
    __GmRight.uiPulse =   10 * ONEBLOCK;
    __GmLeft.uiPulse  =   10 * ONEBLOCK;
    __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
     GuiSpeedCtr=__SPEEDUP;
     
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   
   
 mouseStop();
    while(1);



}
/*********************************************************************************************************
** Function name:       __delay
** Descriptions:        延时函数
** input parameters:    uiD :延时参数，值越大，延时越久
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __delay (uint32  uiD)
{
    for (; uiD; uiD--);
}

void Delay_us(u32 Nus)   
{    
	 SysTick->LOAD=Nus*9;          		
	 SysTick->CTRL|=0x01;              
	 while(!(SysTick->CTRL&(1<<16)));     
	 SysTick->CTRL=0X00000000;        
	 SysTick->VAL=0X00000000;               
}
/*********************************************************************************************************
** Function name:       PIDInit
** Descriptions:        PID初始化
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void PIDInit(void) 
{  
    __GmLPID.usEncoder_new = 32768;
    //__GmLPID.usEncoder_last = 65535;
    __GmLPID.usFeedBack = 0 ;  //速度反馈值
    __GmLPID.sFeedBack = 0 ;
    
    __GmRPID.usEncoder_new = 32768;
    //__GmRPID.usEncoder_last = 65535;
    __GmRPID.usFeedBack = 0 ;  //速度反馈值
    __GmRPID.sFeedBack = 0 ;
    
    __GmSPID.sRef = 0 ;        //速度设定值 
    __GmSPID.sFeedBack = 0 ;        
    __GmSPID.sPreError = 0 ;   //前一次，速度误差,,vi_Ref - vi_FeedBack 
    __GmSPID.sPreDerror = 0 ;   //前一次，速度误差之差，d_error-PreDerror; 
        
    __GmSPID.fKp = __KP; 
    __GmSPID.fKi = __KI;
    __GmSPID.fKd = __KD; 
       
    __GmSPID.iPreU = 0 ;      //电机控制输出值 
    
    __GmWPID.sRef = 0 ;        //速度设定值 
    __GmWPID.sFeedBack = 0 ;       
    __GmWPID.sPreError = 0 ;   //前一次，速度误差,,vi_Ref - vi_FeedBack 
    __GmWPID.sPreDerror = 0 ;   //前一次，速度误差之差，d_error-PreDerror; 
    
    __GmWPID.fKp = __KP;  //30
    __GmWPID.fKi = __KI;  //0.1,0.01
    __GmWPID.fKd = __KD; 
       
    __GmWPID.iPreU = 0 ;      //电机控制输出值 
    
}

/*********************************************************************************************************
** Function name:       __SPIDContr
** Descriptions:        直线PID控制
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __SPIDContr(void) 
{ 
    float  error,d_error,dd_error;
    static uint8   K_I=1;
    error = __GmSPID.sRef - __GmSPID.sFeedBack; // 偏差计算
    d_error = error - __GmSPID.sPreError; 
    dd_error = d_error - __GmSPID.sPreDerror;
    if(error> Deadband)
      error -= Deadband;
    else if(error < -Deadband)
      error += Deadband;
    else
      error = 0;
    if((error > error_IMAX)||(error < -error_IMAX))
      K_I=0;
    else
      K_I=1;
    
    __GmSPID.sPreError = error; //存储当前偏差 
    __GmSPID.sPreDerror = d_error;
    
    __GmSPID.iPreU += (int16)(  __GmSPID.fKp * d_error + K_I*__GmSPID.fKi * error  + __GmSPID.fKd*dd_error); 
}
/*********************************************************************************************************
** Function name:       __WPIDContr
** Descriptions:        旋转方向PID控制
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __WPIDContr(void) 
{ 
    float  error,d_error,dd_error; 
    static uint8   K_I=1;
    error = __GmWPID.sRef + 0.5*GsTpusle_T -  __GmWPID.sFeedBack; // 偏差计算
    d_error = error - __GmWPID.sPreError; 
    dd_error = d_error - __GmWPID.sPreDerror;
    if(error> Deadband)
      error -= Deadband;
    else if(error < -Deadband)
      error += Deadband;
    else
      error = 0;
    if((error > error_IMAX)||(error < -error_IMAX))
      K_I=0;
    else
      K_I=1;
    
    __GmWPID.sPreError = error; //存储当前偏差 
    __GmWPID.sPreDerror = d_error;
    __GmWPID.iPreU += (int16)(  __GmWPID.fKp * d_error + K_I*__GmWPID.fKi * error  + __GmWPID.fKd*dd_error);
        
}

/*********************************************************************************************************
** Function name:      __PIDContr
** Descriptions:        PID控制，通过脉冲数控制电机
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __PIDContr(void)
{
    __SPIDContr();
    __WPIDContr();
    __GmLeft.sSpeed = __GmSPID.iPreU - __GmWPID.iPreU ;
    if(__GmLeft.sSpeed>=0){
     __GmLeft.cDir=__MOTORGOAHEAD; 
    if( __GmLeft.sSpeed >= U_MAX )   //速度PID，防止调节最高溢出 
       __GmLeft.sSpeed = U_MAX;      
    if( __GmLeft.sSpeed <= U_MIN ) //速度PID，防止调节最低溢出  
       __GmLeft.sSpeed = U_MIN;
    }
    else{
      __GmLeft.cDir=__MOTORGOBACK;
      __GmLeft.sSpeed *=-1; 
    if( __GmLeft.sSpeed >= U_MAX )   //速度PID，防止调节最高溢出 
       __GmLeft.sSpeed = U_MAX;      
    if( __GmLeft.sSpeed <= U_MIN ) //速度PID，防止调节最低溢出  
       __GmLeft.sSpeed = U_MIN;
    }
      
    __GmRight.sSpeed = __GmSPID.iPreU + __GmWPID.iPreU ;
    if(__GmRight.sSpeed>=0){
     __GmRight.cDir=__MOTORGOAHEAD; 
    if( __GmRight.sSpeed >= U_MAX )   //速度PID，防止调节最高溢出 
       __GmRight.sSpeed = U_MAX;      
    if( __GmRight.sSpeed <= U_MIN ) //速度PID，防止调节最低溢出  
       __GmRight.sSpeed = U_MIN;
    }
    else{
      __GmRight.cDir=__MOTORGOBACK;
      __GmRight.sSpeed *=-1; 
    if( __GmRight.sSpeed >= U_MAX )   //速度PID，防止调节最高溢出 
       __GmRight.sSpeed = U_MAX;      
    if( __GmRight.sSpeed <= U_MIN ) //速度PID，防止调节最低溢出  
       __GmRight.sSpeed = U_MIN;
    }
    __rightMotorContr();
    __leftMotorContr();
    
}
/*********************************************************************************************************
** Function name:       __Encoder
** Descriptions:        采集编码器输出的脉冲
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/



float _jueduizhi(float x)
{
 if(x<0)x=-x;
 return x;
}

uint8 start_firstge=1;
uint8 start_A=0,start_B,start_C,start_D=0;
uint8 quzabo=0;

uint8 start_speedr=0;
uint8 start_speedl=0;
uint8 start_LT45V=0;
uint8 start_RT45V=0;
uint8 start_LT135L=0;
uint8 start_RT135L=0;
uint8 start_LT90V=0;
uint8 start_RT90V=0;
uint8 start_LT180=0;
uint8 start_RT180=0;




float zhixianjuli=0;
float bianmaqijiaodu=0;
uint8 zanshi=0;
static uint8 speedcun[256];
static u16 codernumber=0;
void __Encoder(void)
{   
  uint8 cun_flag=0;
  
    static u16 Dir_L;
    static u16 Dir_R;
	
   
    __GmLPID.usEncoder_new = TIM_GetCounter(TIM2);
    __GmRPID.usEncoder_new = TIM_GetCounter(TIM3);
	
    Dir_R=TIM3->CR1;
    Dir_R=(Dir_R&0x0010)>>4;	
	
    Dir_L=TIM2->CR1;
    Dir_L=(Dir_L&0x0010)>>4;	
		
	if(Dir_L==1)//向下计数  向后退
	{
             __GmLPID.usFeedBack = 32768 - __GmLPID.usEncoder_new;
            TIM_SetCounter(TIM2, 32768);
             __GmLeft.uiPulseCtr += __GmLPID.usFeedBack;	
	     __GmLeft.cRealDir = __MOTORGOBACK;
             __GmLPID.sFeedBack= -1*__GmLPID.usFeedBack;		
	}
	else
	{   
            __GmLPID.usFeedBack = __GmLPID.usEncoder_new - 32768;
            TIM_SetCounter(TIM2, 32768);
           codernumber+=__GmLPID.usFeedBack;
            __GmLeft.uiPulseCtr += __GmLPID.usFeedBack;
	    __GmLeft.cRealDir = __MOTORGOAHEAD;
            __GmLPID.sFeedBack= __GmLPID.usFeedBack; 
	}
	
	if(Dir_R==1)//向下计数  前进
	{
            __GmRPID.usFeedBack = 32768 - __GmRPID.usEncoder_new;
           
            TIM_SetCounter(TIM3, 32768);
            __GmRight.uiPulseCtr += __GmRPID.usFeedBack;
	    __GmRight.cRealDir = __MOTORGOAHEAD;
	    __GmRPID.sFeedBack = __GmRPID.usFeedBack;
        }  
	else
	{ 
            __GmRPID.usFeedBack = __GmRPID.usEncoder_new - 32768;
            TIM_SetCounter(TIM3, 32768);
           // if(__GmRPID.usFeedBack>300)
           //   __GmRPID.usFeedBack=76;
            __GmRight.uiPulseCtr += __GmRPID.usFeedBack;   
            __GmRight.cRealDir = __MOTORGOBACK;
            __GmRPID.sFeedBack = -1*__GmRPID.usFeedBack;  
           
	}
	__GmSPID.sFeedBack = (__GmRPID.sFeedBack + __GmLPID.sFeedBack)/2 ;
        __GmWPID.sFeedBack = (__GmRPID.sFeedBack - __GmLPID.sFeedBack)/2 ;	
    /*    if(quzabo == 0){__GmRPID.sFeedBack = __GmLPID.sFeedBack = 0;}
        //__GmSPID.sFeedBack = (__GmRPID.sFeedBack + __GmLPID.sFeedBack)/2 ;
        //__GmWPID.sFeedBack = (__GmRPID.sFeedBack - __GmLPID.sFeedBack)/2 ;    //zzzzzz  yuanlai 2
    
        __GmSPID.sFeedBack = (0.5448*__GmRPID.sFeedBack + 0.5448*__GmLPID.sFeedBack)/2 ;//0.52044
       
        if( __GmSPID.sFeedBack>=50)
        {
          __GmWPID.sFeedBack = (0.5448*(__GmRPID.sFeedBack-0.8) - 0.5448*__GmLPID.sFeedBack)/7.03 ; //6.8cm轮子间距
        }
        else
        __GmWPID.sFeedBack = (0.5448*(__GmRPID.sFeedBack) - 0.5448*__GmLPID.sFeedBack)/7.03 ; //6.8cm轮子间距
     
        zhixianjuli = zhixianjuli +  __GmSPID.sFeedBack*0.001;
       bianmaqijiaodu = bianmaqijiaodu + __GmWPID.sFeedBack*0.001;
      
       
       
       
        if(start_firstge == 1)
        {
         if(zhixianjuli>=13.2){start_firstge = 0;zhixianjuli = 0;}
        }
        
        
        if(start_firstge == 0)
        {
          
         
           if((zhixianjuli<=9)&&(zhixianjuli>0)){start_A=1;}else {start_A=0;}
           if(zhixianjuli>9){start_B=1;}else {start_B=0;}
           if(zhixianjuli>=18){zhixianjuli = zhixianjuli - 18;}
          
        }
       
      */
        
        
       
        
}




void __irSendFreq (int8  __cNumber)
{
    switch (__cNumber) 
    {
      case 1: /*前方红外*/                                                            
          GPIO_SetBits(GPIOA,GPIO_Pin_5);  //前方左 开
           break;
  
      case 2:  /*左右红外*/                                                         
          GPIO_SetBits(GPIOC,GPIO_Pin_13); //前方右 开
          break;  
      case 3:                                                         
          GPIO_SetBits(GPIOA,GPIO_Pin_3); //左 开
          break; 
      case 4:                                                         
          GPIO_SetBits(GPIOC,GPIO_Pin_2); //右 开
          break;           
      default:
          break;
    }
}

void __irCheck (void)
{      
    static uint8 ucState = 0;
    switch(ucState)
    {
        case 0:
          ucIRCheck[3] = ADC_ConvertedValue[3]-Sunlight[3];
        if((ucIRCheck[3]<0)||(ucIRCheck[3]>35000))
                ucIRCheck[3]=0;          
          GPIO_ResetBits(GPIOC,GPIO_Pin_2); 
          if(ucIRCheck[3]>GusDistance_R_Far)
          {
              __GucDistance[__RIGHT]  |= 0x01;
          }         
          else
          {
              __GucDistance[__RIGHT]  &= 0xfe;              
          }
          
          if(ucIRCheck[3]>GusDistance_R_Mid)
          {
              __GucDistance[__RIGHT]  |= 0x02;
          }
          
          else
          {
              __GucDistance[__RIGHT]  &= 0xfd;
          }  
          
           if(ucIRCheck[3]>GusDistance_R_Near)
          {
              __GucDistance[__RIGHT]  |= 0x04;
          }
          
          else
          {
              __GucDistance[__RIGHT]  &= 0xfb;
          }           
          Sunlight[0]=0;
          __delay(1000);          
          Sunlight[0]=ADC_ConvertedValue[0];
          __irSendFreq (1);
          break;
          
        case 1:
          ucIRCheck[0] = ADC_ConvertedValue[0]-Sunlight[0];
           if((ucIRCheck[0]<0)||(ucIRCheck[0]>30000))
               ucIRCheck[0]=0;               
          GPIO_ResetBits(GPIOA,GPIO_Pin_5);
          
          if(ucIRCheck[0]>GusDistance_FL_Far)
          {
              __GucDistance[__FRONTL]  |= 0x01;
          }
          else
          {
               __GucDistance[__FRONTL] &= 0xfe;
          }  
          if(ucIRCheck[0]>GusDistance_FL_Near)
          {
              __GucDistance[__FRONTL]  |= 0x02; 
          }
          else
          {
              __GucDistance[__FRONTL]  &= 0xfd;
          }
          Sunlight[1]=0;
          __delay(1000);          
          Sunlight[1]=ADC_ConvertedValue[1];
          __irSendFreq (2);            
          break;
          
        case 2:          
          ucIRCheck[1] = ADC_ConvertedValue[1]-Sunlight[1];
          if((ucIRCheck[1]<0)||(ucIRCheck[1]>30000))
              ucIRCheck[1]=0;              
          GPIO_ResetBits(GPIOC,GPIO_Pin_13);
          if(ucIRCheck[1]>GusDistance_FR_Far)
          {
              __GucDistance[__FRONTR]  |= 0x01;
          }
          else
          {
               __GucDistance[__FRONTR] &= 0xfe;
          } 
          
          if(ucIRCheck[1]>GusDistance_FL_Near)
          {
              __GucDistance[__FRONTR]  |= 0x02;
          }
          else
          {
               __GucDistance[__FRONTR]  &= 0xfd;
          }          
          if((ucIRCheck[0]>GusDistance_FL_Far)&&(ucIRCheck[1]<GusDistance_FR_Far))
          {
            GucGoHead =1;
          }
          else if((ucIRCheck[0]<GusDistance_FL_Far)&&(ucIRCheck[1]>GusDistance_FR_Far))
          {
            GucGoHead1 =1;
          }
          else
          {
            GucGoHead =0;
            GucGoHead1 =0;
          }  
           Sunlight[2]=0;
            __delay(1000);          
           Sunlight[2]=ADC_ConvertedValue[2];
          __irSendFreq (3);  
          break;
          
        case 3:
          ucIRCheck[2] = ADC_ConvertedValue[2]-Sunlight[2];
          if((ucIRCheck[2]<0)||(ucIRCheck[2]>30000))
             ucIRCheck[2]=0;           
          GPIO_ResetBits(GPIOA,GPIO_Pin_3);
          if(ucIRCheck[2]>GusDistance_L_Far)
          {
              __GucDistance[__LEFT]   |= 0x01;
          }
          else
          {
              __GucDistance[__LEFT]   &= 0xfe;
          }
          
          if(ucIRCheck[2]>GusDistance_L_Mid)
          {
              __GucDistance[__LEFT]   |= 0x02;
          }
          else
          {
              __GucDistance[__LEFT]   &= 0xfd;
          }
          
          if(ucIRCheck[2]>GusDistance_L_Near)
          {
              __GucDistance[__LEFT]   |= 0x04;
          }
          else
          {
              __GucDistance[__LEFT]   &= 0xfb;
          }  
           Sunlight[3]=0;
           __delay(1000);                 
          Sunlight[3]=ADC_ConvertedValue[3];
          __irSendFreq (4);  
          break;
          default: 
            break;
    }  
    ucState = (ucState + 1) % 4;    
}

void sensorDebug (void)
{
    zlg7289Download(2, 0, 0, __GucDistance[__LEFT  ]);//传感器左
    zlg7289Download(2, 1, 0, __GucDistance[__FRONTL]);//传感器前左
    zlg7289Download(2, 2, 0, __GucDistance[__FRONTR]);//传感器前右    
    zlg7289Download(2, 3, 0, __GucDistance[__RIGHT ]);//传感器右
}

/*********************************************************************************************************
** Function name:       __rightMotorContr
** Descriptions:        右直流电机驱动
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __rightMotorContr(void)
{
    switch (__GmRight.cDir) 
    {
    case __MOTORGOAHEAD:                                                
      TIM_SetCompare1(TIM1,__GmRight.sSpeed);
      TIM_SetCompare2(TIM1,0);
        break;

    case __MOTORGOBACK:                                                 
      TIM_SetCompare1(TIM1,0);
      TIM_SetCompare2(TIM1,__GmRight.sSpeed);
        break;
		
    case __MOTORGOSTOP:                                                   
      TIM_SetCompare1(TIM1,0);		
      TIM_SetCompare2(TIM1,0);
        break;

    default:
        break;
    }
}

/*********************************************************************************************************
** Function name:       __leftMotorContr
** Descriptions:        左直流电机驱动
** input parameters:    __GmLeft.cDir :电机运行方向
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __leftMotorContr(void)
{
    switch (__GmLeft.cDir) 
    {
    case __MOTORGOAHEAD:                                            
      TIM_SetCompare4(TIM1,0);                                          
      TIM_SetCompare3(TIM1,__GmLeft.sSpeed);
        break;

    case __MOTORGOBACK:                                                 
      TIM_SetCompare4(TIM1,__GmLeft.sSpeed);                                          
      TIM_SetCompare3(TIM1,0);
        break;
		
    case __MOTORGOSTOP:                                                 
      TIM_SetCompare3(TIM1,0);                                          
      TIM_SetCompare4(TIM1,0);
        break;
		
    default:
        break;
    }
}

/*********************************************************************************************************
** Function name:       __SpeedUp
** Descriptions:        电脑鼠加速程序
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __SpeedUp (void)
{
    uint16 Speed;
    Speed=__GmSPID.sFeedBack;
    if(__GmSPID.sRef<__GiMaxSpeed){
      if(Speed >=__GmSPID.sRef)
      {
        __GmSPID.sRef=__GmSPID.sRef+10;//8 10
      }
    }   
}
/*********************************************************************************************************
** Function name:       __SpeedDown
** Descriptions:        电脑鼠减速程序
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __SpeedDown (void)
{
    uint16 Speed;
    Speed=__GmSPID.sFeedBack;
    if(__GmSPID.sRef>=MINSPEED)
    {
      if(Speed <=__GmSPID.sRef+3)
      {
       __GmSPID.sRef=__GmSPID.sRef-2;//2
      }
    }
}


/*********************************************************************************************************
** Function name:       startCheck
** Descriptions:        读取按键
** input parameters:    无
** output parameters:   无
** Returned value:      true:  按键已按下
**                      false: 按键未按下
*********************************************************************************************************/
uint8 startCheck (void)
{
  if (GPIO_ReadInputDataBit(GPIOC, __START) == 0) {
        __delay(50);
        while(GPIO_ReadInputDataBit(GPIOC, __START) == 0);
        return(true);
    }else {
        return(false);
    }
}




void TIM2_IROHandler(void)
{
	TIM_ClearFlag(TIM2,TIM_FLAG_Update);
}


void TIM3_IRQHandler(void)
{
	TIM_ClearFlag(TIM3,TIM_FLAG_Update);
}

u32 GyroscopeDebug[8] = {0};
/*
SourceDate : 采样值存储地址
NewDate : 新采样值
Average ：返回值
*/
void GyroscopeFilter(u32 NewDate,u32 *SourceDate,u32 *Average)
{
   
  unsigned char i = 0;
  u32 NumMax,NumMin = 0;
  unsigned char MaxCount,MinCount = 0;
  u32 Sum = 0;
  for(i = 0;i < 7;i++)
   {
     *(SourceDate + i) = *(SourceDate + i + 1);   
   }
   *(SourceDate + 7) = NewDate;
  NumMax = *SourceDate;
  NumMin = *SourceDate;
  MaxCount = 0;
  MinCount = 0;
  for(i = 1;i < 8;i++)
  {
    if(*(SourceDate + i) > NumMax)        //如果大于当前最大值 
    {
      NumMax = *(SourceDate + i);
      MaxCount = i;
    }
    else if(*(SourceDate + i) < NumMin )
    {
      NumMin = *(SourceDate + i);
      MinCount = i;
    }
    else
    {}
  }
  if(NumMax == NumMin)
  {
    *Average = NumMax;
  }
  else 
  {
    Sum = 0;
    for(i = 0;i < 8;i++)
    {
      if((i != MaxCount)&&(i != MinCount))
      {
        Sum += *(SourceDate + i);
      }
    }
    *Average = Sum/6;
  }
  
}
//zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz
float ADCMVdianyar=0;
float ADCMVdianya=0;
float  ADCMVdianyafl=0;
float  ADCMVdianyafr=0;
static float leftdis=0;
static float rightdis=0;
static float fldis=0;
static float frdis=0;

void TIM6_IRQHandler(void)
{
   
    static uint8 ucState = 0;
    static uint16 integer=0;      //红外距离表整数部分
    static float decimal=0;      //红外距离表小数部分
    static uint16 integerr=0;
    static float decimalr=0; 
    
     static uint16 integerfl=0;
    static float decimalfl=0; 
        static uint16 integerfr=0;
           static float decimalfr=0;
    
    
    
    //unsigned char i = 0;
    TIM_ClearFlag(TIM6,TIM_FLAG_Update);
    GyroscopeFilter(ADC_ConvertedValue[4],Angle_TLY,&Angle_TLY_Average);
    
   
    
     switch(ucState)
    {
        case 0:
            Sunlight[3] = ADC_ConvertedValue[3];
            ucState++;
            __irSendFreq (4);
            break;
            
        case 1:
            ucIRCheck[3] = ADC_ConvertedValue[3]-Sunlight[3];
             if(ucIRCheck[3]>35000)
                ucIRCheck[3] = 0;  
            ADCMVdianya=ADCmv(ucIRCheck[3]);
            if(ADCMVdianya<300){ADCMVdianya = 300;}
            integer=(int)((ADCMVdianya-300)/15);//100  16
            decimal=(float)(((int)ADCMVdianya-300)%15)/15;
            rightdis=(disCheck3[integer]-disCheck3[integer + 1])*decimal+disCheck3[integer + 1];
                   
            GPIO_ResetBits(GPIOC,GPIO_Pin_2); 
            if(ucIRCheck[3]>GusDistance_R_Far)
            {
                __GucDistance[__RIGHT]  |= 0x01;
            }         
            else
            {
                __GucDistance[__RIGHT]  &= 0xfe;              
            } 
          
            if(ucIRCheck[3]>GusDistance_R_Mid)
            {
                __GucDistance[__RIGHT]  |= 0x02;
            }
          
            else
            {
                __GucDistance[__RIGHT]  &= 0xfd;
            }  
          
            if(ucIRCheck[3]>GusDistance_R_Near)
            {
                __GucDistance[__RIGHT]  |= 0x04;
            }
          
            else
            {
                __GucDistance[__RIGHT]  &= 0xfb;
            }
            ucState++;
           // __irSendFreq (1);
            break;
            
        case 2:
            Sunlight[0] = ADC_ConvertedValue[0];
            ucState++;
            __irSendFreq (1);
            break;
            
        case 3:
           ucIRCheck[0] = ADC_ConvertedValue[0]-Sunlight[0];
             ADCMVdianyafl=ADCmv(ucIRCheck[0]);
            if(ucIRCheck[0]>30000)
                ucIRCheck[0]=0;    
            
            if(ADCMVdianyafl<700){ADCMVdianyafl = 700;}
            integerfl=(int)((ADCMVdianyafl-700)/16);//700  16
            decimalfl=(float)(((int)ADCMVdianyafl-700)%16)/16;
            fldis=(disCheck0[integerfl]-disCheck0[integerfl + 1])*decimalfl+disCheck0[integerfl + 1];//线性取值
            
            
            
            GPIO_ResetBits(GPIOA,GPIO_Pin_5);
          
            if(ucIRCheck[0]>GusDistance_FL_Far)
            {
                __GucDistance[__FRONTL]  |= 0x01;
            }
            else
            {
                __GucDistance[__FRONTL] &= 0xfe;
            }  
            if(ucIRCheck[0]>GusDistance_FL_Near)
            {
                __GucDistance[__FRONTL]  |= 0x02; 
            } 
            else
            {
                __GucDistance[__FRONTL]  &= 0xfd;
            }
            //__irSendFreq (2);
            ucState++;
            break;
            
        case 4:
            Sunlight[1] = ADC_ConvertedValue[1];
            ucState++;
            __irSendFreq (2);
            break;
            
        case 5:
            ucIRCheck[1] = ADC_ConvertedValue[1]-Sunlight[1];
            ADCMVdianyafr=ADCmv(ucIRCheck[1]);
            if(ucIRCheck[1]>30000)
                ucIRCheck[1]=0;   
            
           if(ADCMVdianyafr<700){ADCMVdianyafr = 700;}
            integerfr=(int)((ADCMVdianyafr-700)/16);
            decimalfr=(float)(((int)ADCMVdianyafr-700)%16)/16;
            frdis=(disCheck1[integerfr]-disCheck1[integerfr + 1])*decimalfr+disCheck1[integerfr + 1];//线性取值
            
            
            GPIO_ResetBits(GPIOC,GPIO_Pin_13);
            if(ucIRCheck[1]>GusDistance_FR_Far)
            {
                __GucDistance[__FRONTR]  |= 0x01;
            }
            else
            {
                __GucDistance[__FRONTR] &= 0xfe;
            } 
          
            if(ucIRCheck[1]>GusDistance_FR_Near)
            {
                __GucDistance[__FRONTR]  |= 0x02;
            }
            else
            {
                __GucDistance[__FRONTR]  &= 0xfd;
            }          
            if((ucIRCheck[0]>GusDistance_FL_Far)&&(ucIRCheck[1]<GusDistance_FR_Far))
            {
                GucGoHead =1;
            }
            else if((ucIRCheck[0]<GusDistance_FL_Far)&&(ucIRCheck[1]>GusDistance_FR_Far))
            {
                GucGoHead1 =1;
            }
            else if((ucIRCheck[0]>GusDistance_FL_Far)&&(ucIRCheck[1]>GusDistance_FR_Far))
            {
                GucGoHead2 =1;
            }
            else
            {
                GucGoHead =0;
                GucGoHead1 =0;
                GucGoHead2 =0;
            }
            ucState++;
            //__irSendFreq (3);
            break;
            
        case 6:
            Sunlight[2] = ADC_ConvertedValue[2];
            ucState++;
            __irSendFreq (3);
            break;
            
        case 7:
            ucIRCheck[2] = ADC_ConvertedValue[2]-Sunlight[2];
             if(ucIRCheck[2]>30000)
                ucIRCheck[2]=0;  
            ADCMVdianyar=ADCmv(ucIRCheck[2]);
            if(ADCMVdianyar<300){ADCMVdianyar = 300;}
            integerr=(int)((ADCMVdianyar-300)/15);//100  16
            decimalr=(float)(((int)ADCMVdianyar-300)%15)/15;
            leftdis=(disCheck2[integerr]-disCheck2[integerr + 1])*decimal+disCheck2[integerr + 1];//线性取值
                    
            GPIO_ResetBits(GPIOA,GPIO_Pin_3);
            if(ucIRCheck[2]>GusDistance_L_Far)
            {
                __GucDistance[__LEFT]   |= 0x01;
            }
            else
            {
                __GucDistance[__LEFT]   &= 0xfe;
            }
          
            if(ucIRCheck[2]>GusDistance_L_Mid)
            {
                __GucDistance[__LEFT]   |= 0x02;
            }
            else
            {
                __GucDistance[__LEFT]   &= 0xfd;
            }
          
            if(ucIRCheck[2]>GusDistance_L_Near)
            {
                __GucDistance[__LEFT]   |= 0x04;
            }
            else
            {
                __GucDistance[__LEFT]   &= 0xfb;
            }
            if((__GucDistance[__FRONTR] & 0x02)&&(__GucDistance[__FRONTL] & 0x02))
            {
              GucFrontJinju = 1;
            }
            else 
            {
              GucFrontJinju = 0;
            }
            
            ucState++;
           
            break;
            
        default:
            break;
    }
    
    ucState = ucState % 8;
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

uint8 zuihouyige=0;

uint16 lingshiL,lingshiR,lingshizuixiaoL,lingshizuixiaoR=0;
uint8 lingshichengliL,lingshichengliR=0;

extern uint8 zijiaozheng0;


static float zijiaozheng=0;
uint16 zijiaozheng_time=0;
static float sum_zijiaozheng=0;
uint8 zijiaozheng_flag=1;

uint8 Start_goback=0;//返回点编码器清零标志

static u16 qidianclock=0;
static uint8 use_gyro=1;
extern uint8 GYROInit;
static uint8 GYROSTAT=1;
static float GYROTim=0;
static float GYRONum=0;
static float GYROZero=0;

void SysTick_Handler(void)
{  
   static int8 n = 0,m=0 ,k=0,l=0,a=0,b=0,c=0,w=0;
    static u16  t=0, s=0;
    static u32  encoder=0;
    uint16 Sp;
    if(zijiaozheng0==1)
        {
        if(zijiaozheng_flag)
        {  
         
          sum_zijiaozheng += Angle_TLY_Average;
          zijiaozheng_time++;
          if(zijiaozheng_time==3000){zijiaozheng=sum_zijiaozheng/3000;zijiaozheng_flag=0;jiao=0;}
          
        }
        
        }
    __Encoder(); 
    TIM6_IRQHandler();
    Sp=__GmSPID.sFeedBack;
    encoder+=Sp;
    t++;
    s++;
    if(t == 1000)
    {
        t = 0;
        GPIO_SetBits(GPIOB, GPIO_Pin_12);
        if(s == 2000)
        {
            s = 0;
            GPIO_ResetBits(GPIOB, GPIO_Pin_12);
        }
    }


        
        switch (__GmRight.cState) {
     case __MOTORSTOP:                                                   /*  停止，同时清零速度和脉冲值  */
              __GmRight.uiPulse    = 0;
              __GmRight.uiPulseCtr = 0;
              __GmLeft.uiPulse    = 0;
              __GmLeft.uiPulseCtr = 0;
              break;
      
          case __WAITONESTEP:                                                 /*  暂停一步                    */
              __GmRight.cState = __MOTORRUN;
              if(leftdis<100)                    //直线校正
               {
                 GsTpusle_T = 1*(leftdis - 72);__PIDContr();//GPIO_ResetBits(GPIOB,GPIO_Pin_12);
                
               }
               else if(rightdis<100)
               {
                 GsTpusle_T = -1*(rightdis - 84);__PIDContr();//GPIO_ResetBits(GPIOB,GPIO_Pin_12);
               }
              break;
      //75    65
          case __MOTORRUN:                                                    /*  电机运行                    */
            if (__GucMouseState == __GOAHEAD)                                 /*  根据传感器状态微调电机位置  */
            {                              
                 if(leftdis<120)
                  {
                    GsTpusle_T = 1*(leftdis -75);GPIO_SetBits(GPIOB,GPIO_Pin_12);
                    __PIDContr();
                  }
                  else if(rightdis<170)
                  {
                    GsTpusle_T = -1*(rightdis - 100);GPIO_ResetBits(GPIOB,GPIO_Pin_12);
                   __PIDContr();
                  }
                  else{
                    GsTpusle_T = 0;
                    __PIDContr();
                  }
                  if(GuiSpeedCtr==__SPEEDUP)
                  { 
                    k=(k+1)%5;//20
                    if(k==4)
                    __SpeedUp();
                  }
                  else if(GuiSpeedCtr==__SPEEDDOWN)
                  {
                      k=(k+1)%10;
                      if(k==5)
                      __SpeedDown(); 

                  }
                  else;
                 
            }
           
            else if (__GucMouseState == __GOAHEAD_45)  //直线45矫正
            {       
              if((ucIRCheck[0]>DisFlX))//&&(!zuihouyige))//左前
              {
                    GsTpusle_T=-20;
                    voltageDetect(); 
                    __PIDContr();      
              }
                
              else if((ucIRCheck[1]>DisFrX))//&&(!zuihouyige))//右前
              {
                    GsTpusle_T=20;
                    voltageDetect();
                    __PIDContr();
              }
              else if((ucIRCheck[0]<DisFrX)&&(ucIRCheck[1]<DisFlX))//左前，右前都无墙
              {
                  if(leftdis<40)//37
                  {
                  GsTpusle_T=10.0*(leftdis - 40);//37 40
                      voltageDetect(); 
                      __PIDContr();
                  }
                  else if(rightdis<55) //55
                  {
                  GsTpusle_T=-10.0*(rightdis - 55);//55
                      voltageDetect(); 
                      __PIDContr();
                  }
                 else
                 {
                 voltageDetect();
                  GsTpusle_T = 3*jiao;
                  __PIDContr();
                 }
              }
              else
              {
                GsTpusle_T=0;
              }
              __PIDContr();
              if(GuiSpeedCtr==__SPEEDUP)
                  { 
                    k=(k+1)%10;//20
                    if(k==9)
                    __SpeedUp();
                  }
              else if(GuiSpeedCtr==__SPEEDDOWN)
                  {
                      k=(k+1)%10;
                      if(k==5)
                      __SpeedDown(); 

                  }
              else;
          }
          else if(__GucMouseState == __RUN90VR)
          {
            if(ucIRCheck[1]>DisFrX)
            {
              GsTpusle_T=20;
              voltageDetect();
              __PIDContr();
            }  
            else 
            {
                if(rightdis<55)
                {
                GsTpusle_T=-10.0*(rightdis - 55);
                voltageDetect(); 
                __PIDContr();
                }
               else
               {
               voltageDetect();
                GsTpusle_T = 3*jiao;
                __PIDContr();
               }
            }
            __PIDContr();
          }
          else if(__GucMouseState == __RUN90VL)  //连续90V型转弯
          {
            if(ucIRCheck[0]>DisFlX)
            {
              GsTpusle_T=-20;
              voltageDetect();
            }  
            else 
            {
                if(leftdis<38)
                {
                GsTpusle_T=10.0*(leftdis - 38);
                voltageDetect(); 
                __PIDContr();
                }
               else
               {
               voltageDetect();
                GsTpusle_T = 3*jiao;
                __PIDContr();
               }
            }
            __PIDContr();
          }
          else if(__GucMouseState == __AFTER)
                  {
                    voltageDetect();
                    GsTpusle_T = 3*jiao;
                    __PIDContr();
                  }
          else if (__GucMouseState == __SPURT45)                                 /*  根据传感器状态微调电机位置  */
            {                              
                 if(leftdis<80)
                  {
                    GsTpusle_T = 0.8*(leftdis - 38);GPIO_SetBits(GPIOB,GPIO_Pin_12);  //37
                    __PIDContr();
                  }
                  else if(rightdis<85)
                  {
                    GsTpusle_T =- 0.8*(rightdis - 57);GPIO_ResetBits(GPIOB,GPIO_Pin_12);//54
                   __PIDContr();
                  }
                 else
                 { GsTpusle_T=0;  __PIDContr();}
                 
                 
                  if(GuiSpeedCtr==__SPEEDUP)
                  { 
                    k=(k+1)%5;//20
                    if(k==4)
                    __SpeedUp();
                  }
                  else if(GuiSpeedCtr==__SPEEDDOWN)
                  {
                      k=(k+1)%10;
                      if(k==5)
                      __SpeedDown(); 

                  }
                  else;
            }
          else{
            GsTpusle_T = 0;
            voltageDetect1();                 //陀螺仪
            voltageDetect();                 
            }                  
            __PIDContr();     
            break;
           
          case 4:
            GsTpusle_T = 0;
            __PIDContr();
            break;
            
//          case 5:
//            __GmRight.sSpeed = 10;
//            __rightMotorContr();
//            __GmRight.cState = __MOTORSTOP;
//            __GmLeft.sSpeed = 10;
//            __leftMotorContr();
//            __GmLeft.cState = __MOTORSTOP;
//            break;
            
           default:
             break;
    }           
}
/*********************************************************************************************************
** Function name:       mazeSearch
** Descriptions:        前进N格
** input parameters:    iNblock: 前进的格数
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void mouseGoahead1 (int8  cNBlock)                       //冲刺看前方红外，无坐标更新,当goto end速度大于20时设定速度为20
{
    int8 cL = 0, cR = 0, cCoor = 1,cB;
    GuiSpeedCtr=__SPEEDUP;
    if (__GmLeft.cState) {
        cCoor = 0;
    }
    if(cNBlock==1)
    {
        cL = 1;
        cR = 1;
        if(GucFangXiang == GucDirTemp)
        {
           GuiTpusle_LR = 3200;
           GuiTpusle_S = 0;
        }
        else
        {
           GuiTpusle_LR = 3200;
           GuiTpusle_S = 0;
        }
        __GiMaxSpeed = 100;
    }
    else{
        GuiTpusle_LR = 0;
        GuiTpusle_S = 0;
    }
    GucFangXiang = GucDirTemp;
    if(((GmcMouse.cX==7)&&(GmcMouse.cY==7))|| 
         ((GmcMouse.cX==8)&&(GmcMouse.cY==8))||
         ((GmcMouse.cX==7)&&(GmcMouse.cY==8))||
           ((GmcMouse.cX==8)&&(GmcMouse.cY==7))){
       cL = 0;
       cR = 0; 
       GuiTpusle_LR = 0;
       GuiTpusle_S = 0;
    }
    if((__GucMouseState==__TURNRIGHT)||(__GucMouseState==__TURNLEFT))
    {
        __GmLeft.uiPulseCtr = 14400;       //1182(34mm)
        __GmRight.uiPulseCtr = 14400;//7200
    }
    
    cB=cNBlock;
    __GucMouseState   = __GOAHEAD;
   // __GiMaxSpeed      =   25;
    __GmRight.uiPulse = __GmRight.uiPulse + cNBlock * ONEBLOCK ;
    __GmLeft.uiPulse  = __GmLeft.uiPulse  + cNBlock * ONEBLOCK ;
    __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
    if(cNBlock > 5)
    {
      __GiMaxSpeed=140;
    }
    else if(cNBlock == 5)
    {
      __GiMaxSpeed=132;
    }
    else if(cNBlock == 4)
    {
      __GiMaxSpeed=120;
    }
    else if(cNBlock == 3)
    {
      __GiMaxSpeed=108;
    }
    else if(cNBlock == 2)
    {
      __GiMaxSpeed=100;
    }
    else
     ;
    while (__GmLeft.cState != __MOTORSTOP) {
       
        if (__GmLeft.uiPulseCtr >= ONEBLOCK) {                          /*  判断是否走完一格            */
            __GmLeft.uiPulse    -= ONEBLOCK;
            __GmLeft.uiPulseCtr -= ONEBLOCK;
            if (cCoor) {
                cNBlock--;
                if(cNBlock==0)
                   goto End;
                if(cNBlock<cB-1)//给回速一个时间
                  GuiSpeedCtr=__SPEEDUP;
            } else {
                cCoor = 1;
            }
        }
        
        if (__GmRight.uiPulseCtr >= ONEBLOCK) {                         /*  判断是否走完一格            */
            __GmRight.uiPulse    -= ONEBLOCK;
            __GmRight.uiPulseCtr -= ONEBLOCK;
        }
        if (cNBlock < 2) {
          if(__GmSPID.sFeedBack>100){
              GuiSpeedCtr= 3;
             __GmSPID.sRef=100;
          }  
          if (cL) 
          {                                                       /*  是否允许检测左边            */
            if (ucIRCheck[2]<GusDistance_L_Far)             /*  左边有支路，跳出程序        */
            {                 
                __GmRight.uiPulse = __GmRight.uiPulseCtr  + 6000- GuiTpusle_LR;    //3094(89mm)
                __GmLeft.uiPulse  = __GmLeft.uiPulseCtr   + 6000- GuiTpusle_LR;
                while (ucIRCheck[2]<GusDistance_L_Far) 
                {
                    
                    if ((__GmLeft.uiPulseCtr + 600) > __GmLeft.uiPulse) 
                    {
                        goto End;
                    }
                }
            }
            } else {                                                    /*  左边有墙时开始允许检测左边  */
                if (ucIRCheck[2]>GusDistance_L_Far) {
                    cL = 1;
                }
            }
         if (cR) 
            {                                                       /*  是否允许检测右边            */
            if (ucIRCheck[3]<GusDistance_R_Far)               /*  右边有支路，跳出程序        */
            {                
                __GmRight.uiPulse = __GmRight.uiPulseCtr + 6000- GuiTpusle_LR;
                __GmLeft.uiPulse  = __GmLeft.uiPulseCtr + 6000- GuiTpusle_LR;
                while (ucIRCheck[3]<GusDistance_R_Far) 
                {
                    
                    if ((__GmLeft.uiPulseCtr + 600) > __GmLeft.uiPulse) 
                    {
                        goto End;
                    }
                }
            }
            } else {
                if (ucIRCheck[3]>GusDistance_R_Far) {                   /*  右边有墙时开始允许检测右边  */
                    cR = 1;
                }
            }
        }
   }
    /*
     *  设定运行任务，让电脑鼠走到支路的中心位置
     */
End:     ;//GuiSpeedCtr= 3;
         //__GmSPID.sRef=18;
}


static uint8 Continuous=0;
uint8 final;
void mouseGoahead(int8  cNBlock)                                
{
    int8 cL = 0, cR = 0, cCoor = 1,cB;
    GuiSpeedCtr=__SPEEDUP;
    if (__GmLeft.cState)                   
    {                  
        cCoor = 0;
    }
    if(cNBlock==1)
    {
        cL = 1;
        cR = 1;
        if(GucFangXiang == GucDirTemp)      
        {
           GuiTpusle_LR =15000;
           GuiTpusle_S  = 0;               
        }
        else
        {
           GuiTpusle_LR = 10000;
           GuiTpusle_S  = 600;
        }
        __GiMaxSpeed = 100;
    }
    else
    {
        GuiTpusle_LR = 0;
        GuiTpusle_S  = 0;
        __GiMaxSpeed = 320;
    }
    GucFangXiang = GucDirTemp;
    if(((GmcMouse.cX==7)&&(GmcMouse.cY==7))|| 
       ((GmcMouse.cX==8)&&(GmcMouse.cY==8))||
       ((GmcMouse.cX==7)&&(GmcMouse.cY==8))||
       ((GmcMouse.cX==8)&&(GmcMouse.cY==7)))
    {
        cL = 0;
        cR = 0;
        GuiTpusle_LR = 0;
        GuiTpusle_S  = 0;
    }
    if((__GucMouseState==__TURNRIGHT)||(__GucMouseState==__TURNLEFT))
    {
        __GmLeft.uiPulseCtr = 30000;      
        __GmRight.uiPulseCtr= 30000;

    }
    
    cB=cNBlock;
    __GucMouseState   = __GOAHEAD;
    __GmRight.uiPulse = __GmRight.uiPulse + cNBlock * ONEBLOCK ;
    __GmLeft.uiPulse  = __GmLeft.uiPulse  + cNBlock * ONEBLOCK ;
    __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
    if(cNBlock > 5)
    {
        __GiMaxSpeed=130;
    }
    else if(cNBlock == 5)
    {
        __GiMaxSpeed=130;
    }
    else if(cNBlock == 4)
    {
        __GiMaxSpeed=130;
    }
    else if(cNBlock == 3)
    {
        __GiMaxSpeed=130;
    }
    else if(cNBlock == 2)
    {
        __GiMaxSpeed=130;
    }
    else;
       
    while (__GmLeft.cState != __MOTORSTOP)
    {
        if (__GmLeft.uiPulseCtr >= ONEBLOCK)                /*  判断是否走完一格            */
        {                          
            __GmLeft.uiPulse    -= ONEBLOCK;
            __GmLeft.uiPulseCtr -= ONEBLOCK;
            if (cCoor) 
            {
                cNBlock--;
                if(cNBlock==0)
                    goto End;
                if(cNBlock<cB-1)                          //给回速一个时间
                    GuiSpeedCtr=__SPEEDUP;
            } 
            else 
            {
                cCoor = 1;
            }
        }
        
        if (__GmRight.uiPulseCtr >= ONEBLOCK)                /*  判断是否走完一格            */
        {                         
            __GmRight.uiPulse    -= ONEBLOCK;
            __GmRight.uiPulseCtr -= ONEBLOCK;
        }
        if(cNBlock < 3)
        {
            if(__GmSPID.sFeedBack>160)
            {              
                 __GmSPID.sRef=160;
              //  GuiSpeedCtr= __SPEEDDOWN;
            }          
        }
        if (cNBlock < 2) 
        {         
            if(__GmSPID.sFeedBack>120)
            {
                GuiSpeedCtr= 3;
                __GmSPID.sRef=2;
            }  
          
            if (cL) 
            {                                                       /*  是否允许检测左边            */
                if (ucIRCheck[2]<GusDistance_L_Far)                 /*  左边有支路，跳出程序        */
                {                 
                    __GmRight.uiPulse = __GmRight.uiPulseCtr  + 27700 - GuiTpusle_LR;    
                    __GmLeft.uiPulse  = __GmLeft.uiPulseCtr   + 27700 - GuiTpusle_LR;
                    while (ucIRCheck[2]<GusDistance_L_Far) 
                    {                   
                        if ((__GmLeft.uiPulseCtr + 100) > __GmLeft.uiPulse) 
                        {
                            goto End;
                        }
                    }
                }
            } 
            else 
            {                                                    
                if (ucIRCheck[2]>GusDistance_L_Far)                 /*  左边有墙时开始允许检测左边  */
                {
                    cL = 1;
                }
            }
            if (cR) 
            {                                                       /*  是否允许检测右边            */
                if (ucIRCheck[3]<GusDistance_R_Far)                 /*  右边有支路，进入程序        */
                {                
                    __GmRight.uiPulse  = __GmRight.uiPulseCtr + 26500- GuiTpusle_LR;
                    __GmLeft.uiPulse   = __GmLeft.uiPulseCtr  + 26500- GuiTpusle_LR;
                    while (ucIRCheck[3]<GusDistance_R_Far) 
                    {                   
                        if ((__GmLeft.uiPulseCtr + 100) > __GmLeft.uiPulse) 
                        {
                            goto End;
                        }
                    }
                }
            } 
            else 
            {
                if (ucIRCheck[3]>GusDistance_R_Far)                 /*  右边有墙时开始允许检测右边  */
                {                   
                    cR = 1;
                }
            }
        }
   }


End:     ;

}









uint8 C_continuous=0;

static uint8 sssssspp=0;
uint8 qidianwucha=0;

void mouseGoahead_liang (int8  cNBlock)                                    //连续转弯用
{
   __GucMouseState   = __SPURT45;
   int8 cL = 0, cR = 0, cCoor = 1,cB;
   __GmLeft.uiPulse =0;          
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =0;           
   __GmRight.uiPulseCtr=0;
    GuiSpeedCtr=__SPEEDUP;
    if (__GmLeft.cState) {
        cCoor = 0;
    }
    GucFangXiang = GucDirTemp;
    if(((GmcMouse.cX==7)&&(GmcMouse.cY==7))|| 
         ((GmcMouse.cX==8)&&(GmcMouse.cY==8))||
         ((GmcMouse.cX==7)&&(GmcMouse.cY==8))||
           ((GmcMouse.cX==8)&&(GmcMouse.cY==7))){
       cL = 0;
       cR = 0;
       GuiTpusle_LR = 0;
       GuiTpusle_S  = 0;
    }
    if((__GucMouseState==__TURNRIGHT)||(__GucMouseState==__TURNLEFT))
    {
        __GmLeft.uiPulseCtr = 30000;       //1182(34mm)
        __GmRight.uiPulseCtr = 30000;

    }
    
    cB=cNBlock;
    __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
    __GucMouseState   = __SPURT45;
   // __GiMaxSpeed      =   25;
    __GmRight.uiPulse = __GmRight.uiPulse + cNBlock * ONEBLOCK ;
    __GmLeft.uiPulse  = __GmLeft.uiPulse  + cNBlock * ONEBLOCK ;
    __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
    
    if(cNBlock > 5)
    {
      __GiMaxSpeed=400;
    }
    else if(cNBlock == 5)
    {
      __GiMaxSpeed=320;
    }
    else if(cNBlock == 4)
    {
      __GiMaxSpeed=320;
    }
    else if(cNBlock == 3)
    {
      __GiMaxSpeed=320;
    }
    else if(cNBlock == 2)
    {
      __GiMaxSpeed=280;
    }
    else  __GiMaxSpeed=180;
    
    
    while (__GmLeft.cState != __MOTORSTOP)
    {
       
        if (__GmLeft.uiPulseCtr >= ONEBLOCK) 
        {                          /*  判断是否走完一格            */
            __GmLeft.uiPulse    -= ONEBLOCK;
            __GmLeft.uiPulseCtr -= ONEBLOCK;
            if (cCoor) {
                cNBlock--;
                if(cNBlock==0)
                   goto End;
             /*   if(cNBlock<cB-1)//给回速一个时间
                  GuiSpeedCtr=__SPEEDUP;*/
            } else {
                cCoor = 1;
            }
        }
        
        if (__GmRight.uiPulseCtr >= ONEBLOCK) {                         /*  判断是否走完一格            */
            __GmRight.uiPulse    -= ONEBLOCK;
            __GmRight.uiPulseCtr -= ONEBLOCK;
        }
              if(cNBlock < 3)
        {
            if(__GmSPID.sFeedBack>220)
            {              
                __GiMinSpeed=200;
                GuiSpeedCtr= __SPEEDDOWN;
            }          
        }
        
        
        if (cNBlock < 2)   //2//45度时，最后一格走完这么多脉冲后开始检测柱子
        {
           if(__GmSPID.sFeedBack>200)
            {
               __GmSPID.sRef=200;
                GuiSpeedCtr= __SPEEDDOWN;
            }
          while(1)
          {
           if (cL) 
           {                                                       /*  是否允许检测左边            */
              if  ((__GucDistance[__LEFT]  & 0x01)==0)
              {                 /*  左边有支路，跳出程序        */
                goto End;//直接进入下一个动作
              }
           } 
           else 
           {                                                        /*  左边有墙时开始允许检测左边  */
              if (ucIRCheck[2]>GusDistance_L_Far) 
              {
                  cL = 1; 
              }
           }//20550
          if (cR) 
          {                                                       /*  是否允许检测右边            */
              if ((__GucDistance[__RIGHT]  & 0x01)==0)
              {               /*  右边有支路，跳出程序        */
                goto End;
              }
          }
          else 
          {
              if (ucIRCheck[3]>GusDistance_R_Far)
              {                       
                  cR = 1;
              }
          }
         }
        }

   }
    /*
     *  设定运行任务，让电脑鼠走到支路的中心位置
     */
End:     ;
         //GuiSpeedCtr= 3;
         //__GmSPID.sRef=23;
}

void mouseTurnleft_90(void)
{
  
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse); 
   __GucMouseState   = __TURNLEFT; 
   __GmRight.cState = __MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  = __MOTORRUN;
   GucMouseDir     = (GucMouseDir + 6) % 8;                             /*  方向标记                    */
             
   __GmSPID.sRef=100;           ///120
   __GmWPID.sRef=80;//70
   GW=0;
    while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>85200)   
       {
         break;
       } 
      
   }
    
   __GmWPID.sRef=0;
   __GucMouseState   = __GOAHEAD;
 		//mouseStop();
 		//while(1)
			;	 
   GuiSpeedCtr=3;
   __GmLeft.uiPulse =2000;          
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =2000;           
   __GmRight.uiPulseCtr=0;
  while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
  while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);	
 
   __GucMouseState   = __TURNLEFT;
 	
   GuiSpeedCtr=__SPEEDUP;
  
   // __GmRight.cState = __MOTORSTOP;       
   // __GmLeft.cState  = __MOTORSTOP; 
      __GmRight.uiPulse    = 0;
              __GmRight.uiPulseCtr = 0;
              __GmLeft.uiPulse    = 0;
              __GmLeft.uiPulseCtr = 0;
             

}

void mouseTurnleft_90_t(void)
{
  int cL=0,cR=0;
  static int i=0;
  i++;
   __GmSPID.sRef=200;
   
     piancha_l = (rightdis - 55)/10;
   if(rightdis>125){piancha_l = 0;}//左转右边没墙
   if(piancha_l > 2){piancha_l = 2;}
    if(piancha_l < -2){piancha_l = -2;}
   if(usepiancha==0)piancha_l = 0;
   
   __GmLeft.uiPulse =18500-piancha_l*2886.67;    //9000    //14400      
    __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse=18500-piancha_l*2886.67;           
   __GmRight.uiPulseCtr=0;
     __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   
   
 
   GW=0;
   time=0;   
   __GucMouseState   = __TURNLEFT; 
   __GmRight.cState =__MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  =__MOTORRUN;
   GucMouseDir     = (GucMouseDir + 6) % 8;                            /*  方向标记                    */
   
  
   __GmWPID.sRef=120;
   jiao=0;
    while(1)
   {
       W=(float)jiao;
       if(W<-90*0.84)   
       {
         break;
       } 
      
   }
  // if(i==4){ mouseStop();while(1);}
   angle_temp=-90;
  // mouseStop();while(1);
   __GmSPID.sRef=200;
   __GmWPID.sRef=0;
   __GucMouseState   = __GOAHEAD;
   GuiSpeedCtr=3;
     if(actionsequence[action+1]==STRAIGHT90L){
     
     __GucMouseState   = __GOAHEAD;
      __GmSPID.sRef=280;
     __GmLeft.uiPulse =50400;   //60400   
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =50400;           
   __GmRight.uiPulseCtr=0;
   __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
  while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
  while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
  goto End;
       
     }
   
     else if((actionsequence[action+1]==RT90L)||(actionsequence[action+1]==RT135L)||(actionsequence[action+1]==RT45L)||(actionsequence[action+1]==R180L))

   {
     
     __GucMouseState   = __GOAHEAD;
     __GmRight.cState =__MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  =__MOTORRUN;
 while(1)
      {
           if (cR) 
           {                                                      
              if  (rightdis>160)
              {
                goto End;//直接进入下一个动作
              }
           } 
           else 
           {                                                     
              if (rightdis<160) 
              {
                  cR = 1; 
              }
          }  
      }
 goto End;

   }
   
     else if((actionsequence[action+1]==LT90L)||(actionsequence[action+1]==LT45L)||(actionsequence[action+1]==LT135L)){
        __GucMouseState   = __GOAHEAD;
         __GmSPID.sRef=200;
        
        __GmLeft.uiPulse =27400;   //  
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =27400;           
   __GmRight.uiPulseCtr=0;
   __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
  while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
  while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
        
        
     
      while(1)
      {
           if (cL) 
           {                                                      
              if  (leftdis>110)
              {
                goto End;//直接进入下一个动作
              }
           } 
           else 
           {                                                     
              if (leftdis<110) 
              {
                  cL = 1; 
              }
          }
      }
     
     goto End;
     
     }
     else {
       __GucMouseState   = __GOAHEAD;
     }
   
   
     __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
  while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
  while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
  
//   mouseStop();while(1);
 
    
   GuiSpeedCtr=__SPEEDUP;
    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
End:
    {//GuiSpeedCtr=__SPEEDUP;
   // __GmRight.cState = __MOTORSTOP;       
   // __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;}

}

void mouseTurnleft_90_v(void)
{     __GmSPID.sRef=200;
   uint8 cL = 0,cR=0;
   static int i=0;
   i++;
   __GmLeft.uiPulse =6400;    //9000 7500         
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =6400;           
   __GmRight.uiPulseCtr=0;

   GW=0;
   time=0;   
   __GucMouseState   = __TURNRIGHT; 
   __GmRight.cState =__MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  =__MOTORRUN;
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   GucMouseDir     = (GucMouseDir + 6) % 8;                            /*  方向标记                    */
   
    
   __GmWPID.sRef=120;
   jiao=0;
    while(1)
   {
       W=(float)jiao;
       if(W<-90*0.82)   
       {
         break;
       } 
      
   }
  // if(i==2) {mouseStop();while(1);}
  // mouseStop();while(1);
   angle_temp=-90;
   after_turn=1;
   __GmWPID.sRef=0; 
   __GmSPID.sRef=200;
   //GuiSpeedCtr=__SPEEDUP;
   if((actionsequence[action+1]==RV90L)||(actionsequence[action+1]==RV135L)||(actionsequence[action+1]==RV45L)) 
   {
          __GucMouseState = __RUN90VR;
           __GmSPID.sRef=200;
                    __GmLeft.uiPulse =7500;    //9000 7500         
                    __GmLeft.uiPulseCtr=0;
                   __GmRight.uiPulse =7500;           
                   __GmRight.uiPulseCtr=0; 
                   __GmRight.cState =__MOTORRUN;        
                   __GmLeft.cState  =__MOTORRUN;
                   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
                   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
       while(1)
      {
           if (cR) 
           {                           
              if  (rightdis>120)
              {__GmSPID.sRef=200;
                   __GmLeft.uiPulse =500;    //9000 7500         
                    __GmLeft.uiPulseCtr=0;
                   __GmRight.uiPulse =500;           
                   __GmRight.uiPulseCtr=0; 
                   __GmRight.cState =__MOTORRUN;        
                   __GmLeft.cState  =__MOTORRUN;
                   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
                   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
                   if(rightdis>120)
                   {
                      goto End;//直接进入下一个动作
                   }
                   else
                     cR=0;
              }
           } 
           else 
           {                                                     
              if (rightdis<120) 
              {
                  cR = 1; 
              }
          }  
      }
          //goto End;
    
   }
   else if(actionsequence[action+1]==LV45L)
   {
        __GucMouseState   = __RUN90VL;  
   while(1)
      {
           if (cL) 
           {                                                      
              if  (leftdis>120)
              {
                goto End;//直接进入下一个动作
              }
           } 
           else 
           {                                                     
              if (leftdis<120) 
              {
                  cL = 1; 
              }
          }  
      }
 goto End;
   }
   else
   {
      __GucMouseState = __GOAHEAD_45;
   }
   GuiSpeedCtr = __SPEEDUP ;
   __GmLeft.uiPulse = 22000;    //9000  //14400        
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse = 22000;           
   __GmRight.uiPulseCtr=0;
     __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
  while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
  while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
    GuiSpeedCtr=__SPEEDUP;
    //__GmRight.cState = __MOTORSTOP;       
    //__GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
   End:{
    GuiSpeedCtr=__SPEEDUP;
    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
}
}




void mouseTurnleft_135_t(void)
{
   int8 cR = 0,cL=0;
   static int i=0;
   i++;__GmSPID.sRef=200;
  
     piancha_l = (rightdis - 55)/10;
   if(rightdis>125){piancha_l = 0;}//左转右边没墙
   if(piancha_l > 2){piancha_l = 2;}
    if(piancha_l < -2){piancha_l = -2;}
   if(usepiancha==0)piancha_l = 0;
   
   __GmLeft.uiPulse =20300-piancha_l*2886.67;    //9000          
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =20300-piancha_l*2886.67;           
   __GmRight.uiPulseCtr=0;
   GW=0;
   time=0;   
   
   __GmRight.cState =__MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  =__MOTORRUN;
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   GucMouseDir     = (GucMouseDir + 5) % 8;                          
   
    __GucMouseState   = __TURNLEFT;
   __GmWPID.sRef=120;
   jiao=0;
    while(1)
   {
       W=(float)jiao;
       if(W<-135*0.85)   
       {
         break;
       } 
      
   }
 //if(i==1){mouseStop(); while(1);}
   angle_temp=-135;
   after_turn=1;

   __GmWPID.sRef=0; 
   __GmSPID.sRef=200;
      __GucMouseState = __GOAHEAD_45;
  // mouseStop(); while(1);
      
      
            //__GmSPID.sRef=160;
           
      
   if((actionsequence[action+1]==RV90L)||(actionsequence[action+1]==RV135L)) 
   
    { 
             lingshiL=ucIRCheck[2];  //左侧红外采集量赋给一个变量
             lingshiR=ucIRCheck[3];  //右侧红外采集量赋给一个变量
             lingshizuixiaoL=lingshiL;
             lingshizuixiaoR=lingshiR;
              __GucMouseState   = __RUN90VR;
                
     
   // mouseStop();while(1);
              while(1)
              {
                if(lingshiL<ucIRCheck[3]){lingshiL=ucIRCheck[3];}
                if(lingshiL>lingshizuixiaoL+400){lingshichengliL=1;}
                if(lingshichengliL)
                {
                  if(lingshiL>ucIRCheck[3]+700){lingshichengliL=0; goto End;}//break;
                }
              }//while循环结束，结束峰值检测
   }
   else if(actionsequence[action+1]==RV45L)   //特殊情况
   {
      __GucMouseState = __RUN90VR;
      __GmLeft.uiPulse =37000;    //9000  //14400        
      __GmLeft.uiPulseCtr=0;
      __GmRight.uiPulse =37000;           
      __GmRight.uiPulseCtr=0;
      __GmRight.cState  = __MOTORRUN;
      __GmLeft.cState   = __MOTORRUN;
      while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
      while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
         // mouseStop(); while(1);
      goto End;
   }
   else  if((actionsequence[action+1]==LV45L)||(actionsequence[action+1]==LV90L)||(actionsequence[action+1]==LV135L))
   { 
             lingshiL=ucIRCheck[2];  //左侧红外采集量赋给一个变量
             lingshiR=ucIRCheck[3];  //右侧红外采集量赋给一个变量
             lingshizuixiaoL=lingshiL;
             lingshizuixiaoR=lingshiR;
              __GucMouseState   = __RUN90VL;
              while(1)
              {
                if(lingshiL<ucIRCheck[2]){lingshiL=ucIRCheck[2];}
                if(lingshiL>lingshizuixiaoL+300){lingshichengliL=1;}
                if(lingshichengliL)
                {
                  if(lingshiL>ucIRCheck[2]+500){lingshichengliL=0;goto End;}//break;
                }
              }//while循环结束，结束峰值检测
   }
   //.GuiSpeedCtr = __SPEEDUP ;
   __GmLeft.uiPulse =32000;    //9000  //14400        
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =32000;           
   __GmRight.uiPulseCtr=0;
   __GmRight.cState  = __MOTORRUN;
   __GmLeft.cState   = __MOTORRUN;
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);   
   //GuiSpeedCtr=__SPEEDUP;
   // __GmRight.cState = __MOTORSTOP;       
   // __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
    __GmRight.uiPulse =  0;
    __GmLeft.uiPulse = 0;
End:;
}


void mouseTurnleft_135_v(void)
{
   int8 cL = 0, cR = 0;
   __GmLeft.uiPulse =29000;    //9000          
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =29000;           
   __GmRight.uiPulseCtr=0;
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   GW=0;
   time=0;   
   __GucMouseState   = __TURNRIGHT; 
   __GmRight.cState =__MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  =__MOTORRUN;
   GucMouseDir     = (GucMouseDir + 5) % 8;                            /*  方向标记                    */
   
   __GmSPID.sRef=140;
   __GmWPID.sRef=100;
   
   // mouseStop();while(1);  
   jiao=0;
    while(1)
   {
       W=(float)jiao;
       if(W<-135*0.9)   //65800
       {
         break;
       } 
      
   }
//   mouseStop();while(1);  
   GoHead45_fx=0;
   GoHead45_flag=0;
   __GmWPID.sRef=0; 
   __GmSPID.sRef=160;   
   __GucMouseState   = __GOAHEAD;
 
   GuiSpeedCtr=3;
    if(actionsequence[action+1]==STRAIGHT90L) //多个迷宫格时
   {
     //mouseStop();while(1);
     __GmLeft.uiPulse =55000;    //9000    //14400      25400
     __GmLeft.uiPulseCtr=0;
     __GmRight.uiPulse =55000;           
     __GmRight.uiPulseCtr=0;
     __GmRight.cState  = __MOTORRUN;
     __GmLeft.cState   = __MOTORRUN;  
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
     goto End;
   }
   else  //单个迷宫格时
   { 
      __GmLeft.uiPulse =35000;    //9000    //14400      25400
      __GmLeft.uiPulseCtr=0;
      __GmRight.uiPulse =35000;           
      __GmRight.uiPulseCtr=0;
      __GmRight.cState  = __MOTORRUN;
      __GmLeft.cState   = __MOTORRUN;  
      while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
      while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
      //mouseStop();while(1);
      while(1)
      {
           if (cL) 
           {                                                       /*  是否允许检测左边            */
              if  ((__GucDistance[__LEFT]  & 0x01)==0)             /*  左边有支路，跳出程序        */   
              {
                goto End;//直接进入下一个动作
              }
           } 
           else 
           {                                                        /*  左边有墙时开始允许检测左边  */
              if (ucIRCheck[2]>GusDistance_L_Far) 
              {
                  cL = 1; 
              }
          }//20550
          if (cR) 
          {                                                       /*  是否允许检测右边            */
              if ((__GucDistance[__RIGHT]  & 0x01)==0)            /*  右边有支路，跳出程序        */
              {
                goto End;
              }
          }
          else 
          {
              if (ucIRCheck[3]>GusDistance_R_Far)
              {                       
                  cR = 1;
              }
          }
      }
   }  
   //GuiSpeedCtr=__SPEEDUP;
   __GmRight.cState = __MOTORSTOP;       
   __GmLeft.cState  = __MOTORSTOP;
   __GmRight.sSpeed = 0;
   __rightMotorContr();
   __GmLeft.sSpeed = 0;
   __leftMotorContr();
   __GmRight.uiPulseCtr = 0;
   __GmLeft.uiPulseCtr = 0;
   End: 
  {__GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0; 
  }
}

void mouseTurnright_90(void)
{  
   //i++;
//    mouseStop();
//    while(1);
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   GW=0;
   time=0;   
   __GucMouseState   = __TURNRIGHT; 
   __GmRight.cState =__MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  =__MOTORRUN;
   GucMouseDir     = (GucMouseDir + 2) % 8;                            /*  方向标记                    */
   
   __GmSPID.sRef=100;
   __GmWPID.sRef=-80;
    while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>41200*1.8)   
       {
         break;
       } 
      
   }

   __GmWPID.sRef=0;     
   __GucMouseState   = __GOAHEAD;
   GuiSpeedCtr=3;
   __GmLeft.uiPulse =14400;    //9000          
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =14400;           
   __GmRight.uiPulseCtr=0;
  while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
  while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
  //if(i==6)
  //{
    //mouseStop();
    //while(1);
  //}
  // __GucMouseState   = __TURNRIGHT;
   
//   mouseStop();
//    while(1);  
    
   GuiSpeedCtr=__SPEEDUP;
    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
  //mouseStop();
    //while(1);
    //if(i==)
    //{
    //  mouseStop();
    // while(1);
    //}
   
}
void mouseTurnright_90_t(void)
{  
  int cR=0,cL=0;
  __GucMouseState   = __GOAHEAD;
  static int ii=0;ii++;
  
      piancha_r = (leftdis - 40)/10;
   if(leftdis > 120){piancha_r = 0;}
   if(piancha_r > 2){piancha_r = 2;}
   if(piancha_r < -2){piancha_r = -2;}
   if(usepiancha==0) piancha_r=0;
  
     __GmLeft.uiPulse =20000-piancha_r*2886.67;    //9000    //14400      
    __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse=20000-piancha_r*2886.67;           
   __GmRight.uiPulseCtr=0;
     __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   GW=0;
   time=0;   
   __GucMouseState   = __TURNRIGHT; 
   __GmRight.cState =__MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  =__MOTORRUN;
   GucMouseDir     = (GucMouseDir + 2) % 8;                            /*  方向标记                    */
   
   __GmSPID.sRef=160;
   __GmWPID.sRef=-100;
   jiao=0;
    while(1)
   {
       W=(float)jiao;
       if(W>90*0.92)   
       {
         break;
       } 
      
   }
   angle_temp=90;
//   if(ii==2){}
   
   __GmWPID.sRef=0;     
   __GucMouseState   = __GOAHEAD;
   GuiSpeedCtr=3;
   after_turn=1;
   __GucMouseState   =__AFTER;
   __GmLeft.uiPulse =6000;   //60400   
     __GmLeft.uiPulseCtr=0;
     __GmRight.uiPulse =6000;           
     __GmRight.uiPulseCtr=0;
      __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
  while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
  while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
  // if(ii==2){mouseStop(); while(1);} 
   __GucMouseState   = __GOAHEAD;
   
   if(actionsequence[action+1]==STRAIGHT90L) //如果前方有直行的路
   {
     __GmSPID.sRef=280;
     __GmLeft.uiPulse =64000;   //60400   
     __GmLeft.uiPulseCtr=0;
     __GmRight.uiPulse =64000;           
     __GmRight.uiPulseCtr=0;
      __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
  while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
  while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
  goto End;
   }
   else if((actionsequence[action+1]==RV45L)||(actionsequence[action+1]==RV90L))//待改进
   {
      __GucMouseState   = __GOAHEAD;
     
      while(1)
      {
           if (cR) 
           {                                                      
              if  (rightdis>120)
              {
                goto End;//直接进入下一个动作
              }
           } 
           else 
           {                                                     
              if (rightdis<120) 
              {
                  cR = 1; 
              }
          }  
      }
   }
  else if((actionsequence[action+1]==LV45L)||(actionsequence[action+1]==LV90L))//待改进
  {
  
   __GucMouseState   = __GOAHEAD;
     
      while(1)
      {
           if (cL) 
           {                                                      
              if  (leftdis>120)
              {
                goto End;//直接进入下一个动作
              }
           } 
           else 
           {                                                     
              if (leftdis<120) 
              {
                  cL = 1; 
              }
          }  
      }
  }
  
  else if(actionsequence[action+1]==LT45L||actionsequence[action+1]==LT90L)
    {
    __GmLeft.uiPulse =27400;   //  
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =27400;           
   __GmRight.uiPulseCtr=0;
   __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
  while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
  while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
  
   __GucMouseState   = __GOAHEAD;
     
      while(1)
      {
           if (cL) 
           {                                                      
              if  (leftdis>120)
              {
                goto End;//直接进入下一个动作
              }
           } 
           else 
           {                                                     
              if (leftdis<120) 
              {
                  cL = 1; 
              }
          }
      }
  }
  else if(actionsequence[action+1]==RT90L){
    __GmSPID.sRef=240;
     __GmLeft.uiPulse =27400;   //  
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =27400;           
   __GmRight.uiPulseCtr=0;
   __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
  while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
  while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
    __GucMouseState   = __GOAHEAD;
     
      while(1)
      {
           if (cR) 
           {                                                      
              if  (rightdis>120)
              {
                goto End;//直接进入下一个动作
              }
           } 
           else 
           {                                                     
              if (rightdis<120) 
              {
                  cR = 1; 
              }
          }  
      }
    
  }
     __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
  while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
  while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
  
     // mouseStop();
  // while(1);
 
    
   GuiSpeedCtr=__SPEEDUP;
    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
//   mouseStop(); while(1);
End:
  {  __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;}
}

 
void mouseTurnright_90_v(void)
{   __GmSPID.sRef=200;
   uint8 cL = 0,cR=0;
   __GmLeft.uiPulse =13000;    //9000 7500         
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =13000;           
   __GmRight.uiPulseCtr=0;

   GW=0;
   time=0;   
   __GucMouseState   = __TURNRIGHT; 
   __GmRight.cState =__MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  =__MOTORRUN;
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   GucMouseDir     = (GucMouseDir + 2) % 8;                            /*  方向标记                    */
   
   
   __GmWPID.sRef=-120;
   jiao=0;
    while(1)
   {
       W=(float)jiao;
       if(W>90*0.85)   
       {
         break;
       } 
      
   }
    //mouseStop();while(1);
   //mouseStop();while(1);
   angle_temp=90;
   after_turn=1;
   __GmWPID.sRef=0; 
   __GmSPID.sRef=200;
   if((actionsequence[action+1]==LV90L)||(actionsequence[action+1]==LV135L)||(actionsequence[action+1]==LV45L)) 
   {
     __GucMouseState   = __RUN90VL;  
     
      //__GmSPID.sRef=160;
                         __GmLeft.uiPulse =6500;    //9000 7500         
                    __GmLeft.uiPulseCtr=0;
                   __GmRight.uiPulse =6500;           
                   __GmRight.uiPulseCtr=0; 
                   __GmRight.cState =__MOTORRUN;        
                   __GmLeft.cState  =__MOTORRUN;
                   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
                   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   while(1)
      {
           if (cL) 
           {                                                      
              if  (leftdis>120)
              {
               // __GmSPID.sRef=160;
                    __GmLeft.uiPulse =500;    //9000 7500         
                    __GmLeft.uiPulseCtr=0;
                   __GmRight.uiPulse =500;           
                   __GmRight.uiPulseCtr=0; 
                   __GmRight.cState =__MOTORRUN;        
                   __GmLeft.cState  =__MOTORRUN;
                   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
                   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
                   if(leftdis>120)
                   {
                      goto End;//直接进入下一个动作
                   } 
                   else
                     cL=0;
              }
           } 
           else 
           {                                                     
              if (leftdis<120) 
              {
                  cL = 1; 
              }
          }  
      }
 //goto End;
   }
   else if(actionsequence[action+1]==RV45L)
   {
      __GucMouseState = __RUN90VR;
       while(1)
      {
           if (cR) 
           {                                                      
              if  (rightdis>120)
              {
                goto End;//直接进入下一个动作
              }
           } 
           else 
           {                                                     
              if (rightdis<120) 
              {
                  cR = 1; 
              }
          }  
      }
 goto End;
   }
   else
   {
      __GucMouseState = __GOAHEAD_45;
   }
  // GuiSpeedCtr = __SPEEDUP ;
   __GmLeft.uiPulse = 22000;    //9000  //14400        
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse = 22000;           
   __GmRight.uiPulseCtr=0;
     __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
  while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
  while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
  
End: {
    GuiSpeedCtr=__SPEEDUP;
    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
}
}


void mouseTurnright_135_v(void)
{  
   int8 cL = 0, cR = 0;
   __GucMouseState   = __GOAHEAD_45;
    __GmSPID.sRef=140;
   __GmLeft.uiPulse = 12600;    //9000          
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse = 12600;           
   __GmRight.uiPulseCtr=0;
   __GmRight.cState  = __MOTORRUN;
     __GmLeft.cState   = __MOTORRUN;  
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse); 
  
   GW=0;
   time=0;   
   __GucMouseState   = __TURNRIGHT; 
   __GmRight.cState =__MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  =__MOTORRUN;
   GucMouseDir     = (GucMouseDir + 3) % 8;                            /*  方向标记                    */
   
   __GmSPID.sRef=200;
   __GmWPID.sRef=-120;
   jiao=0;
    while(1)
   {
       W=(float)jiao;
       if(W>135*0.8)   
       {
         break;
       } 
      
   }
 //  mouseStop();while(1);
   GoHead45_fx=1;
   GoHead45_flag=0;
   __GmWPID.sRef=0; 
   __GmSPID.sRef=200;
   __GucMouseState   = __GOAHEAD;
   GuiSpeedCtr = 3 ;
    if(actionsequence[action+1]==STRAIGHT90L) //多个迷宫格时
   {
     //mouseStop();while(1);
     __GmLeft.uiPulse =55000;    //9000    //14400      25400
     __GmLeft.uiPulseCtr=0;
     __GmRight.uiPulse =55000;           
     __GmRight.uiPulseCtr=0;
     __GmRight.cState  = __MOTORRUN;
     __GmLeft.cState   = __MOTORRUN;  
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
//     mouseStop();while(1);
     goto End;
   }
    else if((actionsequence[action+1]==RT90L) ||(actionsequence[action+1]==RT45L) )
    {
    
    __GmLeft.uiPulse =25000;    //9000    //14400      25400
     __GmLeft.uiPulseCtr=0;
     __GmRight.uiPulse =25000;           
     __GmRight.uiPulseCtr=0;
     __GmRight.cState  = __MOTORRUN;
     __GmLeft.cState   = __MOTORRUN;  
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
    
    
    __GucMouseState   = __GOAHEAD;
     
 while(1)
      {
           if (cR) 
           {                                                      
              if  (rightdis>120)
              {
                goto End;//直接进入下一个动作
              }
           } 
           else 
           {                                                     
              if (rightdis<120) 
              {
                  cR = 1; 
              }
          }  
      }
 goto End;
    
    
    
    
    }
    
    else if((actionsequence[action+1]==LT90L)||(actionsequence[action+1]==LT45L)||(actionsequence[action+1]==LT135L) )
    {
    __GmLeft.uiPulse =35000;    //9000    //14400      25400
     __GmLeft.uiPulseCtr=0;
     __GmRight.uiPulse =35000;           
     __GmRight.uiPulseCtr=0;
     __GmRight.cState  = __MOTORRUN;
     __GmLeft.cState   = __MOTORRUN;  
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
    
    
     while(1)
      {
           if (cL) 
           {                                                      
              if  (leftdis>120)
              {
                goto End;//直接进入下一个动作
              }
           } 
           else 
           {                                                     
              if (leftdis<120) 
              {
                  cL = 1; 
              }
          }
      }
     
     goto End;
    
    
    }
   
    else {
    
     __GmLeft.uiPulse =55000;    //9000    //14400      25400
     __GmLeft.uiPulseCtr=0;
     __GmRight.uiPulse =55000;           
     __GmRight.uiPulseCtr=0;
     __GmRight.cState  = __MOTORRUN;
     __GmLeft.cState   = __MOTORRUN;  
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
    
    
    }
    
    
    
    
    
   End: 
  {__GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0; 
  }
}
void mouseTurnright_135_t(void)
{  
 
  
   uint8 cL = 0,cR=0;
   static uint8 i=0;
   i++;
     
    __GmSPID.sRef=180;
    piancha_r = (leftdis - 40)/10;
   if(leftdis > 120){piancha_r = 0;}
   if(piancha_r > 2){piancha_r = 2;}
   if(piancha_r < -2){piancha_r = -2;}
   if(usepiancha==0) piancha_r=0;
   
   __GmLeft.uiPulse =32000-piancha_r*2886.67;    //9000          
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =32000-piancha_r*2886.67;           
   __GmRight.uiPulseCtr=0;

   GW=0;
   time=0; 
   __GucMouseState   = __TURNRIGHT; 
   __GmRight.cState =__MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  =__MOTORRUN  ;
 

   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   GucMouseDir     = (GucMouseDir + 3) % 8;                            /*  方向标记                    */
   
   __GmSPID.sRef=140;
   __GmWPID.sRef=-120;
   jiao=0;
    while(1)
   {
       W=(float)jiao;
       if(W>135*0.85)   
       {
         break;
       } 
      
   }
   
  // if(i==2){mouseStop();while(1);}
   //mouseStop();while(1);
   angle_temp=135;
   after_turn=1;
   __GmWPID.sRef=0; 
  __GmSPID.sRef=200;
   if((actionsequence[action+1]==LV90L)||(actionsequence[action+1]==LV135L)||(actionsequence[action+1]==LV45L)) 
   { __GmSPID.sRef=200;
     __GucMouseState   = __GOAHEAD_45;  
     while(1)
      {
           if (cL) 
           {                                                      
              if  (leftdis>120)
              {
                goto End;//直接进入下一个动作
              }
           } 
           else 
           {                                                     
              if (leftdis<120) 
              {
                  cL = 1; 
              }
          }  
      }
     goto End;
   }
   else if((actionsequence[action+1]==RV90L)||(actionsequence[action+1]==RV45L)||(actionsequence[action+1]==R180L))
   {
      __GucMouseState = __RUN90VR;
    while(1)
      {
           if (cR) 
           {                                                      
              if  (rightdis>160)
              {
                goto End;//直接进入下一个动作
              }
           } 
           else 
           {                                                     
              if (rightdis<160) 
              {
                  cR = 1; 
              }
          }  
      }
 goto End;
   }

   
   
   else if(actionsequence[action+1]==STRAIGHT45L)
   {
      __GucMouseState = __GOAHEAD_45;
      
      __GmLeft.uiPulse = 50000;    //9000  //14400        
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse = 50000;           
   __GmRight.uiPulseCtr=0;
     __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
  while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
  while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
      
       goto End;
   }
   
   
   else{
     __GucMouseState = __GOAHEAD_45;
     
   }
   //mouseStop();while(1);
   //GuiSpeedCtr = __SPEEDUP;
   __GmLeft.uiPulse = 22000;    //9000  //14400        
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse = 22000;           
   __GmRight.uiPulseCtr=0;
     __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
  while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
  while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   
    //__GmRight.cState = __MOTORSTOP;       
    //__GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
End:{ GuiSpeedCtr=__SPEEDUP;
   __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;}
}



extern uint8 xiexianyige;
static uint8 jjjjjj=0;




void mouseGoaheadhui_45 (uint8_t  cNBlock)
{
    uint8_t cL = 1, cR = 1, cCoor = 1,cB,L_max, R_max,r_over,l_over;
    
    zuihouyige=0;
    cB=cNBlock;
    __GucMouseState   = __GOAHEAD_45;
    __GmRight.uiPulse = __GmRight.uiPulse + cNBlock *ONEBLOCK_45;
    __GmLeft.uiPulse  = __GmLeft.uiPulse  + cNBlock *ONEBLOCK_45;
    __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
     //__GmRight.cState  = __WAITONESTEP;
     //__GmLeft.cState   = __WAITONESTEP;
   
    GuiSpeedCtr=__SPEEDUP;
    //__GiMaxSpeed=50;
    if(cNBlock > 5)
    {
      __GiMaxSpeed=360;
    }
    else if(cNBlock == 5)
    {
      __GiMaxSpeed=300;
    }
    else if(cNBlock == 4)
    {
      __GiMaxSpeed=260;
    }
    else if(cNBlock == 3)
    {
      __GiMaxSpeed=260;
    }
    else if(cNBlock == 2)
    {
      __GiMaxSpeed=230;
    }
    else  __GiMaxSpeed=180;
    
 
    
  // if(cNBlock > 3){__GiMaxSpeed=sqrt(2*400*((cNBlock-1)*5.7-5.7)+2500);}//a=3.3m/s2
  // else  {__GiMaxSpeed=50;
  //   mouseStop();while(1);}
    while (__GmLeft.cState != __MOTORSTOP) 
    {
       
       
      
        if (__GmLeft.uiPulseCtr >= ONEBLOCK_45) 
        {                          
            __GmLeft.uiPulse    -= ONEBLOCK_45;
            __GmLeft.uiPulseCtr -= ONEBLOCK_45;         
            cNBlock--;
            if(cNBlock==0)
              break;
        }

        
            if(cNBlock < 3)
        {
            if(__GmSPID.sFeedBack>220)
            {              
                __GiMinSpeed=200;
                GuiSpeedCtr= __SPEEDDOWN;
            }          
        }
        
        
        if (cNBlock < 2)   //2//45度时，最后一格走完这么多脉冲后开始检测柱子
        {
         /*  if(__GmSPID.sFeedBack>150)
            {
               __GmSPID.sRef=150;
                 GuiSpeedCtr= __SPEEDDOWN;
            }  
           */
          

          __GmLeft.uiPulse =4000;           
          __GmLeft.uiPulseCtr=0;
          __GmRight.uiPulse =4000; 
          __GmRight.uiPulseCtr=0;   
          while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
          while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
          zuihouyige=1;
         // mouseStop();while(1);
             
        
        /***************A峰值检测*/
         lingshiL=ucIRCheck[2];  //左侧红外采集量赋给一个变量
         lingshiR=ucIRCheck[3];  //右侧红外采集量赋给一个变量
         lingshizuixiaoL=lingshiL;
         lingshizuixiaoR=lingshiR;
          
          
         //如果下一个动作是 斜线  左转45度    左转90度    左转135度  用左侧红外峰值检测
         if( (actionsequence[ action ]==LV45L)||(actionsequence[ action ]==LV90L)||(actionsequence[ action ]==LV135L) )
          { 
              while(1)
              {
    //            xiexianyige=0;
                if(lingshiL<ucIRCheck[2]){lingshiL=ucIRCheck[2];}
                if(lingshiL>lingshizuixiaoL+500){lingshichengliL=1;}
                if(lingshichengliL)
                {
                  if(lingshiL>ucIRCheck[2]+800){lingshichengliL=0;goto End;}//break;
                }
 
              }//while循环结束，结束峰值检测
              //mouseStop();while(1);
          }
         //如果下一个动作是 斜线  右转45度    右转90度    右转135度  用右侧红外峰值检测  
          else if( (actionsequence[ action ]==RV45L)||(actionsequence[ action ]==RV90L)||(actionsequence[ action ]==RV135L))
          {   
              while(1)
             {
               if(lingshiR<ucIRCheck[3]){lingshiR=ucIRCheck[3];}
               if(lingshiR>lingshizuixiaoR+500){lingshichengliR=1;}
               if(lingshichengliR)
               {
                  if(lingshiR>ucIRCheck[3]+800){lingshichengliR=0;goto End;}
               }
             }
          }
          
         }
    }
            
End:   
     
  zuihouyige=0;
 
          
}









void mouseGoahead_L (int8  cNBlock)                                    //连续转弯用
{
    int8 cL = 0, cR = 0, cCoor = 1,cB;
    int16 GuiTpusle_LR_l,GuiTpusle_LR_r;
    GuiSpeedCtr=__SPEEDUP;
    if (__GmLeft.cState) {
        cCoor = 0;
    }
    if(cNBlock==1)
    {
        cL = 1;
        cR = 1;
        //GuiTpusle_LR = 0;    //100
        if(GucFangXiang == GucDirTemp)
        {
           //GucYiBaiBa=1;
           GuiTpusle_LR = 13000;//13000
           GuiTpusle_S  = 0;
           if(GucDirTemp==2 ){GuiTpusle_LR_r=0;W_l=0;}//zuojiezuo
            if(GucDirTemp== 6 ){ GuiTpusle_LR_l=0;W_r=0;}//youjieyou
        }
        else
          
        {
           GuiTpusle_LR = 11000;
           GuiTpusle_S  = 0;
          // mouseStop();while(1);
           if(GucDirTemp==2 ){GuiTpusle_LR_r=0;W_l=0;}//zuojieyou
            if(GucDirTemp== 6 ){ GuiTpusle_LR_l=0;W_r=0;}//youjiezuo
           
        }

        __GiMaxSpeed = 120;//120
    }
    else{
        GuiTpusle_LR = 0;
        GuiTpusle_S  = 0;
        __GiMaxSpeed = 150;
    }
    GucFangXiang = GucDirTemp;
    if(((GmcMouse.cX==7)&&(GmcMouse.cY==7))|| 
         ((GmcMouse.cX==8)&&(GmcMouse.cY==8))||
         ((GmcMouse.cX==7)&&(GmcMouse.cY==8))||
           ((GmcMouse.cX==8)&&(GmcMouse.cY==7))){
       cL = 0;
       cR = 0;
       GuiTpusle_LR = 0;
       GuiTpusle_S  = 0;
    }
    if((__GucMouseState==__TURNRIGHT)||(__GucMouseState==__TURNLEFT))
    {
        __GmLeft.uiPulseCtr = 30000;       //1182(34mm)
        __GmRight.uiPulseCtr = 30000;

    }
    
    cB=cNBlock;
    __GucMouseState   = __GOAHEAD;
    __GmRight.uiPulse = __GmRight.uiPulse + cNBlock * ONEBLOCK ;
    __GmLeft.uiPulse  = __GmLeft.uiPulse  + cNBlock * ONEBLOCK ;
    __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
    
      if(cNBlock > 5)
    {
        __GiMaxSpeed=200;
    }
    else if(cNBlock == 5)
    {
        __GiMaxSpeed=200;
    }
    else if(cNBlock == 4)
    {
        __GiMaxSpeed=200;
    }
    else if(cNBlock == 3)
    {
        __GiMaxSpeed=200;
    }
    else if(cNBlock == 2)
    {
        __GiMaxSpeed=200;
    }
    else;
    
    while (__GmLeft.cState != __MOTORSTOP)
    {
       
        if (__GmLeft.uiPulseCtr >= ONEBLOCK) 
        {                          /*  判断是否走完一格            */
            __GmLeft.uiPulse    -= ONEBLOCK;
            __GmLeft.uiPulseCtr -= ONEBLOCK;
            if (cCoor) {
                cNBlock--;
                if(cNBlock==0)
                   goto End;
                if(cNBlock<cB-1)//给回速一个时间
                  GuiSpeedCtr=__SPEEDUP;
            } else {
                cCoor = 1;
            }
        }
        
        if (__GmRight.uiPulseCtr >= ONEBLOCK) {                     /*  判断是否走完一格            */
            __GmRight.uiPulse    -= ONEBLOCK;
            __GmRight.uiPulseCtr -= ONEBLOCK;
        }
        if (cNBlock < 2) {
          if(__GmSPID.sFeedBack>120){
              GuiSpeedCtr= 3;
             __GmSPID.sRef=120;
          }
          if (cL) 
          {                                                       /*  是否允许检测左边            */
            if (ucIRCheck[2]<GusDistance_L_Far)             /*  左边有支路，跳出程序        */
            {                 
                __GmRight.uiPulse = __GmRight.uiPulseCtr  + 19500 - GuiTpusle_LR - GuiTpusle_LR_l;    //3094(89mm)
                __GmLeft.uiPulse  = __GmLeft.uiPulseCtr   + 19500 - GuiTpusle_LR - GuiTpusle_LR_l;
                while (ucIRCheck[2]<GusDistance_L_Far) 
                {
                    
                    if ((__GmLeft.uiPulseCtr + 100) > __GmLeft.uiPulse) 
                    {
                        goto End;
                    }
                }
            }
            } else {                                                    /*  左边有墙时开始允许检测左边  */
                if (ucIRCheck[2]>GusDistance_L_Far) {
                    cL = 1;
                }
            }
         if (cR) 
            {                                                       /*  是否允许检测右边            */
            if (ucIRCheck[3]<GusDistance_R_Far)               /*  右边有支路，跳出程序        */
            {                
                __GmRight.uiPulse  = __GmRight.uiPulseCtr + 18800- GuiTpusle_LR - GuiTpusle_LR_r;
                __GmLeft.uiPulse   = __GmLeft.uiPulseCtr  + 18800- GuiTpusle_LR - GuiTpusle_LR_r;
                while (ucIRCheck[3]<GusDistance_R_Far) 
                {
                    
                    if ((__GmLeft.uiPulseCtr + 100) > __GmLeft.uiPulse) 
                    {
                        goto End;
                    }
                }
            }
            } else {
                if (ucIRCheck[3]>GusDistance_R_Far) {                   /*  右边有墙时开始允许检测右边  */
                    cR = 1;
                }
            }
        }
   }
    /*
     *  设定运行任务，让电脑鼠走到支路的中心位置
     */
End:     ;//GuiSpeedCtr= 3;
         //__GmSPID.sRef=23;
}



void mouseStop(void)
{   
  __GmSPID.sRef=0;
  __GmWPID.sRef=0;
  GuiSpeedCtr=5;  
}
/****************************************************************************************************
** Function name:       mouseTurnright
** Descriptions:        右转
** input parameters:    无
** output parameters:   无
** Returned value:      无         按步数转弯
*********************************************************************************************************/
static uint8 after_turnright;

static uint8 aaaaa;

static uint8 lwt1;

float bc_r;


static int sssss=0;
void mouseTurnright(void)
{  
static int ir=0;   
//if(ir==0){ mouseStop(); while(1);}
   //while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
  // while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   GW=0;
   time=0;   
   __GucMouseState   = __TURNRIGHT; 
   __GmRight.cState =__MOTORRUN;       
   __GmLeft.cState  =__MOTORRUN;
   GucMouseDir     = (GucMouseDir + 2) % 8;                            /*  方向标记                    */
  
  
   
 piancha_r = (leftdis - 95)/10;
   if(leftdis > 155){piancha_r = 0;}
   if(piancha_r > 2){piancha_r = 2;}
   if(piancha_r < -2){piancha_r = -2;}
   
   __GmSPID.sRef=145;
   __GmWPID.sRef=-95;//-90//-85
    while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>95000)//81400 //82400 84400 
       {
         break;
       } 
      
   }
   ir++;
   //if(ir==1){ mouseStop(); while(1);}
     //if(ir==2){ mouseStop(); while(1);}  
   __GmWPID.sRef=0;     
  __GucMouseState   = __GOAHEAD;
   GuiSpeedCtr=3;
   
   __GmLeft.uiPulse =17000- piancha_r*2886.67;      //8000 //9000   9500 8000 7000 6800 6200 5800 5400 5200//4800
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =17000- piancha_r*2886.67;           
   __GmRight.uiPulseCtr=0;
   
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   
   __GucMouseState   = __TURNRIGHT;
   
  //if(ir==2){ mouseStop(); while(1);} 
   //if(ir==1){ mouseStop(); while(1);}
   // mouseStop();      while(1);           
     
     
   GuiSpeedCtr=__SPEEDUP;
    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;

}

static int lllllw=0;
void mouseTurnrightyuandi(void)
{
  __GucMouseState   = __TURNRIGHT; 
   __GmRight.cState =__MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  =__MOTORRUN;
   GucMouseDir     = (GucMouseDir + 2) % 8;                            /*  方向标记                    */
   GuiSpeedCtr=3;
__GmSPID.sRef=30;
  __GmLeft.uiPulse =8500;           //7700
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =8500;            
   __GmRight.uiPulseCtr=0;
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
  __GmSPID.sRef=0;
     while(1)
   {
     __GmWPID.sRef=-4; 
       if(jiao>90)      //315000
       {
         break;
       }                        
   }
   
   after_turnright=1;
    __GmWPID.sRef=0; 
   
 
   /* qidianclock=0;
       lllllw=0;
  while(1)
   {

   lllllw++;if(lllllw==3000){lllllw=0;}
     if(lllllw==1)
     {
      if(rightdis<140)
               {
                GsTpusle_T = -0.6*(rightdis - 45);GPIO_ResetBits(GPIOB,GPIO_Pin_12);
                
            
                 
                 __PIDContr();
               }
              else  if(leftdis<120)
               {
                
                  GsTpusle_T = 0.5*(leftdis - 45);__PIDContr();GPIO_SetBits(GPIOB,GPIO_Pin_12);
               }
               else if((rightdis>140)&&(leftdis>120)){GsTpusle_T=0;__PIDContr();GPIO_ResetBits(GPIOB,GPIO_Pin_12);}
               
     
     if(qidianclock==500){qidianclock=0;break;}
     
     }
   
   
   }*/
   
   
 __GmSPID.sRef=50;
  
   __GucMouseState   = __GOAHEAD;
   
   //piancha_r=0;
  // GuiSpeedCtr=3;
   __GmLeft.uiPulse =16000;    //9000          
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =16000;           
   __GmRight.uiPulseCtr=0;
  while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
  while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   //Continuous=1;
  zhixianjuli=0;
  bianmaqijiaodu = bianmaqijiaodu + 1.571;

   __GucMouseState   = __TURNRIGHT;
   
//mouseStop();while(1);
    
   GuiSpeedCtr=__SPEEDUP;
    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
}


void mouseRT90(void)
{  
 
   __GmRight.cState =__MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  =__MOTORRUN;
   
  __GmLeft.uiPulse =9950; //9800          
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =9950; //9800
    __GmRight.uiPulseCtr=0;   
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   GW=0;
   time=0;   
   __GucMouseState   = __TURNRIGHT; 
   __GmRight.cState =__MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  =__MOTORRUN;
   GucMouseDir     = (GucMouseDir + 2) % 8;                            /*  方向标记                    */
   start_speedr=1;//滴答定时器中语句响应
   piancha_r = (leftdis - 66)/10;
   if(leftdis > 120){piancha_r = 0;}
   if(piancha_r > 2){piancha_r = 2;}
   if(piancha_r < -2){piancha_r = -2;}
   __GmSPID.sRef=50;
   while(1)
   {
     __GmWPID.sRef=-speed_r[start_flag]-0.6;//给每时刻的线速度角速度赋值，进行弧线转弯-0.6
     if(end_speedr == 1){end_speedr=0;break;} 
   }
   after_turnright=1;
   
 
   __GmWPID.sRef=0; 
   __GucMouseState   = __GOAHEAD;
  
   
   lingshiL=ucIRCheck[2];
   lingshiR=ucIRCheck[3];
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulseCtr=0;
   while(1)
  {
   __GmLeft.uiPulse =38000 - piancha_r/0.00052;           //27000//33000
   
   __GmRight.uiPulse =38000 - piancha_r/0.00052;            
   
   if(((__GmRight.uiPulseCtr+200 ) >= __GmRight.uiPulse)&&((__GmLeft.uiPulseCtr+200 ) >= __GmLeft.uiPulse)) 
   {
     break;
   }
    if(lingshiL<ucIRCheck[2]){lingshiL=ucIRCheck[2];}
    if(lingshiR<ucIRCheck[3]){lingshiR=ucIRCheck[3];}
    if((lingshiL>=1300)&&(ucIRCheck[2]<=800)){break;}
    if((lingshiR>=1300)&&(ucIRCheck[3]<=800)){break;;}
            
  }
   

  zhixianjuli=0;
  bianmaqijiaodu = bianmaqijiaodu + 1.571;

   __GucMouseState   = __TURNRIGHT;
   
 
    
   GuiSpeedCtr=__SPEEDUP;
    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;

  
}
void mouseTurnright_180(void)
{  
   //i++;
    int8 cL = 0, cR = 0;
    __GucMouseState   = __SPURT45;
   piancha_r = (leftdis - 40)/10;
   if(leftdis > 120){piancha_r = 0;}
   if(piancha_r > 2){piancha_r = 2;}
   if(piancha_r < -2){piancha_r = -2;}
    
    
    
   __GmLeft.uiPulse =15000;    //9000    //14400      
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse= 15000;           
   __GmRight.uiPulseCtr=0;
 
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   GW=0;
   time=0;   
   __GucMouseState   = __TURNRIGHT; 
   __GmRight.cState =__MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  =__MOTORRUN;
   GucMouseDir     = (GucMouseDir + 4) % 8;                            /*  方向标记                    */
   
   __GmSPID.sRef=200;
   __GmWPID.sRef=-120;
   jiao=0;
    while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>76400)//81400 //82400 84400 
       {
         break;
       } 
      
   }
 __GmWPID.sRef=0;   
  
   __GucMouseState   = __SPURT45;
 //  GuiSpeedCtr=3;
   __GmLeft.uiPulse =22000- piancha_r*2886.67;    //9000          
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =22000- piancha_r*2886.67;           
   __GmRight.uiPulseCtr=0;
  while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
  while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
__GucMouseState   = __TURNRIGHT; 
   __GmRight.cState =__MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  =__MOTORRUN;
   
   __GmWPID.sRef=-120;
   W=0;
   GW=0;
   while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>90800)//81400 //82400 84400 
       {
         break;
       } 
      
   }
   
   // mouseStop();while(1);
  //  mouseStop();while(1);

 GoHead45_flag=0;
   __GmWPID.sRef=0; 
   __GmSPID.sRef=200;   
  __GucMouseState   = __SPURT45;
 
   GuiSpeedCtr=3;
   if(actionsequence[action+1]==STRAIGHT90L) //多个迷宫格时
   {  __GucMouseState   = __SPURT45;
 
    
 
     __GmLeft.uiPulse =35000;    //9000    //14400      25400
     __GmLeft.uiPulseCtr=0;
     __GmRight.uiPulse =35000;           
     __GmRight.uiPulseCtr=0;
     __GmRight.cState  = __MOTORRUN;
     __GmLeft.cState   = __MOTORRUN;  
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
     goto End;
   }
   else  //单个迷宫格时
   { 
      __GmLeft.uiPulse =35000;    //9000    //14400      25400
      __GmLeft.uiPulseCtr=0;
      __GmRight.uiPulse =35000;           
      __GmRight.uiPulseCtr=0;
      __GmRight.cState  = __MOTORRUN;
      __GmLeft.cState   = __MOTORRUN;  
      while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
      while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
      //mouseStop();while(1);
      while(1)
      {
           if (cL) 
           {                                                       /*  是否允许检测左边            */
              if  ((__GucDistance[__LEFT]  & 0x01)==0)             /*  左边有支路，跳出程序        */   
              {
                goto End;//直接进入下一个动作
              }
           } 
           else 
           {                                                        /*  左边有墙时开始允许检测左边  */
              if (ucIRCheck[2]>GusDistance_L_Far) 
              {
                  cL = 1; 
              }
          }//20550
          if (cR) 
          {                                                       /*  是否允许检测右边            */
              if ((__GucDistance[__RIGHT]  & 0x01)==0)            /*  右边有支路，跳出程序        */
              {
                goto End;
              }
          }
          else 
          {
              if (ucIRCheck[3]>GusDistance_R_Far)
              {                       
                  cR = 1;
              }
          }
      }
   }  
   //GuiSpeedCtr=__SPEEDUP;
   __GmRight.cState = __MOTORSTOP;       
   __GmLeft.cState  = __MOTORSTOP;
   __GmRight.sSpeed = 0;
   __rightMotorContr();
   __GmLeft.sSpeed = 0;
   __leftMotorContr();
   __GmRight.uiPulseCtr = 0;
   __GmLeft.uiPulseCtr = 0;
   End: 
  {__GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0; 
  }
}

void mouseTurnright_45(void)

{  
 // j++;
  // mouseStop();
  // while(1);
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   GW=0;  
   __GucMouseState   = __TURNRIGHT_45; 
   __GmRight.cState =__MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  =__MOTORRUN;
   GucMouseDir     = (GucMouseDir + 1) % 8;                           /*  方向标记                    */
   
   __GmSPID.sRef=100;
   __GmWPID.sRef=-80;
    while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>20000)   
       {
         break;
       } 
      
   }
  mouseStop();   while(1);
   __GmWPID.sRef=0;     
 
   __GucMouseState   = __GOAHEAD;
   __GmLeft.uiPulse =8000;    //4200          
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =8000;           
   __GmRight.uiPulseCtr=0;
  while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
  while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
  
  //mouseStop();
  //while(1);
  
  
 /* flag_right++;                                                                 //转弯调试
  if(flag_right==2)
  {
    // __delay(9000000);
     mouseStop();  
     while(1);
  }   */ 
     
  

  // __GucMouseState   = __TURNRIGHT;
   GuiSpeedCtr=__SPEEDUP;
    //__GmRight.cState = __MOTORSTOP;       
    //__GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0; 
    
  //  if(j==2)
  //  {
   //   mouseStop();
   //   while(1); 
  //  }
    
}


void mouseTurnright_45_v(void)
{   __GmSPID.sRef=200;  
   int8 cL = 0, cR = 0;
   __GmLeft.uiPulse =23500;           
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =23500;           
   __GmRight.uiPulseCtr=0;
     __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   time=0;
   
   //__GucMouseState   = __TURNRIGHT; 
   __GucMouseState   = __TURNRIGHT_45; 
   __GmRight.cState = __MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  = __MOTORRUN;
   GucMouseDir     = (GucMouseDir + 1) % 8;                             /*  方向标记                    */
             
            ///120
   __GmWPID.sRef=-120;//70
   GW=0;
   jiao=0;
    while(1)
   {
       W=(float)jiao;
       if(W>45*0.6)   
       {
         break;
       } 
      
   }
  // mouseStop(); while(1);
 // mouseStop(); while(1);
   __GmSPID.sRef=230; 
    GoHead45_flag=0;
     GoHead45_fx=1;

   __GmWPID.sRef=0;     
   __GucMouseState   = __GOAHEAD;
   GuiSpeedCtr=3;
   
    //转弯后要分两种情况：1只有一个迷宫格，但并没有进入mousegoahead
   //                   2有多个迷宫格，需要进入mousegoahead
   if(actionsequence[action+1]==STRAIGHT90L) //多个迷宫格时
   {
     //mouseStop();while(1);
     __GmLeft.uiPulse = 66000;    //9000    //14400      25400
     __GmLeft.uiPulseCtr=0;
     __GmRight.uiPulse = 66000;           
     __GmRight.uiPulseCtr=0;
     __GmRight.cState  = __MOTORRUN;
     __GmLeft.cState   = __MOTORRUN;  
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
     goto End;
   }
   else if((actionsequence[action+1]==RT90L)||(actionsequence[action+1]==RT135L)||(actionsequence[action+1]==RT45L)) //单个迷宫格时
   { 
        __GucMouseState   = __GOAHEAD;
          __GmLeft.uiPulse = 26000;    //9000    //14400      25400
     __GmLeft.uiPulseCtr=0;
     __GmRight.uiPulse = 26000;           
     __GmRight.uiPulseCtr=0;
     __GmRight.cState  = __MOTORRUN;
     __GmLeft.cState   = __MOTORRUN;  
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
      while(1)
      {
           if (cR) 
           {                                                      
              if  (rightdis>200)
              {
                goto End;//直接进入下一个动作
              }
           } 
           else 
           {                                                     
              if (rightdis<200) 
              {
                  cR = 1; 
              }
          }  
      }

      }
     else if((actionsequence[action+1]==LV90L)||(actionsequence[action+1]==LV45L)){
        __GucMouseState   = __GOAHEAD;
     
      while(1)
      {
           if (cL) 
           {                                                      
              if  (leftdis>120)
              {
                goto End;//直接进入下一个动作
              }
           } 
           else 
           {                                                     
              if (leftdis<120) 
              {
                  cL = 1; 
              }
          }
      }
     
     
     
     }
      
   //GuiSpeedCtr=__SPEEDUP;
   __GmRight.cState = __MOTORSTOP;       
   __GmLeft.cState  = __MOTORSTOP;
   __GmRight.sSpeed = 0;
   __rightMotorContr();
   __GmLeft.sSpeed = 0;
   __leftMotorContr();
   __GmRight.uiPulseCtr = 0;
   __GmLeft.uiPulseCtr = 0;
   End: 
   {__GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0; 
   }
}




void mouseTurnright_45_t(void)
{  
  static uint8 i;i++;
  int8 cL = 0;
    
     
    piancha_r = (leftdis - 40)/10;
   if(leftdis > 120){piancha_r = 0;}
   if(piancha_r > 2){piancha_r = 2;}
   if(piancha_r < -2){piancha_r = -2;}
   if(usepiancha==0) piancha_r=0;
     
  __GmLeft.uiPulse =1000;//-piancha_r*2886.67;           
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =1000;///-piancha_r*2886.67;           
   __GmRight.uiPulseCtr=0;
     __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
 
   GW=0;  
   //__GucMouseState   = __TURNRIGHT;
   __GucMouseState   = __TURNRIGHT_45; 
   __GmRight.cState =__MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  =__MOTORRUN;
   GucMouseDir     = (GucMouseDir + 1) % 8;                           /*  方向标记                    */
    __GmSPID.sRef=160;

   __GmWPID.sRef=-100;
   jiao=0;
    while(1)
   {    
       W=(float)jiao;  
       if(W>45*0.8)   
       {
         break;
       } 
   }
    __GucMouseState = __GOAHEAD_45;
   angle_temp=45;
   after_turn=1;
   __GmWPID.sRef=0; 
   __GmSPID.sRef=220;
   
   if(actionsequence[action+1]==STRAIGHT45L){
   __GucMouseState = __GOAHEAD_45;
       
   __GmLeft.uiPulse =20000;           
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =20000;           
   __GmRight.uiPulseCtr=0;
     __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   
   goto End;
   
   }
      else  if((actionsequence[action+1]==LV45L)||(actionsequence[action+1]==LV90L)||(actionsequence[action+1]==LV135L))
   { 
             lingshiL=ucIRCheck[2];  //左侧红外采集量赋给一个变量
             lingshiR=ucIRCheck[3];  //右侧红外采集量赋给一个变量
             lingshizuixiaoL=lingshiL;
             lingshizuixiaoR=lingshiR;
              __GucMouseState   = __RUN90VL;
               __GmLeft.uiPulse =5000;           
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =5000;           
   __GmRight.uiPulseCtr=0;
     __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   // mouseStop();while(1);
              while(1)
              {
                if(lingshiL<ucIRCheck[2]){lingshiL=ucIRCheck[2];}
                if(lingshiL>lingshizuixiaoL+500){lingshichengliL=1;}
                if(lingshichengliL)
                {
                  if(lingshiL>ucIRCheck[2]+1000){lingshichengliL=0; goto End;}//break;
                }
              }//while循环结束，结束峰值检测
   }
   else
   {
      __GucMouseState = __GOAHEAD_45;
   }
    
   if(i==0){
   mouseStop();while(1);
   }
    //GuiSpeedCtr=__SPEEDUP;
    //__GmRight.cState = __MOTORSTOP;       
    //__GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
End: {//GuiSpeedCtr=__SPEEDUP;
    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;}
}


void mouseTurnleft_45(void)
{
   
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   time=0;
   __GucMouseState   = __TURNLEFT_45;   
   __GmRight.cState = __MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  = __MOTORRUN;
   GucMouseDir     = (GucMouseDir + 7) % 8;                             /*  方向标记                    */
             
   __GmSPID.sRef=100;           ///120
   __GmWPID.sRef=80;//70
   GW=0;
    while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>17500*1.5)   
       {
         break;
       } 
      
   }

   __GmWPID.sRef=0; 
   __GucMouseState   = __GOAHEAD;

   __GmLeft.uiPulse =8000;           
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =8000;           
   __GmRight.uiPulseCtr=0;
     __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   GuiSpeedCtr=__SPEEDUP;

    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
}


void mouseTurnleft_45_v(void)
{
   int8 cL = 0, cR = 0;
   static int i=0;
   i++;
   __GmSPID.sRef=160;  
   
  //  __GucMouseState   = __GOAHEAD_45;
   __GmLeft.uiPulse = 16000;           
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse = 16000;           
   __GmRight.uiPulseCtr=0;
     __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   time=0;
   
  // __GucMouseState   = __TURNLEFT_45;mouseStop();while(1);
// __GucMouseState   = __TURNLEFT   
   __GmRight.cState = __MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  = __MOTORRUN;
   GucMouseDir     = (GucMouseDir + 7) % 8;                             /*  方向标记                    */
             
         ///120
   __GmSPID.sRef=160;  
   __GmWPID.sRef=100;//70
   GW=0;
   jiao=0;
    while(1)
   {
       W=(float)jiao*1.06;
       if(W<-45*0.5)   
       {
         break;
       } 
      
   }

   angle_temp=-45;
   after_turn=1;
   GoHead45_flag=0; 
   GoHead45_fx=0;
 __GmSPID.sRef=230;
  __GmWPID.sRef=0;     
  
  __GucMouseState   =__AFTER;
   __GmLeft.uiPulse =6000;    //9000    //14400      25400
     __GmLeft.uiPulseCtr=0;
     __GmRight.uiPulse = 6000;           
     __GmRight.uiPulseCtr=0;
     __GmRight.cState  = __MOTORRUN;
     __GmLeft.cState   = __MOTORRUN;  
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
     __GucMouseState   = __GOAHEAD;
 
   if(actionsequence[action+1]==STRAIGHT90L) //多个迷宫格时
   {
     __GmLeft.uiPulse =54000;    //9000    //14400      25400
     __GmLeft.uiPulseCtr=0;
     __GmRight.uiPulse =54000;           
     __GmRight.uiPulseCtr=0;
     __GmRight.cState  = __MOTORRUN;
     __GmLeft.cState   = __MOTORRUN;  
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
     goto End;
   }
   else if(actionsequence[action+1]==RT135L||actionsequence[action+1]==RT90L)
   {
    
     __GmLeft.uiPulse =19000;    //9000    //14400      25400
     __GmLeft.uiPulseCtr=0;
     __GmRight.uiPulse =19000;           
     __GmRight.uiPulseCtr=0;
     __GmRight.cState  = __MOTORRUN;
     __GmLeft.cState   = __MOTORRUN;  
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
    
 while(1)
      {
           if (cR)
           {                                                      
              if  (rightdis>120)
              {
                goto End;//直接进入下一个动作
              }
           } 
           else 
           {                                                     
              if (rightdis<120) 
              {
                  cR = 1; 
              }
          }  
      }
     goto End;
      } //单个迷宫格时
    
   
   else if(actionsequence[action+1]==LT135L||actionsequence[action+1]==LT90L){
     __GmLeft.uiPulse =25000;    //9000    //14400      25400
     __GmLeft.uiPulseCtr=0;
     __GmRight.uiPulse =25000;           
     __GmRight.uiPulseCtr=0;
     __GmRight.cState  = __MOTORRUN;
     __GmLeft.cState   = __MOTORRUN;  
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
     while(1)
      {
           if (cL) 
           {                                                      
              if  (leftdis>160)
              {
                goto End;//直接进入下一个动作
              }
           } 
           else 
           {                                                     
              if (leftdis<160) 
              {
                  cL = 1; 
              }
          }
      }
     
     goto End;
      }
   //GuiSpeedCtr=__SPEEDUP;
   else{
   
   __GmLeft.uiPulse =25000;    //9000    //14400      25400
     __GmLeft.uiPulseCtr=0;
     __GmRight.uiPulse =25000;           
     __GmRight.uiPulseCtr=0;
     __GmRight.cState  = __MOTORRUN;
     __GmLeft.cState   = __MOTORRUN;  
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   
   
   }
   __GmRight.cState = __MOTORSTOP;       
   __GmLeft.cState  = __MOTORSTOP;
   __GmRight.sSpeed = 0;
   __rightMotorContr();
   __GmLeft.sSpeed = 0;
   __leftMotorContr();
   __GmRight.uiPulseCtr = 0;
   __GmLeft.uiPulseCtr = 0;
   End: 
   {__GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0; 
   }
}

void mouseTurnleft_45_t(void)
{
  
    static int i=0;i++;
    int8 zjbl=100;  
    int8 cR = 0;
 
      __GmSPID.sRef=140;
      
      
    piancha_l = (rightdis - 55)/10;
   if(rightdis>125){piancha_l = 0;}//左转右边没墙
   if(piancha_l > 2){piancha_l = 2;}
    if(piancha_l < -2){piancha_l = -2;}
   if(usepiancha==0)piancha_l = 0;
      
      
    __GmLeft.uiPulse = 3500-piancha_l*2886.67;   //5000     
    __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse = 3500-piancha_l*2886.67;           
   __GmRight.uiPulseCtr=0;
     __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);  
   GW=0;  
   //__GucMouseState   = __TURNLEFT;
   __GucMouseState   = __TURNLEFT_45;
   __GmRight.cState =__MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  =__MOTORRUN;
   GucMouseDir     = (GucMouseDir + 7) % 8;                           /*  方向标记                    */
   
    
   __GmWPID.sRef=80;
   jiao=0;
    while(1)
   {
       W=(float)jiao;
       if(W<-45*0.78)   
       {
         break;
       } 
      
   } 
  // mouseStop();while(1);
   angle_temp=-45;
   after_turn=1;
   __GmWPID.sRef=0; 
   __GmSPID.sRef=240;
   if(actionsequence[action+1]==STRAIGHT45L){
   
      __GucMouseState = __GOAHEAD_45;
   
    __GmLeft.uiPulse =20000;    //9000  //14400        
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =20000;           
   __GmRight.uiPulseCtr=0;
     __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
  while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
  while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   
   goto End;
   
   }
   
  else if((actionsequence[action+1]==RV90L)||(actionsequence[action+1]==RV135L)) 
   {
     __GucMouseState   = __RUN90VR;  
     while(1)
      {
           if (cR) 
           {                                                      
              if  (rightdis>120)
              {
                goto End;//直接进入下一个动作
              }
           } 
           else 
           {                                                     
              if (rightdis<120) 
              {
                  cR = 1; 
              }
          }  
      }
   }
   else if(actionsequence[action+1]==RV45L)
   {
      __GucMouseState = __RUN90VR;
      __GmLeft.uiPulse =35000;    //9000  //14400        
      __GmLeft.uiPulseCtr=0;
      __GmRight.uiPulse =35000;           
      __GmRight.uiPulseCtr=0;
      __GmRight.cState  = __MOTORRUN;
      __GmLeft.cState   = __MOTORRUN;
      while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
      while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
      goto End;
   }
   else
   {
      __GucMouseState = __GOAHEAD_45;
   }
   //GuiSpeedCtr = __SPEEDUP ;
   __GmLeft.uiPulse =16000;    //9000  //14400        
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =16000;           
   __GmRight.uiPulseCtr=0;
     __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
  while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
  while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
//   if(i==1){
//   mouseStop();while(1);
//   }
    //GuiSpeedCtr=__SPEEDUP;
    //__GmRight.cState = __MOTORSTOP;       
    //__GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
End:
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
  
  
  ;
  // if(i==1){
//   mouseStop();while(1);
   
}



/*********************************************************************************************************
** Function name:       mouseTurnleft
** Descriptions:        左转
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/

 float bc_l;

static uint8 lwt;
void mouseTurnleft(void)
{ 
  static int il=0;
  //if(il==0){ mouseStop(); while(1);}
  
  piancha_l = (rightdis - 90)/10;
   if(rightdis>145){piancha_l = 0;}//左转右边没墙
   if(piancha_l > 2){piancha_l = 2;}
    if(piancha_l < -2){piancha_l = -2;}

   //while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   //while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   time=0;   
   __GucMouseState   = __TURNLEFT; 
   __GmRight.cState = __MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  = __MOTORRUN;
   GucMouseDir     = (GucMouseDir + 6) % 8;                            /*  方向标记                    */
             
   __GmSPID.sRef=145;           ///120//150
   __GmWPID.sRef=95;//70//90
   GW=0;
    while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>86500) //81200//  80200 79200 78700 78900 //79000 79500 80000
       {
         break;
       } 
      
   }
   il++;

 // if(il==1){ mouseStop(); while(1);}     
   //if(il==2){mouseStop();  while(1);}  
   __GmWPID.sRef=0; 
   __GucMouseState   = __GOAHEAD;
 
   GuiSpeedCtr=3;
   __GmLeft.uiPulse =21000- piancha_l*2886.67;     //10000 //5500 5000   
   __GmLeft.uiPulseCtr= 0;
   __GmRight.uiPulse =21000- piancha_l*2886.67;        //衔接脉冲    
   __GmRight.uiPulseCtr=0;
  
             
              
 
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   

   __GucMouseState   = __TURNLEFT;
   
    //if(il==1){mouseStop();  while(1);  }        
   //if(il==2){mouseStop();  while(1);} 
   
   
   GuiSpeedCtr=__SPEEDUP;
    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;

   
}

void mouseTurnleftyuandi(void)
{
   time=0;   
   __GucMouseState   = __TURNLEFT; 
   __GmRight.cState = __MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  = __MOTORRUN;
   GucMouseDir     = (GucMouseDir + 6) % 8;                            /*  方向标记                    */
             

   __GmWPID.sRef=80;//70
   GW=0;
    while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>88000)   
       {
         break;
       } 
      
   }
   __GmWPID.sRef=0; 
  //__GmWPID.sRef=0; 
    __GucMouseState   = __GOAHEAD;
  __GmSPID.sRef=80;
   GuiSpeedCtr=3;
   __GmLeft.uiPulse =2000;           
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =2000;           
   __GmRight.uiPulseCtr=0;
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
    

   GuiSpeedCtr=__SPEEDUP;
    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;

}

void mouseLT90(void)
{
  static int j=0;
   j++;
   
    __GmRight.cState =__MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  =__MOTORRUN;
  
    __GmLeft.uiPulse =10700;           
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =10700; 
    __GmRight.uiPulseCtr=0;   
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   __GucMouseState   = __TURNLEFT; 
   __GmRight.cState = __MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  = __MOTORRUN;
   GucMouseDir     = (GucMouseDir + 6) % 8;                             /*  方向标记                    */
             
   start_speedl=1;//滴答定时器中语句响应
   piancha_l = (rightdis - 66)/10;
   if(rightdis>120){piancha_l = 0;}//左转右边没墙
   if(piancha_l > 2){piancha_l = 2;}
    if(piancha_l < -2){piancha_l = -2;}
   __GmSPID.sRef=50;
   while(1)
   {
     __GmWPID.sRef=speed_r[start_flag1]+0.7;//给每时刻的线速度角速度赋值，进行弧线转弯+0.7
     if(end_speedl == 1){end_speedl=0;break;} 
   }
 
   after_turnleft=1;

   
  //mouseStop(); while(1);
   __GmWPID.sRef=0; 
   __GucMouseState   = __GOAHEAD;
 //piancha_l=0;
   
   GuiSpeedCtr=3;
   //mouseStop();while(1);
   
   lingshiL=ucIRCheck[2];
   lingshiR=ucIRCheck[3];
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulseCtr=0;
   while(1)
  {
   __GmLeft.uiPulse =27000 - piancha_l/0.00052;           //7700
   
   __GmRight.uiPulse =27000 - piancha_l/0.00052;            
   
   if(((__GmRight.uiPulseCtr+200 ) >= __GmRight.uiPulse)&&((__GmLeft.uiPulseCtr+200 ) >= __GmLeft.uiPulse)) 
   {
     break;
   }
    if(lingshiL<ucIRCheck[2]){lingshiL=ucIRCheck[2];}
    if(lingshiR<ucIRCheck[3]){lingshiR=ucIRCheck[3];}
    if((lingshiL>=1300)&&(ucIRCheck[2]<=350)){break;}
    if((lingshiR>=1300)&&(ucIRCheck[3]<=350)){break;;}
            
  }
  //mouseStop();while(1);
 

   zhixianjuli=0;
   bianmaqijiaodu = bianmaqijiaodu - 1.571;
  
   __GucMouseState   = __TURNLEFT;
 
   
   GuiSpeedCtr=__SPEEDUP;
    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
   
 
   
}

 

void mouseTurnleft_180(void)
{
 // mouseStop();
//  while(1);
   //j++;
  
   __GucMouseState   = __SPURT45;
   int8 cL = 0, cR = 0;
    piancha_l = (rightdis - 55)/10;
   if(rightdis>125){piancha_l = 0;}//左转右边没墙
   if(piancha_l > 2){piancha_l = 2;}
    if(piancha_l < -2){piancha_l = -2;}
   __GmLeft.uiPulse =13000;    //9000    //14400   17000
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse= 13000;           
   __GmRight.uiPulseCtr=0;
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   
   time=0;   
   __GucMouseState   = __TURNLEFT; 
   __GmRight.cState = __MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  = __MOTORRUN;
   GucMouseDir     = (GucMouseDir + 4) % 8;                             /*  方向标记                    */
             
   __GmSPID.sRef=200;           ///120
   __GmWPID.sRef=120;//70
   GW=0;
   jiao=0;
   while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>79400)//81400 //82400 84400 
       {
         break;
       }
      
   }
    __GmRight.cState = __MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  = __MOTORRUN;
   __GucMouseState   = __SPURT45;
   //mouseStop();while(1);  
      __GmWPID.sRef=0; 
    __GmLeft.uiPulse =18000- piancha_l*2886.67;            //7700
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =18000- piancha_l*2886.67;             
   __GmRight.uiPulseCtr=0;
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   
   
    __GmRight.cState = __MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  = __MOTORRUN;
    __GucMouseState   = __TURNLEFT; 
    GW=0;
   W=0;
      __GmWPID.sRef=120;
   while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>74400)//81400 //82400 84400 
       {
         break;
       } 
      
   }
  // mouseStop();while(1);
   GoHead45_flag=0;
   __GmWPID.sRef=0; 
   __GmSPID.sRef=200;   
  __GucMouseState   = __SPURT45;
 
   GuiSpeedCtr=3;
   if(actionsequence[action+1]==STRAIGHT90L) //多个迷宫格时
   {
     //
      __GucMouseState   = __SPURT45;
     __GmLeft.uiPulse =55000;    //9000    //14400      25400
     __GmLeft.uiPulseCtr=0;
     __GmRight.uiPulse =55000;           
     __GmRight.uiPulseCtr=0;
     __GmRight.cState  = __MOTORRUN;
     __GmLeft.cState   = __MOTORRUN;  
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
     goto End;
   }
   else  //单个迷宫格时
   { 
      __GmLeft.uiPulse =35000;    //9000    //14400      25400
      __GmLeft.uiPulseCtr=0;
      __GmRight.uiPulse =35000;           
      __GmRight.uiPulseCtr=0;
      __GmRight.cState  = __MOTORRUN;
      __GmLeft.cState   = __MOTORRUN;  
      while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
      while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
      //mouseStop();while(1);
      while(1)
      {
           if (cL) 
           {                                                       /*  是否允许检测左边            */
              if  ((__GucDistance[__LEFT]  & 0x01)==0)             /*  左边有支路，跳出程序        */   
              {
                goto End;//直接进入下一个动作
              }
           } 
           else 
           {                                                        /*  左边有墙时开始允许检测左边  */
              if (ucIRCheck[2]>GusDistance_L_Far) 
              {
                  cL = 1; 
              }
          }//20550
          if (cR) 
          {                                                       /*  是否允许检测右边            */
              if ((__GucDistance[__RIGHT]  & 0x01)==0)            /*  右边有支路，跳出程序        */
              {
                goto End;
              }
          }
          else 
          {
              if (ucIRCheck[3]>GusDistance_R_Far)
              {                       
                  cR = 1;
              }
          }
      }
   }  
   //GuiSpeedCtr=__SPEEDUP;
   __GmRight.cState = __MOTORSTOP;       
   __GmLeft.cState  = __MOTORSTOP;
   __GmRight.sSpeed = 0;
   __rightMotorContr();
   __GmLeft.sSpeed = 0;
   __leftMotorContr();
   __GmRight.uiPulseCtr = 0;
   __GmLeft.uiPulseCtr = 0;
   End: 
  {__GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0; 
  }
}

void ZEROcorrect(void)
{
static int il=0; 
   __GmRight.cState = __MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  = __MOTORRUN;
   __GmSPID.sRef=0;           ///120//150
   __GmWPID.sRef=95;//70//90
   GW=0;
    while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>82000) //81200//  80200 79200 78700 78900 //79000 79500 80000
       {
         break;
       } 
      
   }
   il++;
   __GmWPID.sRef=0;
 delay(50000);delay(50000);delay(50000);delay(50000);
 GW=0;
 __GmWPID.sRef=-95;
  GW=0;
  while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>82000) //81200//  80200 79200 78700 78900 //79000 79500 80000
       {
         break;
       } 
      
   }
 //if(il==2){ mouseStop(); while(1);}     
   __GmWPID.sRef=0; 
   __GucMouseState   = __GOAHEAD;
 
   GuiSpeedCtr=__SPEEDUP;
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
** Descriptions:        左转7
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void mouseTurnright_C(void)
{
  static int q=0;
  q++;
  while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   GW=0;
   time=0;   
   __GucMouseState   = __TURNRIGHT; 
   __GmRight.cState =__MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  =__MOTORRUN;
   GucMouseDir     = (GucMouseDir + 2) % 8;                            /*  方向标记                    */
   piancha_r = (leftdis - 95)/10;
   if(leftdis > 155){piancha_r = 0;}
   if(piancha_r > 2){piancha_r = 2;}
   if(piancha_r < -2){piancha_r = -2;}
   __GmSPID.sRef=145;
   __GmWPID.sRef=-95;
//    while(1)
//   {
//       W=(float)GW*5121/4096/2;
//       if(W>60000)   
//       {
//         break;
//       } 
//      
//   }
    while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>93000)//81400 //82400 84400 
       {
         break;
       } 
      
   }
   //if(q==1){mouseStop();while(1);}
   __GmWPID.sRef=0;     
   __GucMouseState   = __GOAHEAD;
   GuiSpeedCtr=3;
   __GmLeft.uiPulse =17000-piancha_r*2886.67;    //9000          
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =17000-piancha_r*2886.67;           
   __GmRight.uiPulseCtr=0;
  while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
  while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
  //if(i==6)
  //{
    //mouseStop();
    //while(1);
  //}
   __GucMouseState   = __TURNRIGHT;
  //if(q==2){mouseStop();while(1);}
//   mouseStop();
//    while(1);  
    
   GuiSpeedCtr=__SPEEDUP;
    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
  //mouseStop();
    //while(1);
    //if(i==)
    //{
      //mouseStop();
      //while(1);
    //}
   
}
/*********************************************************************************************************
** Function name:       mouseTurnleft
** Descriptions:        左转
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void mouseTurnleft_C(void)
{static int j=0;
   j++;
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   time=0;  
    piancha_l = (rightdis - 90)/10;
   if(rightdis>145){piancha_l = 0;}//左转右边没墙
   if(piancha_l > 2){piancha_l = 2;}
    if(piancha_l < -2){piancha_l = -2;}
   
   __GucMouseState   = __TURNLEFT; 
   __GmRight.cState = __MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  = __MOTORRUN;
   GucMouseDir     = (GucMouseDir + 6) % 8;                             /*  方向标记                    */
             
   __GmSPID.sRef=145;           ///120
   __GmWPID.sRef=95;//70
   GW=0;
    while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>84000)//81400 //82400 84400 
       {
         break;
       } 
      
   }
  
 //if(j==1){mouseStop();while(1);}
   __GmWPID.sRef=0; 
   __GucMouseState   = __GOAHEAD;
 
   GuiSpeedCtr=3;
   __GmLeft.uiPulse =9600- piancha_l*286.67;           //7700
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =9600- piancha_l*286.67;              
   __GmRight.uiPulseCtr=0;
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
      __GucMouseState   = __TURNLEFT;
   
//   mouseStop();
//    while(1);  
   
   
   GuiSpeedCtr=__SPEEDUP;
    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
    //if(j==5)
    //{
      //mouseStop();
      //while(1); 
    //}
   
}
void mouseTurnright_Y1(void)
{  
   __GmLeft.uiPulse =6000;            // 135*32*1024/(12*25*3.14)  R_r=14mm  R_l=14+72=86 ;4697
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =6000;           // 22*32*1024/(12*25*3.14);
   __GmRight.uiPulseCtr=0;
   __GmSPID.sRef=0;
   __GmWPID.sRef=-20;
   GW=0;
   __GucMouseState   = __TURNRIGHTY; 
   __GmRight.cState =__MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  =__MOTORRUN;
   GucMouseDir     = (GucMouseDir + 2) % 8;                            /*  方向标记                    */
   
    while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>90000)   
       {
         break;
       } 
      // if(W>70000)
      //   __GmWPID.sRef=10;
   }
   __mazeInfDebug();

   __GmWPID.sRef=0;
   __GmSPID.sRef=50;

   GuiSpeedCtr=__SPEEDUP;
   __GmRight.cState = __MOTORSTOP;       
   __GmLeft.cState  = __MOTORSTOP;
   __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
  // __delay(100000);
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
void mouseTurnleft_Y1(void)
{  
  // __delay(2000000);
   __GmLeft.uiPulse =6000;            // 22*32*1024/(12*25*3.14)  R_l=14mm  R_r=14+72=86
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =6000;           // 135*32*1024/(12*25*3.14)
   __GmRight.uiPulseCtr=0;
   
   __GmWPID.sRef=24;
   GW=0;
   __GucMouseState   = __TURNLEFTY; 
 
   __GmRight.cState = __MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  = __MOTORRUN;
   GucMouseDir     = (GucMouseDir + 6) % 8;                            /*  方向标记                    */
    while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>90000)   
       {
         break;
       } 
      // if(W>70000)
      //   __GmWPID.sRef=10;
   }
   __mazeInfDebug();
   __GmWPID.sRef=0;
   __GmSPID.sRef=25;
  
   GuiSpeedCtr=__SPEEDUP;
   __GmRight.cState = __MOTORSTOP;       
   __GmLeft.cState  = __MOTORSTOP;
   __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
   //__delay(100000);
   __GmRight.uiPulseCtr = 0;
   __GmLeft.uiPulseCtr = 0;
}


void mouseTurnright_Y(void)
{  
GW=0;
   time=0;   
   
    /*   __GucMouseState   = __GOAHEAD;
       
  __GmLeft.uiPulse =15000;          //7700//15500
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =15000;            
   __GmRight.uiPulseCtr=0;

   __GmRight.cState = __MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  = __MOTORRUN;
 
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);*/
   __GucMouseState   = __TURNRIGHT; 
   __GmRight.cState =__MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  =__MOTORRUN;
   GucMouseDir     = (GucMouseDir + 2) % 8;                            /*  方向标记                    */
   
   __GmSPID.sRef=0;
   __GmWPID.sRef=-80;
    while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>92600)   
       {
         break;
       } 
      
   }
    
   __GmWPID.sRef=0;  
   __GmSPID.sRef=120;
   __GucMouseState   = __GOAHEAD;
 
   GuiSpeedCtr=3;
   __GmLeft.uiPulse =27000;          //7700//15500
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =27000;            
   __GmRight.uiPulseCtr=0;

              
 
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
//     mouseStop();while(1);
   
      __GucMouseState   = __TURNRIGHT;
   //   mouseStop();while(1);
  GuiSpeedCtr=__SPEEDUP;
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
void mouseTurnleft_Y(void)   //不调
{
  time=0;   
  /*   __GucMouseState   = __GOAHEAD;
  __GmLeft.uiPulse =15000;          //7700//15500
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =15000;            
   __GmRight.uiPulseCtr=0;

   __GmRight.cState = __MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  = __MOTORRUN;
 
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
  
  */
   __GucMouseState   = __TURNLEFT; 
   __GmRight.cState = __MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  = __MOTORRUN;
   GucMouseDir     = (GucMouseDir + 6) % 8;                            /*  方向标记                    */
             

   __GmWPID.sRef=80;//70
   GW=0;
    while(1)
   {
       W=(float)GW*5121/4096/2;
       if(W>90000)   
       {
         break;
       } 
      
   }
   
  
   __GmWPID.sRef=0; 
   __GmSPID.sRef=120;
   __GucMouseState   = __GOAHEAD;
 
   GuiSpeedCtr=3;
   __GmLeft.uiPulse =25000;          //7700//15500
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =25000;            
   __GmRight.uiPulseCtr=0;

              
 
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   

   __GucMouseState   = __TURNLEFT;
   // mouseStop();while(1);
   GuiSpeedCtr=__SPEEDUP;
    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
}




void onestep(void)
{
     __GiMaxSpeed = 50;
     __GucMouseState   = __GOAHEAD;
    __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
     __GmRight.uiPulse =6000;             
     __GmRight.uiPulseCtr=0;
     __GmLeft.uiPulse =6000;
     __GmLeft.uiPulseCtr=0;
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);    
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse); 
//     mouseStop();
//     while(1);
   
}

void onestep1(void)
{
     __GmRight.uiPulse =35000;  // 75000          
     __GmRight.uiPulseCtr=0;
     __GmLeft.uiPulse =35000;
     __GmLeft.uiPulseCtr=0;
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);    
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse); 
}
void onestep2(void)
{
     __GmRight.uiPulse =12000;          
     __GmRight.uiPulseCtr=0;
     __GmLeft.uiPulse =12000;
     __GmLeft.uiPulseCtr=0;
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);    
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse); 
}


void onestep3(void)
{
     __GmRight.uiPulse =7000;             
     __GmRight.uiPulseCtr=0;
     __GmLeft.uiPulse =7000;
     __GmLeft.uiPulseCtr=0;
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);    
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse); 
}
/*********************************************************************************************************
** Function name:       mouseTurnback
** Descriptions:        根据前方近距，旋转180度
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void mouseTurnback1(void)
{ 
   __GmSPID.sRef=0;  
    GW=0;
    time=0;
     __GucMouseState   = __TURNBACK;
    __GmLeft.cState   = __MOTORRUN;
    __GmRight.cState  = __MOTORRUN;
    GucMouseDir = (GucMouseDir + 4) % 8;                                  
    while(1)
   {
      if(time>604)
        break;
   }

   /* __GmSPID.sRef=-80; 
     __GmRight.uiPulse =16000;          
     __GmRight.uiPulseCtr=0;
     __GmLeft.uiPulse =16000;
     __GmLeft.uiPulseCtr=0;  
     while ((__GmRight.uiPulseCtr+100 ) <= __GmRight.uiPulse);
     while ((__GmLeft.uiPulseCtr+100 ) <= __GmLeft.uiPulse); -*/      
    __GucMouseState   = __GOAHEAD ;
    mouseStop();
    __delay(1500000);
     GuiSpeedCtr=__SPEEDUP;
         __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
}


void mouseTurnback_Y3(void)
{ 
  
  if(GucFrontNear)
  {
      __GmSPID.sRef=120;
      while((ucIRCheck[0]<GusDistance_FL_Near)||(ucIRCheck[1]<GusDistance_FR_Near));
       __GucMouseState   = __GOAHEAD ;
     __GmRight.uiPulse =1000;             
     __GmRight.uiPulseCtr=0;
     __GmLeft.uiPulse = 1000;
     __GmLeft.uiPulseCtr=0;
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);    
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);    
    
  }
      backflag=0;
     if((ucIRCheck[2]>GusDistance_L_Near))   //偏左
     {
       backflag=1;
     }
    GucFrontNear=0;
 
    __GmSPID.sRef=0; 
    mouseStop();
    GW=0;
    time=0;
    
    __GucMouseState   = __TURNBACK;
    __GmLeft.cState   = __MOTORRUN;
    __GmRight.cState  = __MOTORRUN;
    GucMouseDir = (GucMouseDir + 4) % 8;         
   
       
    while(1)
   {
      if(time>725)//597
        break;
   }
    __GmWPID.sRef=0;
     mouseStop();
    __GucMouseState   = __GOBACK ;
    __GmSPID.sRef=-40;
     __GmRight.uiPulse =15000;             
     __GmRight.uiPulseCtr=0;
     __GmLeft.uiPulse = 15000;
     __GmLeft.uiPulseCtr=0;
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);    
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);    
  __GmSPID.sRef=0;
     mouseStop();
    __GucMouseState   = __GOAHEAD ;
    __delay(2000000);
   
    GuiSpeedCtr=__SPEEDUP;
    
   /* __GucMouseState   = __GOAHEAD ;
   // mouseStop();
    //while(1);
    __GmRight.uiPulse =28000;             
     __GmRight.uiPulseCtr=0;
     __GmLeft.uiPulse = 28000;
     __GmLeft.uiPulseCtr=0;
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);    
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse); */    
    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;

    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;//3000
    __GmLeft.uiPulseCtr = 0;//3000
     
}








void mouseTurnback(void)
{ 
if(GucFrontNear)
  {
      __GmSPID.sRef=50;//120
      while((ucIRCheck[0]<GusDistance_FL_Near)||(ucIRCheck[1]<GusDistance_FR_Near));
     __GmRight.uiPulse =1000;             
     __GmRight.uiPulseCtr=0;
     __GmLeft.uiPulse = 1000;
     __GmLeft.uiPulseCtr=0;
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);    
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);      
  }
  GucFrontNear=0;
  backflag=1;
   __GmSPID.sRef=0; 
  // mouseStop();while(1);
    GW=0;
    time=0;
    __GucMouseState   = __TURNBACK;
    __GmLeft.cState   = __MOTORRUN;
    __GmRight.cState  = __MOTORRUN;
    GucMouseDir = (GucMouseDir + 4) % 8;    
        __GmWPID.sRef=90;//80
 
   while(1)
   {
       if(GW>290000)  
       {
         break;
       }                        
   }
//mouseStop();while(1);
    __GmWPID.sRef=0;

   __delay(5000000);
   __GucMouseState   = __GOAHEAD ;

    GuiSpeedCtr=__SPEEDUP;
    __GmSPID.sRef=150;//120
  
    __GmRight.uiPulse =23000;    //23000         
     __GmRight.uiPulseCtr=0;
     __GmLeft.uiPulse = 23000;
     __GmLeft.uiPulseCtr=0;
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);    
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);  

    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;

    
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 23000;
    __GmLeft.uiPulseCtr = 23000;
}

 
void mouseTurnbackqidian(void)
{ 
  
 if(GucFrontNear)
  {
      __GmSPID.sRef=150;//120
      while((ucIRCheck[0]<GusDistance_FL_Near)||(ucIRCheck[1]<GusDistance_FR_Near));
     __GmRight.uiPulse =1000;             
     __GmRight.uiPulseCtr=0;
     __GmLeft.uiPulse = 1000;
     __GmLeft.uiPulseCtr=0;
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);    
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);      
  }
  GucFrontNear=0;
  backflag=1;
   __GmSPID.sRef=0; 
   mouseStop();
    GW=0;
    time=0;
    __GucMouseState   = __TURNBACK;
    __GmLeft.cState   = __MOTORRUN;
    __GmRight.cState  = __MOTORRUN;
    GucMouseDir = (GucMouseDir + 2) % 4;    
        __GmWPID.sRef=90;//80
 
   while(1)
   {
       if(GW>300000)  
       {
         break;
       }                        
   }
// mouseStop();while(1);
    __GmWPID.sRef=0;

   __delay(5000000);
   __GucMouseState   = __GOAHEAD ;

    GuiSpeedCtr=__SPEEDUP;
    __GmSPID.sRef=150;//120
  
    __GmRight.uiPulse =23000;    //23000         
     __GmRight.uiPulseCtr=0;
     __GmLeft.uiPulse = 23000;
     __GmLeft.uiPulseCtr=0;
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);    
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);  

    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;

    
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 23000;
    __GmLeft.uiPulseCtr = 23000;
}




void mouseTurnback_js(void)
{ 

  if(GucFrontNear)
  {
      __GmSPID.sRef=100;
      while((ucIRCheck[0]<GusDistance_FL_Near)||(ucIRCheck[1]<GusDistance_FR_Near));
     __GmRight.uiPulse =1000;             
     __GmRight.uiPulseCtr=0;
     __GmLeft.uiPulse = 1000;
     __GmLeft.uiPulseCtr=0;
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);    
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);      
  }
  GucFrontNear=0;
  backflag=1;
   __GmSPID.sRef=0; 
   mouseStop();
    GW=0;
    time=0;
    __GucMouseState   = __TURNBACK;

    __GmLeft.cState   = __MOTORRUN;
    __GmRight.cState  = __MOTORRUN;
    GucMouseDir = (GucMouseDir + 2) % 4;    
        __GmWPID.sRef=80;
  
   while(1)
   {
       if(GW>290000)      //315000
       {
         break;
       }                        
   }
   
   __GmWPID.sRef = 0;                           
    mouseStop();
    __GucMouseState = __GOBACK;
    __GmSPID.sRef = -40;
    __GmRight.uiPulse = 12000;                  
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulse = 12000;
    __GmLeft.uiPulseCtr = 0;
    while ((__GmRight.uiPulseCtr + 200) <= __GmRight.uiPulse);    
    while ((__GmLeft.uiPulseCtr + 200)  <= __GmLeft.uiPulse);    
  
    mouseStop();
    __GucMouseState = __GOAHEAD;
    __delay(5000000);
    
    GuiSpeedCtr = __SPEEDUP;
    __GmSPID.sRef = 100;                         
    __GucMouseState = __GOAHEAD;
    __GmRight.uiPulse = 3000;   //   //9000            
    __GmRight.uiPulseCtr = 0;  
    __GmLeft.uiPulse = 3000;    //9000                  
    __GmLeft.uiPulseCtr = 0;
    while ((__GmRight.uiPulseCtr + 200 ) <= __GmRight.uiPulse);    
    while ((__GmLeft.uiPulseCtr + 200 ) <= __GmLeft.uiPulse);    
    
    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
}


void mouseTurnbackqi(void)
{ 
  if(GucFrontNear)
  {
      __GmSPID.sRef=0;
      while((ucIRCheck[0]<GusDistance_FL_Near)||(ucIRCheck[1]<GusDistance_FR_Near));
     __GmRight.uiPulse =1000; 
     
     __GmRight.uiPulseCtr=0;
     __GmLeft.uiPulse = 1000;
     __GmLeft.uiPulseCtr=0;
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);    
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);      
  }
  GucFrontNear=0;
  backflag=1;
   __GmSPID.sRef=0; 
   //mouseStop();while(1);
    GW=0;
    time=0;
    __GucMouseState   = __TURNBACK;
    __GmLeft.cState   = __MOTORRUN;
    __GmRight.cState  = __MOTORRUN;
    GucMouseDir = (GucMouseDir + 4) % 8;    
        __GmWPID.sRef=80;
  
   while(1)
   {
       if(GW>280000)  
       {
         break;
       }                        
   }
   __GmWPID.sRef = 0;                             
    mouseStop();
    __GucMouseState = __GOBACK;
    __GmSPID.sRef = -40;
    __GmRight.uiPulse = 26000;     //17000              
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulse = 26000;
    __GmLeft.uiPulseCtr = 0;
    while ((__GmRight.uiPulseCtr + 200) <= __GmRight.uiPulse);    
    while ((__GmLeft.uiPulseCtr + 200)  <= __GmLeft.uiPulse);    
  
    mouseStop();  
    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;

    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;               
    __GmLeft.uiPulseCtr = 0;               


}

void mouseTurnback_H(void)
{ 
  backflag=1;
   __GmSPID.sRef=0; 
   mouseStop();
    GW=0;
    time=0;
    __GucMouseState   = __TURNBACK;

    __GmLeft.cState   = __MOTORRUN;
    __GmRight.cState  = __MOTORRUN;
    GucMouseDir = (GucMouseDir + 4) % 8;    
        __GmWPID.sRef=80;

   while(1)
   {
       if(GW>290000)      //315000
       {
         break;
       }                        
   }
   
   __GmWPID.sRef = 0;                           
  __GmSPID.sRef = -20;
    __GmRight.uiPulse = 100; //12000 //1000                
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulse = 100;
    __GmLeft.uiPulseCtr = 0;
    while ((__GmRight.uiPulseCtr + 200) <= __GmRight.uiPulse);    
    while ((__GmLeft.uiPulseCtr + 200)  <= __GmLeft.uiPulse);    
    __delay(5000000);
    __GmSPID.sRef = 100;                         
    __GucMouseState = __GOAHEAD;
    __GmRight.uiPulse = 3000;   //   //9000            
    __GmRight.uiPulseCtr = 0;  
    __GmLeft.uiPulse = 3000;    //9000                  
    __GmLeft.uiPulseCtr = 0;
    while ((__GmRight.uiPulseCtr + 200 ) <= __GmRight.uiPulse);    
    while ((__GmLeft.uiPulseCtr + 200 ) <= __GmLeft.uiPulse);    
    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
}
static float w = 0;

static float w1 = 0;
static u16 lala=1;
static float gyro_0=0;
void voltageDetect(void)
{if(lala == 1)
  {
    
    gyro_0=zijiaozheng;
    Angle_TLY_Average=zijiaozheng;
    w = ((Angle_TLY_Average - gyro_0)+w1)*0.5*1055*3300/(4096*3.96*360);//3.96为系统参数
    jiao += w*0.001;
    w1 = Angle_TLY_Average - gyro_0;
    lala = 0;
  }
  else
  {
   w = ((Angle_TLY_Average - gyro_0)+w1)*0.5*1055*3300/(4096*3.96*360);//3.96为系统参数
   jiao += w*0.001;
   w1 = Angle_TLY_Average - gyro_0;}   
   if(after_turn){jiao = jiao - angle_temp; after_turn=0;angle_temp=0;
  //if(((jiao<(angle_temp+10))&&(jiao>(angle_temp-10)))&&(after_turn)){jiao = jiao - angle_temp; after_turn=0;angle_temp=0;}
  }
}

void voltageDetect1(void)
{
  u16 w;
  if(Angle_TLY_Average>=voltageDetectRef)
  {
     w= Angle_TLY_Average - voltageDetectRef;
  }
  else
  {
     w= voltageDetectRef - Angle_TLY_Average;
  }
  GW=GW+w;
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
 
 
static uint8 sjjj;

void mazeSearch(void)                
{
    int8 cL = 0, cR = 0, cCoor = 1;
    if (__GmLeft.cState)
    {
        cCoor = 0;
    }
    if((__GucMouseState==__TURNRIGHT)||(__GucMouseState==__TURNLEFT))
    {
        __GmLeft.uiPulseCtr =40000;
        __GmRight.uiPulseCtr =40000;
        cL = 1;
        cR = 1;
        if(((__GucDistance[__FRONTR]!=0)&&(__GucDistance[__FRONTL]!=0))||((__GucDistance[ __LEFT] & 0x01) == 0)||((__GucDistance[__RIGHT] & 0x01) == 0))
        {
          if((__GucDistance[__FRONTR]!=0)&&(__GucDistance[__FRONTL]!=0))
          {
          GuiTpusle_LR =16000;   
            if((__GucMouseState==__TURNRIGHT)&&((__GucDistance[ __LEFT] & 0x01) == 0)){GuiTpusle_LR_l=-800;W_l=0;}//LR 
            if((__GucMouseState==__TURNRIGHT)&&((__GucDistance[__RIGHT] & 0x01) == 0)){GuiTpusle_LR_r=0;W_r=-0;}
              if((__GucMouseState==__TURNLEFT)&&((__GucDistance[ __LEFT] & 0x01) == 0)){GuiTpusle_LR_l=0;W_l=0;}//左接左7200
              if((__GucMouseState==__TURNLEFT)&&((__GucDistance[__RIGHT] & 0x01) == 0)){GuiTpusle_LR_r=-4000;W_r=0;}//1700 700//-1200  
            
             
          }
          else
            GuiTpusle_LR =12000;
        }
        
        else{
          GuiTpusle_LR =0; 
        }
    }
    
    
    else{
      GuiTpusle_LR =0;
    }
    __GucMouseState   = __GOAHEAD;
    __GiMaxSpeed      =   SEARCHSPEED;       
    __GmRight.uiPulse =   MAZETYPE * ONEBLOCK;
    __GmLeft.uiPulse  =   MAZETYPE * ONEBLOCK;
    __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
     GuiSpeedCtr=__SPEEDUP;
    while (__GmLeft.cState != __MOTORSTOP) 
    {
       
        if (__GmLeft.uiPulseCtr >= ONEBLOCK)
        {                          /*  判断是否走完一格*/
            __GmLeft.uiPulse    -= ONEBLOCK;
            __GmLeft.uiPulseCtr -= ONEBLOCK;
            if (cCoor) 
            {
                if(((__GucDistance[__FRONTR]!=0)&&(__GucDistance[__FRONTL]!=0))&&(ucIRCheck[2]>GusDistance_L_Far)&&(ucIRCheck[3]>GusDistance_R_Far))//0x01
              {          
               GucFrontNear=1;
               
                goto End;
              }
              __mouseCoorUpdate();                                    /*  更新坐标                    */
            } 
            else 
            {
                cCoor = 1;
            }
        }
        if (__GmRight.uiPulseCtr >= ONEBLOCK) {                         /*  判断是否走完一格            */
            __GmRight.uiPulse    -= ONEBLOCK;
            __GmRight.uiPulseCtr -= ONEBLOCK;
        }
        
        if (cL) {                                                       /*  是否允许检测左边            */
            if  ((__GucDistance[__LEFT]  & 0x01)==0)
            {                 /*  左边有支路，跳出程序        */
            
                __GmRight.uiPulse =  __GmRight.uiPulseCtr + 18000 - GuiTpusle_LR- GuiTpusle_LR_l; //10800 //7500//5000    //1500
                __GmLeft.uiPulse  =  __GmLeft.uiPulseCtr  + 18000 - GuiTpusle_LR- GuiTpusle_LR_l; //调整转弯脉冲
                while ((__GucDistance[__LEFT]  & 0x01)==0)
                {
                 
                    if ((__GmLeft.uiPulseCtr + 400) > __GmLeft.uiPulse) 
                    {
                        GucFrontNear=0;
                        goto End;
                    }
                }
                __GmRight.uiPulse = MAZETYPE * ONEBLOCK;
                __GmLeft.uiPulse  = MAZETYPE * ONEBLOCK;
                GuiSpeedCtr=__SPEEDUP;
            }
        } else {                                                        /*  左边有墙时开始允许检测左边  */
            if (ucIRCheck[2]>GusDistance_L_Far) {
                cL = 1; 
               
            }
        }
        if (cR) {                                                       /*  是否允许检测右边            */
            if ((__GucDistance[__RIGHT]  & 0x01)==0){//mouseStop();while(1);               /*  右边有支路，跳出程序        */
            
                __GmRight.uiPulse = __GmRight.uiPulseCtr + 19500 - GuiTpusle_LR- GuiTpusle_LR_r; //6500 //5000   //3300
                __GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 19500 - GuiTpusle_LR- GuiTpusle_LR_r; 
                
               
                while ((__GucDistance[__RIGHT]  & 0x01)==0) {
                 
                    if ((__GmLeft.uiPulseCtr + 400) > __GmLeft.uiPulse)
                    {
                        GucFrontNear=0;
                        goto End;
                        
                    }
                }
                __GmRight.uiPulse = MAZETYPE * ONEBLOCK;
                __GmLeft.uiPulse  = MAZETYPE * ONEBLOCK;
                GuiSpeedCtr=__SPEEDUP;
            }
        } else {
            if (ucIRCheck[3]>GusDistance_R_Far)
            {
                cR = 1;
            }
        }
    }
    
End:   
           
          
          __mouseCoorUpdate();                                            /*  更新坐标                    */
        

    
}


/*********************************************************************************************************
** Function name:       __mouseCoorUpdate
** Descriptions:        根据当前方向更新坐标值
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __mouseCoorUpdate (void)
{

    switch (GucMouseDir) {

    case 0:
        GmcMouse.cY++;
        break;

    case 2:
        GmcMouse.cX++;
      
        break;

    case 4:
        GmcMouse.cY--;
        break;

    case 6:
        GmcMouse.cX--;
      
        break;

    default:
        break;
    }
    __wallCheck();
    __mazeInfDebug ();
   

  
    
}

void __mazeInfDebug (void)
{
    /*
     *  显示方向
     */
    switch (GucMouseDir) {
        
    case 0:
        zlg7289Download(2, 3, 0, 0x47);                                /*  向前，用F表示               */
        break;
        
    case 2:
        zlg7289Download(2, 3, 0, 0x77);                                /*  向右，用R表示               */
        break;
        
    case 4:
        zlg7289Download(2, 3, 0, 0x1f);                                /*  向后，用b表示               */
        break;
        
    case 6:
        zlg7289Download(2, 3, 0, 0x0e);                                /*  向左，用L表示               */
        break;
        
    default :
        zlg7289Download(2, 3, 0, 0x4f);                                /*  错误，用E表示               */
        break;
    }
    /*
     *  显示坐标
     */
    zlg7289Download(1, 0, 0, GmcMouse.cX / 10);
    zlg7289Download(1, 1, 0, GmcMouse.cX % 10);
    zlg7289Download(1, 6, 0, GmcMouse.cY / 10);
    zlg7289Download(1, 7, 0, GmcMouse.cY % 10);
}

/*********************************************************************************************************
** Function name:       __wallCheck
** Descriptions:        根据传感器检测结果判断是否存在墙壁
** input parameters:    无
** output parameters:   无
** Returned value:      cValue: 低三位从左到右一次代表左前右。1为有墙，0为没墙。
*********************************************************************************************************/
void __wallCheck (void)
{
    uint8 ucMap = 0;
    uint8 uctemp = 0;
    ucMap |= MOUSEWAY_B;
    
    if (ucIRCheck[2]>GusDistance_L_Far)//左边有挡板，但是距离较远
    {
        ucMap &= ~MOUSEWAY_L;          //相对方向的左边有墙，并却转换成绝对坐标
        uctemp |= 0x06;
    }
    else 
    {
        ucMap |=MOUSEWAY_L;         //相对方向的左边无墙，并却转换成绝对坐标   
    }
    
    if ((ucIRCheck[0]>GusDistance_FL_Far)&&(ucIRCheck[1]>GusDistance_FR_Far)) 
    {
        ucMap &= ~MOUSEWAY_F;
        uctemp |= 0x40;
        //mouseStop();
           //while(1);
    }
    else 
    {
        ucMap |=  MOUSEWAY_F;
    }
    if (ucIRCheck[3]>GusDistance_R_Far) 
    {
        ucMap &= ~MOUSEWAY_R;
        uctemp |= 0x30;
    }else {
        ucMap |=  MOUSEWAY_R;
    }
    GucMapBlock0[GmcMouse.cX][GmcMouse.cY]=ucMap;
    GucMapBlock[GmcMouse.cX][GmcMouse.cY]=ucMap;
    GucMapBlock1[GmcMouse.cX][GmcMouse.cY]=ucMap;
    if(GmcMouse.cY<(MAZETYPE-1))
        {GucMapBlock1[GmcMouse.cX][GmcMouse.cY+1] &= ~(((~ucMap)&0x01)*4);}       /*将该坐标周围坐标墙壁资料更改  注：洪水用*/
       if(GmcMouse.cX<(MAZETYPE-1))
        { GucMapBlock1[GmcMouse.cX+1][GmcMouse.cY]&= ~(((~ucMap)&0x02)*4);}
         if(GmcMouse.cY>0)
         {GucMapBlock1[GmcMouse.cX][GmcMouse.cY-1]&= ~(((~ucMap)&0x04)/4);}
        if(GmcMouse.cX>0)
         {GucMapBlock1[GmcMouse.cX-1][GmcMouse.cY]&= ~(((~ucMap)&0x08)/4);}
              
      if(GmcMouse.cY<(MAZETYPE-1))
         {GucMapBlock[GmcMouse.cX][GmcMouse.cY+1] |=    ((ucMap&0x01)*4);}        /*将该坐标周围坐标墙壁资料更改  注：在初始为有墙时管用*/
      if(GmcMouse.cX<(MAZETYPE-1))
         { GucMapBlock[GmcMouse.cX+1][GmcMouse.cY]|=  ((ucMap&0x02)*4);}
        if(GmcMouse.cY>0)
         {GucMapBlock[GmcMouse.cX][GmcMouse.cY-1]|=  ((ucMap&0x04)/4);}
        if(GmcMouse.cX>0)
          {GucMapBlock[GmcMouse.cX-1][GmcMouse.cY]|=  ((ucMap&0x08)/4);}
    
    zlg7289Download(2, 2, 0, uctemp);
 
}

uint8 DenggaoCheck (void)//搜索中制作等高图stop，原地转弯判断
{
  if((__GucDistance[__FRONT])&&(__GucDistance[ __LEFT] & 0x01)&&(__GucDistance[__RIGHT] & 0x01)){
        return(true);
    }else {
        return(false);
    }
}
uint8 PulseCtrCheck (void)////搜索中制作等高图stop，已走步数判断
{
  if(__GmLeft.uiPulseCtr  > (__GmLeft.uiPulse+15000)){
        return(true);
    }else {
        return(false);
    }
}

/*void EXTI3_IRQHandler(void)
{
    char key; 
    if(EXTI_GetITStatus(EXTI_Line3)!=RESET)
      {
        key = zlg7289Key();
        if (key != 0xff) 
        { 
          if(key>=4)
          {
              if(key==10)
              {
                  DIS[0]=GusDistance_L_Near;
                  DIS[1]=GusDistance_R_Near;
                  DIS[2]=GusDistance_L_Mid;
                  DIS[3]=GusDistance_R_Mid; 
                  DIS[4]=GusDistance_L_Far;
                  DIS[5]=GusDistance_R_Far;
                  DIS[6]=GusDistance_FL_Far;
                  DIS[7]=GusDistance_FR_Far;
                  DIS[8]=GusDistance_FL_Near;
                  DIS[9]=GusDistance_FR_Near;
                  STMFLASH_Write(IR_SAVE_ADDRESS,(u16 *)DIS,10);
                  __delay(1000000);                
              }
           
              else
              {
                  Tab=key;
                  ir_flag=1;     //自动调红外的部分，屏蔽后不会自动调节红外了
              }
           }
          switch(Tab)
          {
              case 4:
                if(ir_flag==1)
                {
                    GusDistance_L_Near=ucIRCheck[2];
                    GusDistance_R_Near=ucIRCheck[3];
                    ir_flag=0;
                }
                if(key==1)
                {
                    GusDistance_L_Near+=10;
                    zlg7289Download (0,4,0,GusDistance_L_Near/1000);
                    zlg7289Download (0,5,0,GusDistance_L_Near/100%10);
                    zlg7289Download (0,6,0,GusDistance_L_Near/10%10);
                    zlg7289Download (0,7,0,GusDistance_L_Near%10);
                }
                else if(key==3)
                {
                    GusDistance_L_Near-=1;
                    zlg7289Download (0,5,0,GusDistance_L_Near/100);
                    zlg7289Download (0,6,0,GusDistance_L_Near/10%10);
                    zlg7289Download (0,7,0,GusDistance_L_Near%10);                 
                }
                else if(key==0)
                {
                    GusDistance_R_Near+=10;
                    zlg7289Download (0,5,0,GusDistance_R_Near/100);
                    zlg7289Download (0,6,0,GusDistance_R_Near/10%10);
                    zlg7289Download (0,7,0,GusDistance_R_Near%10);                    
                }
                else if(key==2)
                {
                    GusDistance_R_Near-=1;
                    zlg7289Download (0,5,0,GusDistance_R_Near/100);
                    zlg7289Download (0,6,0,GusDistance_R_Near/10%10);
                    zlg7289Download (0,7,0,GusDistance_R_Near%10);                 
                }
                break;
              case 5:
                if(ir_flag==1)
                {
                    GusDistance_L_Mid=ucIRCheck[2];
                    GusDistance_R_Mid=ucIRCheck[3];
                    ir_flag=0;
                }
                if(key==1)
                {
                    GusDistance_L_Mid+=10;
                    zlg7289Download (0,5,0,GusDistance_L_Mid/100);
                    zlg7289Download (0,6,0,GusDistance_L_Mid/10%10);
                    zlg7289Download (0,7,0,GusDistance_L_Mid%10);
                }
                else if(key==3)
                {
                    GusDistance_L_Mid-=1;
                    zlg7289Download (0,5,0,GusDistance_L_Mid/100);
                    zlg7289Download (0,6,0,GusDistance_L_Mid/10%10);
                    zlg7289Download (0,7,0,GusDistance_L_Mid%10);                 
                }
                else if(key==0)
                {
                    GusDistance_R_Mid+=10;
                    zlg7289Download (0,5,0,GusDistance_R_Mid/100);
                    zlg7289Download (0,6,0,GusDistance_R_Mid/10%10);
                    zlg7289Download (0,7,0,GusDistance_R_Mid%10);                    
                }
                else if(key==2)
                {
                    GusDistance_R_Mid-=1;
                    zlg7289Download (0,5,0,GusDistance_R_Mid/100);
                    zlg7289Download (0,6,0,GusDistance_R_Mid/10%10);
                    zlg7289Download (0,7,0,GusDistance_R_Mid%10);                 
                }
                break;                
              case 6:
                if(ir_flag==1)
                {
                    GusDistance_L_Far=ucIRCheck[2];
                    GusDistance_R_Far=ucIRCheck[3];
                    ir_flag=0;
                }
                if(key==1)
                {
                    GusDistance_L_Far+=10;
                    zlg7289Download (0,5,0,GusDistance_L_Far/100);
                    zlg7289Download (0,6,0,GusDistance_L_Far/10%10);
                    zlg7289Download (0,7,0,GusDistance_L_Far%10);
                }
                else if(key==3)
                {
                    GusDistance_L_Far-=1;
                    zlg7289Download (0,5,0,GusDistance_L_Far/100);
                    zlg7289Download (0,6,0,GusDistance_L_Far/10%10);
                    zlg7289Download (0,7,0,GusDistance_L_Far%10);                 
                }
                else if(key==0)
                {
                    GusDistance_R_Far+=10;
                    zlg7289Download (0,5,0,GusDistance_R_Far/100);
                    zlg7289Download (0,6,0,GusDistance_R_Far/10%10);
                    zlg7289Download (0,7,0,GusDistance_R_Far%10);                    
                }
                else if(key==2)
                {
                    GusDistance_R_Far-=1;
                    zlg7289Download (0,5,0,GusDistance_R_Far/100);
                    zlg7289Download (0,6,0,GusDistance_R_Far/10%10);
                    zlg7289Download (0,7,0,GusDistance_R_Far%10);                 
                }
                break; 
              case 7:
                if(ir_flag==1)
                {
                    GusDistance_FL_Far=ucIRCheck[0];
                    GusDistance_FR_Far=ucIRCheck[1];
                    ir_flag=0;
                }
                if(key==1)
                {
                    GusDistance_FL_Far+=10;
                    zlg7289Download (0,5,0,GusDistance_FL_Far/100);
                    zlg7289Download (0,6,0,GusDistance_FL_Far/10%10);
                    zlg7289Download (0,7,0,GusDistance_FL_Far%10);
                }
                else if(key==3)
                {
                    GusDistance_FL_Far-=1;
                    zlg7289Download (0,5,0,GusDistance_FL_Far/100);
                    zlg7289Download (0,6,0,GusDistance_FL_Far/10%10);
                    zlg7289Download (0,7,0,GusDistance_FL_Far%10);                 
                }
                else if(key==0)
                {
                    GusDistance_FR_Far+=10;
                    zlg7289Download (0,5,0,GusDistance_FR_Far/100);
                    zlg7289Download (0,6,0,GusDistance_FR_Far/10%10);
                    zlg7289Download (0,7,0,GusDistance_FR_Far%10);                    
                }
                else if(key==2)
                {
                    GusDistance_FR_Far-=1;
                    zlg7289Download (0,5,0,GusDistance_FR_Far/100);
                    zlg7289Download (0,6,0,GusDistance_FR_Far/10%10);
                    zlg7289Download (0,7,0,GusDistance_FR_Far%10);                 
                }
                break; 
              case 8:
                if(ir_flag==1)
                {
                    GusDistance_FL_Near=ucIRCheck[0];
                    GusDistance_FR_Near=ucIRCheck[1];
                    ir_flag=0;
                }
                if(key==1)
                {
                    GusDistance_FL_Near+=10;
                    zlg7289Download (0,5,0,GusDistance_FL_Near/100);
                    zlg7289Download (0,6,0,GusDistance_FL_Near/10%10);
                    zlg7289Download (0,7,0,GusDistance_FL_Near%10);
                }
                else if(key==3)
                {
                    GusDistance_FL_Near-=1;
                    zlg7289Download (0,5,0,GusDistance_FL_Near/100);
                    zlg7289Download (0,6,0,GusDistance_FL_Near/10%10);
                    zlg7289Download (0,7,0,GusDistance_FL_Near%10);                 
                }
                else if(key==0)
                {
                    GusDistance_FR_Near+=10;
                    zlg7289Download (0,5,0,GusDistance_FR_Near/100);
                    zlg7289Download (0,6,0,GusDistance_FR_Near/10%10);
                    zlg7289Download (0,7,0,GusDistance_FR_Near%10);                    
                }
                else if(key==2)
                {
                    GusDistance_FR_Near-=1;
                    zlg7289Download (0,5,0,GusDistance_FR_Near/100);
                    zlg7289Download (0,6,0,GusDistance_FR_Near/10%10);
                    zlg7289Download (0,7,0,GusDistance_FR_Near%10);                 
                }
                break; 
                
                default: 
                break; 
          }
          __delay (200000);
          EXTI_ClearITPendingBit(EXTI_Line3);
        }

      }	
}
*/