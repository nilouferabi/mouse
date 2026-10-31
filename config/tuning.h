/****************************************Copyright (c)****************************************************
** File Name:           tuning.h
** Description:         电脑鼠“重点参数”集中配置（Single Source of Tuning）
**
**   本文件由 mouse_config.h 在末尾自动包含，所有 .c 都能直接使用。
**   原则：
**     1. 这里只放“调车时会改的数值”，硬件寄存器初始化细节仍留在 .c；
**     2. 每个宏后面注明单位/量纲，数值与重构前逐字一致（行为不变）；
**     3. 改参数优先改这里，不要再回到动作函数里翻魔数。
**
**   分组：
**     [1] 机械与编码器        [2] 速度环 / PID
**     [3] 直行贴墙            [4] 90 度转向
**     [5] 掉头                [6] 搜索段衔接
**     [7] 红外与 ADC          [8] 陀螺仪
**     [9] 八向冲刺            [10] 流程 / 外设 / 存储
**     [11] 时间基准
*********************************************************************************************************/

#ifndef __TUNING_H
#define __TUNING_H

/*======================================================================================================
  [1] 机械与编码器
======================================================================================================*/
#define TUNE_ENCODER_CENTER             32768       /* 编码器软件零点（TIM2/TIM3 计数中点）        */
#define TUNE_PWM_PERIOD                 2880        /* TIM1 ARR+1，25 kHz；占空比上限见 U_MAX      */
#define TUNE_PULSE_PER_BLOCK            ONEBLOCK    /* 走一格迷宫所需编码器脉冲                    */
#define TUNE_TURN_PULSE_MARGIN          200         /* 判定“动作完成”时允许的脉冲余量              */
#define TUNE_TURN_HANDOFF_PULSE         30000       /* 转弯结束后交给下一段的编码器基准脉冲        */
#define TUNE_SEARCH_HANDOFF_PULSE       40000       /* 搜索段转弯后重设计数器的基准脉冲            */
#define TUNE_TURN_PULSE_PER_DEG         2886.67     /* 每 1 度对应的转向脉冲（右转/搜索左转）      */
#define TUNE_TURN_PULSE_PER_DEG_L       286.67      /* 左转 _C 版本原值，疑为 2886.67 漏写一位，   */
                                                    /* 先按原值保留，改动前请实车验证              */

/*======================================================================================================
  [2] 速度环 / PID
    速度单位：每 1 ms 编码器脉冲数；PWM 单位：TIM1 CCR 计数
======================================================================================================*/
#define TUNE_PID_KP                     20          /* 比例（原 __KP）                              */
#define TUNE_PID_KI                     0.1         /* 积分（原 __KI）                              */
#define TUNE_PID_KD                     0          /* 微分（原 __KD）                              */
#define TUNE_PID_U_MAX                  1400        /* PWM 输出上限；TIM1 ARR=2880 → 最大 48.6%     */
#define TUNE_PID_U_MIN                  0           /* PWM 输出下限                                 */
#define TUNE_PID_ERROR_IMAX             10          /* 积分分离阈值：|误差|>10 时关闭积分           */
#define TUNE_PID_DEADBAND               3           /* 速度环死区                                   */
#define TUNE_PID_WALL_FEEDFORWARD       0.5         /* 贴墙差速前馈系数（__WPIDContr）              */
#define TUNE_PID_I_LIMIT                30000       /* 积分项硬限幅，防止 int16 溢出（仅防溢出）    */

#define TUNE_SPEED_ACCEL_STEP           10          /* 每 5 ms 增加的设定速度                       */
#define TUNE_SPEED_ACCEL_DIV            5           /* 加速分频：k=(k+1)%5                          */
#define TUNE_SPEED_DECEL_STEP           2           /* 每 10 ms 减少的设定速度                      */
#define TUNE_SPEED_DECEL_DIV            10          /* 减速分频：k=(k+1)%10                         */
#define TUNE_SPEED_DECEL_MARGIN         3           /* 减速时允许的速度正偏差                       */
#define TUNE_SPEED_HOLD                 3           /* GuiSpeedCtr=3：保持当前速度，不加速不减速    */
#define TUNE_SPEED_STOP                 5           /* GuiSpeedCtr=5：停车后不再自动调速            */

/*======================================================================================================
  [3] 直行贴墙（SysTick 1 ms 环）
    距离单位：红外查表得到的 mm；增益单位：PWM 计数 / mm
======================================================================================================*/
#define TUNE_WALL_L_ACTIVE_DIST         120         /* 左侧距离小于此值才参与贴墙                  */
#define TUNE_WALL_L_TARGET_DIST         75          /* 左侧目标距离                                */
#define TUNE_WALL_L_GAIN                1           /* 左侧差速增益                                */
#define TUNE_WALL_R_ACTIVE_DIST         170         /* 右侧距离小于此值才参与贴墙                  */
#define TUNE_WALL_R_TARGET_DIST         100         /* 右侧目标距离                                */
#define TUNE_WALL_R_GAIN                1           /* 右侧差速增益（代码里带负号）                */

/*======================================================================================================
  [4] 90 度转向
    W / GW：陀螺积分判据（原始计数）；脉冲：编码器计数
======================================================================================================*/
#define TUNE_TURN_SPEED                 145         /* 转向时前进速度设定 sRef                     */
#define TUNE_TURN_WPID_L                95          /* 左转方向环设定（代码取正）                  */
#define TUNE_TURN_WPID_R                95          /* 右转方向环设定（代码取负）                  */

#define TUNE_TURN_L_TARGET_DIST         95          /* 右转前用左墙算偏置的目标距离                */
#define TUNE_TURN_L_CLEAR_DIST          155         /* 左侧距离大于此值认为无墙，偏置清零          */
#define TUNE_TURN_R_TARGET_DIST         90          /* 左转前用右墙算偏置的目标距离                */
#define TUNE_TURN_R_CLEAR_DIST          145         /* 右侧距离大于此值认为无墙，偏置清零          */
#define TUNE_TURN_PIANCHA_DIV           10          /* 偏置 = (距离 - 目标)/10                     */
#define TUNE_TURN_PIANCHA_MAX           2           /* 偏置限幅 ±2                                 */

#define TUNE_TURN90_PULSE_SEARCH_R      17000       /* 搜索段右转后续直行脉冲                      */
#define TUNE_TURN90_PULSE_SEARCH_L      21000       /* 搜索段左转后续直行脉冲                      */
#define TUNE_TURN90_PULSE_C_R           17000       /* _C 右转后续直行脉冲                         */
#define TUNE_TURN90_PULSE_C_L           9600        /* _C 左转后续直行脉冲                         */
#define TUNE_TURN90_W_SEARCH_R          95000       /* 搜索段右转陀螺判据                          */
#define TUNE_TURN90_W_SEARCH_L          86500       /* 搜索段左转陀螺判据                          */
#define TUNE_TURN90_W_C_R               93000       /* _C 右转陀螺判据                             */
#define TUNE_TURN90_W_C_L               84000       /* _C 左转陀螺判据                             */

#define TUNE_TURN_Y_W_R                 92600       /* 遇岔路原地右转判据                          */
#define TUNE_TURN_Y_W_L                 90000       /* 遇岔路原地左转判据                          */
#define TUNE_TURN_Y_PULSE_R             27000       /* 原地右转后续直行脉冲                        */
#define TUNE_TURN_Y_PULSE_L             25000       /* 原地左转后续直行脉冲                        */
#define TUNE_TURN_Y_SPEED               120         /* 原地转后前进速度                            */
#define TUNE_TURN_WPID_Y                80          /* 原地转方向环设定                            */

/*======================================================================================================
  [5] 掉头 / 回起点
======================================================================================================*/
#define TUNE_TURNBACK_APPROACH_PULSE    1000        /* 前方有墙时先走的对中脉冲                    */
#define TUNE_TURNBACK_SPEED             50          /* 掉头前对中速度（mouseTurnback）             */
#define TUNE_TURNBACK_SPEED_QIDIAN      150         /* 回起点版对中速度                            */
#define TUNE_TURNBACK_WPID              90          /* 掉头方向环设定                              */
#define TUNE_TURNBACK_WPID_QI           80          /* qi/_H 版本方向环设定                        */
#define TUNE_TURNBACK_GW                290000      /* 掉头陀螺判据（mouseTurnback/_H）            */
#define TUNE_TURNBACK_GW_QI             280000      /* qi 版本掉头判据                             */
#define TUNE_TURNBACK_GW_QIDIAN         300000      /* 回起点版掉头判据                            */
#define TUNE_TURNBACK_RETURN_SPEED      150         /* 掉头后回程速度                              */
#define TUNE_TURNBACK_RETURN_SPEED_H    100         /* _H 版本回程速度                             */
#define TUNE_TURNBACK_RETURN_PULSE      23000       /* 掉头后回程脉冲（mouseTurnback/qidian）      */
#define TUNE_TURNBACK_RETURN_PULSE_QI   26000       /* qi 版本回程脉冲                             */
#define TUNE_TURNBACK_RETURN_PULSE_H    3000        /* _H 版本回程脉冲                             */
#define TUNE_TURNBACK_BACK_PULSE_H      100         /* _H 版本倒车脉冲                             */
#define TUNE_TURNBACK_BACK_SPEED        (-40)       /* 倒车速度（qi 版本）                         */
#define TUNE_TURNBACK_BACK_SPEED_H      (-20)       /* 倒车速度（_H 版本）                         */
#define TUNE_TURNBACK_SETTLE_LOOPS      5000000     /* 掉头后的空循环稳定时间                      */
#define TUNE_TURNBACK_END_MARK          23000       /* 掉头结束时写入 uiPulseCtr 的基准值          */
#define TUNE_ONESTEP3_PULSE             7000        /* 冲刺收尾微调脉冲                            */

/*======================================================================================================
  [6] 搜索段格子衔接
======================================================================================================*/
#define TUNE_SEARCH_TURN_BLEND_BOTH     16000       /* 左右前都有远墙时的接弯补偿                  */
#define TUNE_SEARCH_TURN_BLEND_ONE      12000       /* 只有一侧有墙时的接弯补偿                    */
#define TUNE_SEARCH_TURN_CORR_L         (-800)      /* 右转接左岔路的单侧修正                      */
#define TUNE_SEARCH_TURN_CORR_R         (-4000)     /* 左转接右岔路的单侧修正                      */
#define TUNE_SEARCH_LEFT_ADVANCE        18000       /* 左侧岔路：再前进多少脉冲到路口中心          */
#define TUNE_SEARCH_RIGHT_ADVANCE       19500       /* 右侧岔路：再前进多少脉冲到路口中心          */
#define TUNE_SEARCH_SIDE_MARGIN         400         /* 岔路判定/停车的时间余量（脉冲）             */
#define TUNE_SEARCH_GO_ADVANCE_LEFT     19500       /* 直行时左侧岔路的接弯补偿                    */
#define TUNE_SEARCH_GO_ADVANCE_RIGHT    18800       /* 直行时右侧岔路的接弯补偿                    */
#define TUNE_SEARCH_GO_ADVANCE_SAME     13000       /* 同向衔接补偿                                                */
#define TUNE_SEARCH_GO_ADVANCE_TURN     11000       /* 转向后衔接补偿                                              */
#define TUNE_SEARCH_GO_MARGIN           100         /* 搜索直行岔路检测余量                        */
#define TUNE_GO_SPEED_CORNER            120         /* 单格前进（含转弯衔接）速度                  */
#define TUNE_GO_SPEED_IDLE              150         /* 非单格但未到巡航时的速度                    */
#define TUNE_GO_SPEED_CRUISE            200         /* 多格直行巡航速度                            */
#define TUNE_GO_SPEED_SLOW              120         /* 接近岔路时的减速设定                        */
#define TUNE_PULSE_OVERRUN_MARGIN       15000       /* 超程判定余量（PulseCtrCheck）               */

/*======================================================================================================
  [7] 红外与 ADC
    阈值单位：ADC 原始计数（越大表示越近）
======================================================================================================*/
#define TUNE_ADC_VREF_MV                3300        /* ADC 参考电压 mV                             */
#define TUNE_ADC_FULL_SCALE             4096        /* 12 位 ADC 满量程                            */
#define TUNE_IR_R_SAT                   35000       /* 右红外饱和上限                              */
#define TUNE_IR_FRONT_SAT               30000       /* 前左/前右红外饱和上限                       */
#define TUNE_IR_R_OFFSET_MV             300         /* 右侧查表起点 mV                             */
#define TUNE_IR_R_STEP_MV               15          /* 右侧查表步长 mV                             */
#define TUNE_IR_FILTER_SIZE             8           /* 陀螺滑动窗口长度                            */
#define TUNE_IR_FILTER_SUM_COUNT        6           /* 去最大最小后参与平均的点数                  */
#define TUNE_IR_STATE_COUNT             8           /* TIM6 红外轮询状态数                         */
/* 红外阈值上电默认值（可由 __ir_Get() 从 Flash 覆盖；当前默认不加载） */
#define TUNE_IR_L_NEAR_DEFAULT          1951
#define TUNE_IR_L_MID_DEFAULT           1200
#define TUNE_IR_L_FAR_DEFAULT           450
#define TUNE_IR_R_NEAR_DEFAULT          1693
#define TUNE_IR_R_MID_DEFAULT           1000
#define TUNE_IR_R_FAR_DEFAULT           302
#define TUNE_IR_FL_NEAR_DEFAULT         3076
#define TUNE_IR_FR_NEAR_DEFAULT         2835
#define TUNE_IR_FL_FAR_DEFAULT          386
#define TUNE_IR_FR_FAR_DEFAULT          484

/*======================================================================================================
  [8] 陀螺仪
======================================================================================================*/
#define TUNE_GYRO_ZERO_DEFAULT          2041        /* 陀螺零位（ADC 计数）                        */
#define TUNE_GYRO_GW_SCALE              5121        /* GW → W 的换算系数                           */
#define TUNE_GYRO_SENSITIVITY_MV        1055        /* 陀螺灵敏度（mV/(°/s)）                      */
#define TUNE_GYRO_SYSTEM_PARAM          3.96        /* 系统参数（原注释）                          */
#define TUNE_GYRO_ANGLE_RANGE           360         /* 角度换算分母                                */
#define TUNE_GYRO_CAL_SAMPLES           3000        /* 上电零漂校准采样点数                        */
#define TUNE_GYRO_CAL_TIMEOUT_MS        5000        /* 上电零漂校准超时                            */
#define TUNE_GYRO_TRAPEZOID             0.5         /* 陀螺积分梯形系数                            */
#define TUNE_CONTROL_DT_S               0.001       /* 控制周期 1 ms（积分用）                     */

/******************************************************************************************************
  [9] 八向冲刺（4 向搜索完成后的斜线加速）
    方向编号：0=N, 1=NE, 2=E, 3=SE, 4=S, 5=SW, 6=W, 7=NW
======================================================================================================*/
#define TUNE_8DIR_ENABLE                1           /* 1: 冲刺阶段启用 8 邻域 + 斜向；0: 保持 4 向 */
#define TUNE_8DIR_SPEED_ORTHO           180         /* 正交直行最高速（每 1 ms 脉冲）              */
#define TUNE_8DIR_SPEED_DIAG            150         /* 斜向直行最高速                              */
#define TUNE_8DIR_TURN_SPIN             120         /* 原地转向方向环设定（差速 PWM）              */
#define TUNE_8DIR_TURN_TRIM_R           0.95        /* 右转角度修正：实车转不够就调大              */
#define TUNE_8DIR_TURN_TRIM_L           0.95        /* 左转角度修正                                */
#define TUNE_8DIR_TURN_TIMEOUT_MS       2000        /* 单次转向超时                                */
#define TUNE_8DIR_MOVE_TIMEOUT_MS       6000        /* 单格直行超时（低速也不至于卡死）            */
#define TUNE_8DIR_GYRO_FF               3           /* 斜向直行的陀螺航向保持增益                  */
#define TUNE_8DIR_FRONT_SAFETY          1           /* 1: 斜向时双前近距触发立即停车               */
#define TUNE_8DIR_STOP_DELAY_LOOPS      20000       /* 每格停车后的机械稳定空循环                  */
#define TUNE_8DIR_PULSE_ORTHO           ONEBLOCK    /* 正交一格脉冲                                */
#define TUNE_8DIR_PULSE_DIAG            ONEBLOCK_45 /* 斜向一格脉冲                                */

/*======================================================================================================
  [10] 流程 / 外设 / 存储
======================================================================================================*/
#define TUNE_START_PRESS_SEARCH         1           /* START 按 1 次：搜索                        */
#define TUNE_START_PRESS_SPURT_MIN      3           /* START 按 3~4 次：读 Flash 地图直接冲刺      */
#define TUNE_START_PRESS_SPURT_MAX      5
#define TUNE_GOAL_X0                    8           /* 全迷宫中心目标 X（运行期覆盖值）            */
#define TUNE_GOAL_X1                    7
#define TUNE_START_MARK                 15          /* Flash 中“起点在左上角”的标记                */
#define TUNE_FLASH_MAP_WORDS            256         /* 地图存档半字数                              */
#define TUNE_FAN_DUTY                   600         /* 负压风扇冲刺占空比（0..3600）               */
#define TUNE_FAN_PWM_PERIOD             3600        /* 负压风扇 TIM4 ARR                           */
#define TUNE_LED_SEARCH_ON_MS           100         /* 搜索慢闪：亮 100 ms                         */
#define TUNE_LED_SEARCH_PERIOD_MS       1000        /* 搜索慢闪：周期 1 s                          */
#define TUNE_LED_NEAR_ON_MS             80          /* 近终点快闪：亮 80 ms                        */
#define TUNE_LED_NEAR_PERIOD_MS         160         /* 近终点快闪：周期 160 ms                     */
#define TUNE_LED_NEAR_GOAL_CELLS        2           /* 曼哈顿距离 ≤ 2 格算“近终点”                 */
#define TUNE_LED_ACTIVE_HIGH            1           /* 1: PB12 高电平点亮；0: 低电平点亮            */
#define TUNE_UART_BAUD                  9600        /* USART3 波特率                               */
#define TUNE_UART_IRQ_PREEMPT           3           /* USART3 抢占优先级                           */
#define TUNE_UART_IRQ_SUB               2           /* USART3 子优先级                             */
#define TUNE_DELAY_BOOT                 100000      /* 上电等待                                    */
#define TUNE_DELAY_SENSOR_REFRESH       10000       /* WAIT 状态传感器刷新间隔                     */
#define TUNE_DELAY_SETTLE               10000000    /* 长稳定等待                                  */
#define TUNE_DELAY_POST_ACTION          1000000     /* 动作后短等待                                */
#define TUNE_DELAY_FLASH_READ           10000       /* Flash 读取后等待                            */

/*======================================================================================================
  [11] 时间基准
======================================================================================================*/
#define TUNE_SYSTICK_HZ                 1000        /* SysTick 1 kHz                               */
#define TUNE_TIM6_PRESCALER             143         /* TIM6 PSC                                    */
#define TUNE_TIM6_PERIOD                99          /* TIM6 ARR → 5 kHz                            */
#define TUNE_ENCODER_FILTER_L           6           /* TIM2 编码器输入滤波                         */
#define TUNE_ENCODER_FILTER_R           5           /* TIM3 编码器输入滤波                         */
#define TUNE_ADC_CHANNELS               5           /* ADC 扫描通道数                              */

#endif
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
