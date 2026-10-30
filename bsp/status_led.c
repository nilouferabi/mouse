/****************************************Copyright (c)****************************************************
** File Name:           status_led.c
** Description:         状态灯控制（PB12）
**
**   搜索阶段：慢闪；离任一终点格不超过 TUNE_LED_NEAR_GOAL_CELLS 时快闪。
**   冲刺阶段：常亮。
**   由 SysTick_Handler() 每 1 ms 调用 Led_Tick()。
*********************************************************************************************************/

#include "status_led.h"
#include "mouse_config.h"
#include "maze.h"
#include "stm32f10x.h"

static LED_MODE s_mode = LED_MODE_OFF;
static uint16   s_tick = 0;

static void Led_Write(uint8 on)
{
#if TUNE_LED_ACTIVE_HIGH
    if (on)
        GPIO_SetBits(GPIOB, GPIO_Pin_12);
    else
        GPIO_ResetBits(GPIOB, GPIO_Pin_12);
#else
    if (on)
        GPIO_ResetBits(GPIOB, GPIO_Pin_12);
    else
        GPIO_SetBits(GPIOB, GPIO_Pin_12);
#endif
}

static uint8 Led_Dist(int8 x, int8 y, uint8 gx, uint8 gy)
{
    int16 dx = (int16)x - (int16)gx;
    int16 dy = (int16)y - (int16)gy;
    if (dx < 0) dx = -dx;
    if (dy < 0) dy = -dy;
    return (uint8)(dx + dy);
}

static uint8 Led_NearGoal(void)
{
    uint8 d0 = Led_Dist(GmcMouse.cX, GmcMouse.cY, GucXGoal0, GucYGoal0);
    uint8 d1 = Led_Dist(GmcMouse.cX, GmcMouse.cY, GucXGoal0, GucYGoal1);
    uint8 d2 = Led_Dist(GmcMouse.cX, GmcMouse.cY, GucXGoal1, GucYGoal0);
    uint8 d3 = Led_Dist(GmcMouse.cX, GmcMouse.cY, GucXGoal1, GucYGoal1);
    uint8 d = d0;

    if (d1 < d) d = d1;
    if (d2 < d) d = d2;
    if (d3 < d) d = d3;

    return (d <= TUNE_LED_NEAR_GOAL_CELLS) ? 1 : 0;
}

void Led_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    s_mode = LED_MODE_OFF;
    s_tick = 0;
    Led_Write(0);
}

void Led_SetMode(LED_MODE mode)
{
    if (s_mode == mode)
        return;

    s_mode = mode;
    s_tick = 0;

    if (mode == LED_MODE_SPRINT)
        Led_Write(1);
    else
        Led_Write(0);
}

LED_MODE Led_GetMode(void)
{
    return s_mode;
}

void Led_Tick(void)
{
    uint16 on;
    uint16 period;

    if (s_mode == LED_MODE_OFF) {
        Led_Write(0);
        return;
    }

    if (s_mode == LED_MODE_SPRINT) {
        Led_Write(1);
        return;
    }

    if (Led_NearGoal()) {
        on = TUNE_LED_NEAR_ON_MS;
        period = TUNE_LED_NEAR_PERIOD_MS;
    } else {
        on = TUNE_LED_SEARCH_ON_MS;
        period = TUNE_LED_SEARCH_PERIOD_MS;
    }

    s_tick++;
    if (s_tick >= period)
        s_tick = 0;

    Led_Write((s_tick < on) ? 1 : 0);
}
