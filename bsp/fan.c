/****************************************Copyright (c)****************************************************
** File Name:           fan.c
** Description:         负压风扇/泵控制（TIM4_CH3 / PB8）
**
**   只在冲刺阶段使用；搜索阶段不打开。
**   占空比 = CCR3 / TUNE_FAN_PWM_PERIOD。
*********************************************************************************************************/

#include "fan.h"
#include "mouse_config.h"
#include "stm32f10x.h"

void Fan_Init(void)
{
    RCC->APB1ENR |= 1 << 2;             /* TIM4 时钟 */
    RCC->APB2ENR |= 1 << 3;             /* GPIOB 时钟 */
    RCC->APB2ENR |= 1 << 0;             /* AFIO 时钟 */

    GPIOB->CRH &= 0xFFFFFFF0;           /* 清 PB8 */
    GPIOB->CRH |= 0x0000000B;           /* PB8 = AF 推挽 50MHz */

    AFIO->MAPR &= 0xFFFFEFFF;           /* TIM4_CH3 不重映射 */

    TIM4->CCR3 = 0;
    TIM4->ARR = TUNE_FAN_PWM_PERIOD;
    TIM4->PSC = 0;

    TIM4->CCMR2 |= 6 << 4;              /* CH3 PWM1 */
    TIM4->CCMR2 |= 1 << 3;              /* CH3 预装载 */
    TIM4->CCER  |= 1 << 8;              /* OC3 输出使能 */

    TIM4->CR1 = 0x0080;                 /* ARPE */
    TIM4->CR1 |= 0x01;                  /* 启动 TIM4 */

    Fan_Off();
}

void Fan_SetDuty(uint16 duty)
{
    if (duty > TUNE_FAN_PWM_PERIOD)
        duty = TUNE_FAN_PWM_PERIOD;
    TIM4->CCR3 = duty;
}

void Fan_On(void)
{
    Fan_SetDuty(TUNE_FAN_DUTY);
}

void Fan_Off(void)
{
    TIM4->CCR3 = 0;
}

uint16 Fan_GetDuty(void)
{
    return (uint16)TIM4->CCR3;
}
