#include <stdint.h>
#include "protocol_timer.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_tim.h"


#define CPU_F_MHZ 48

void TIM14_IRQHandler(void)
{
    TIM14->SR &= ~(TIM_SR_CC1OF | TIM_SR_CC1IF);
    protocol_timer_irq();
}

void protocol_timer_init(uint16_t period_ms) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14 , ENABLE);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;

    TIM_TimeBaseInitStruct.TIM_Period = period_ms;
    TIM_TimeBaseInitStruct.TIM_Prescaler = CPU_F_MHZ - 1;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0x0000;

    TIM_TimeBaseInit(TIM14, &TIM_TimeBaseInitStruct);
    TIM_ITConfig(TIM14, TIM_IT_CC1, ENABLE);

    NVIC_EnableIRQ(TIM14_IRQn);
    NVIC_SetPriority(TIM14_IRQn, 0);

    TIM_Cmd(TIM14, ENABLE);
}

void protocol_timer_reset()
{

}
