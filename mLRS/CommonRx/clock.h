//*******************************************************
// MLRS project
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// Clock
//********************************************************
#ifndef CLOCK_H
#define CLOCK_H
#pragma once

// we use a 10us time base, so that overrun is 655 ms
// 65 ms would only be 3 packets
// I have also tested it with 1us time base, and it also works fine
// however, with 1us I just couldn't get the PLL to converge as fast as with 10us !?!?


#define CLOCK_TIMx                TIM3
#define CLOCK_IRQn                TIM3_IRQn
#define CLOCK_IRQHandler          TIM3_IRQHandler
//#define CLOCK_IRQ_PRIORITY        10


#define CLOCK_OC3_SHIFT_10US      100 // 1 ms


volatile uint16_t clock_oc_period;


class ClockBase
{
  public:
    void InitIsrOff(uint16_t oc_period_us);
    void EnableIsr(void);
};


void ClockBase::InitIsrOff(uint16_t oc_period_us)
{
    tim_init_up(CLOCK_TIMx, 0xFFFF, TIMER_BASE_10US);

    clock_oc_period = oc_period_us / 10;

    LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {};
    TIM_OC_InitStruct.CompareValue = 100;  // start in 1 ms;
    LL_TIM_OC_Init(CLOCK_TIMx, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct); // pll.tick()

    TIM_OC_InitStruct.CompareValue = 200; // only needs to be later than OC1
    LL_TIM_OC_Init(CLOCK_TIMx, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct); // pll.update()

    TIM_OC_InitStruct.CompareValue = 200; // only needs to be later than OC1
    LL_TIM_OC_Init(CLOCK_TIMx, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct); // do_post_receive

    nvic_irq_enable_w_priority(CLOCK_IRQn, CLOCK_IRQ_PRIORITY);
}


void ClockBase::EnableIsr(void)
{
    LL_TIM_EnableIT_CC1(CLOCK_TIMx);
    LL_TIM_EnableIT_CC2(CLOCK_TIMx);
    LL_TIM_EnableIT_CC3(CLOCK_TIMx);
}



#endif // CLOCK_H
