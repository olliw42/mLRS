//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// Clock
//********************************************************
#ifndef CLOCK_H
#define CLOCK_H
#pragma once

// we use a 10us time base, so that overrun is 655 ms
// 65 ms would only be 3 packets
// I have tested it with 1us time base, and it also works fine, but hey
// Note that TIM2 may be 16 bit or 32 bit depending on which STM32 controller is used


// TIM1, TIM4 may be used by buzzer
#define CLOCK_TIMx                TIM2
#define CLOCK_IRQn                TIM2_IRQn
#define CLOCK_IRQHandler          TIM2_IRQHandler
//#define CLOCK_IRQ_PRIORITY        10

#define CLOCK_SHIFT_10US          100 // 75 // 100 // 1 ms


volatile bool doPostReceive;

uint16_t CLOCK_PERIOD_10US; // does not change while isr is enabled, so no need for volatile


//-------------------------------------------------------
// Clock Class
//-------------------------------------------------------

class ClockBase
{
  public:
    void Init(void);
    void Reset(void);

    void init_isr_off(void);
    void enable_isr(void);

    uint16_t tim_10us(void);
};


void ClockBase::Init(void)
{
    CLOCK_PERIOD_10US = ((uint16_t)Config.frame_rate_ms * 100);
    doPostReceive = false;

    init_isr_off();
    enable_isr();
}


void ClockBase::Reset(void)
{
    if (!CLOCK_PERIOD_10US) while (1) {}

    __disable_irq();
    uint32_t CNT = CLOCK_TIMx->CNT; // works for both 16 and 32 bit timer
    CLOCK_TIMx->CCR1 = CNT + CLOCK_PERIOD_10US;
    CLOCK_TIMx->CCR3 = CNT + CLOCK_SHIFT_10US;
    LL_TIM_ClearFlag_CC1(CLOCK_TIMx); // important to do
    LL_TIM_ClearFlag_CC3(CLOCK_TIMx);
    __enable_irq();
}


void ClockBase::init_isr_off(void)
{
    tim_init_up(CLOCK_TIMx, 0xFFFFFFFF, TIMER_BASE_10US); // works for both 16 and 32 bit timer

    LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {};
    TIM_OC_InitStruct.CompareValue = 100; // start in 1 ms;
    LL_TIM_OC_Init(CLOCK_TIMx, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct); // pll.tick()

    TIM_OC_InitStruct.CompareValue = 200; // only needs to be later than OC1
    LL_TIM_OC_Init(CLOCK_TIMx, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct); // doPostReceive

    nvic_irq_enable_w_priority(CLOCK_IRQn, CLOCK_IRQ_PRIORITY);
}


void ClockBase::enable_isr(void)
{
    LL_TIM_EnableIT_CC1(CLOCK_TIMx);
    LL_TIM_EnableIT_CC3(CLOCK_TIMx);
}


uint16_t ClockBase::tim_10us(void)
{
    return CLOCK_TIMx->CNT; // return 16 bit even for 32 bit timer
}


//-------------------------------------------------------
// Clock ISR
//-------------------------------------------------------

IRQHANDLER(
void CLOCK_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_CC1(CLOCK_TIMx)) { // this is at about when RX was or was supposed to be received
        LL_TIM_ClearFlag_CC1(CLOCK_TIMx);
        CLOCK_TIMx->CCR3 = CLOCK_TIMx->CCR1 + CLOCK_SHIFT_10US; // next doPostReceive
        CLOCK_TIMx->CCR1 = CLOCK_TIMx->CCR1 + CLOCK_PERIOD_10US; // next tick
        //LED_GREEN_ON;
    }
    if (LL_TIM_IsActiveFlag_CC3(CLOCK_TIMx)) { // this is 1 ms after RX was or was supposed to be received
        LL_TIM_ClearFlag_CC3(CLOCK_TIMx);
        doPostReceive = true;
    }
})


#endif // CLOCK_H
