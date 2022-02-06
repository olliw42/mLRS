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


#define CLOCK_TIMx                TIM4
#define CLOCK_IRQn                TIM4_IRQn
#define CLOCK_IRQHandler          TIM4_IRQHandler
//#define CLOCK_IRQ_PRIORITY        10

#define CLOCK_PERIOD_10US         (FRAME_RATE_MS*100)
#define CLOCK_SHIFT_10US          75 // 100 // 1 ms


volatile bool doPostReceive;


//-------------------------------------------------------
// Clock Class
//-------------------------------------------------------

class ClockBase
{
  public:
    void Init(void);
    void InitIsrOff(void);
    void EnableIsr(void);

    uint16_t tim_10us(void);

    void Reset(void);
};


void ClockBase::InitIsrOff(void)
{
    tim_init_up(CLOCK_TIMx, 0xFFFF, TIMER_BASE_10US);

    LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {};
    TIM_OC_InitStruct.CompareValue = 100; // start in 1 ms;
    LL_TIM_OC_Init(CLOCK_TIMx, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct); // pll.tick()

    TIM_OC_InitStruct.CompareValue = 200; // only needs to be later than OC1
    LL_TIM_OC_Init(CLOCK_TIMx, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct); // doPostReceive

    nvic_irq_enable_w_priority(CLOCK_IRQn, CLOCK_IRQ_PRIORITY);
}


void ClockBase::EnableIsr(void)
{
    LL_TIM_EnableIT_CC1(CLOCK_TIMx);
    LL_TIM_EnableIT_CC3(CLOCK_TIMx);
}


void ClockBase::Init(void)
{
  InitIsrOff();
  EnableIsr();
}


void ClockBase::Reset(void)
{
    __disable_irq();
    uint16_t CNT = CLOCK_TIMx->CNT;
    CLOCK_TIMx->CCR1 = CNT + CLOCK_PERIOD_10US;
    CLOCK_TIMx->CCR3 = CNT + CLOCK_SHIFT_10US;
    LL_TIM_ClearFlag_CC1(CLOCK_TIMx); // important to do
    LL_TIM_ClearFlag_CC3(CLOCK_TIMx);
    __enable_irq();
}


uint16_t ClockBase::tim_10us(void)
{
    return CLOCK_TIMx->CNT;
}


//-------------------------------------------------------
// Clock ISR
//-------------------------------------------------------

IRQHANDLER(
void CLOCK_IRQHandler(void)
{
  if (LL_TIM_IsActiveFlag_CC1(CLOCK_TIMx)) { // this is at about when RX was or was supposed to be received
    LL_TIM_ClearFlag_CC1(CLOCK_TIMx);
    CLOCK_TIMx->CCR3 = CLOCK_TIMx->CCR1 + CLOCK_SHIFT_10US;
    CLOCK_TIMx->CCR1 = CLOCK_TIMx->CCR1 + CLOCK_PERIOD_10US;
    //LED_GREEN_ON;
  }
  if (LL_TIM_IsActiveFlag_CC3(CLOCK_TIMx)) { // this is 1 ms after RX was or was supposed to be received
    LL_TIM_ClearFlag_CC3(CLOCK_TIMx);
    doPostReceive = true;
  }
})


#endif // CLOCK_H
