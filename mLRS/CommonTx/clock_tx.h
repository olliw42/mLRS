//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// Tx Clock
//********************************************************
#ifndef CLOCK_TX_H
#define CLOCK_TX_H
#pragma once

// we use the MICROS timer, which is used in hal/timer.h, for non blocking delay stuff
// it is free running at 1us, so overrun is every 65.5 ms
#include "../Common/hal/timer.h"


#ifndef MICROS_TIMx
  #error MICROS_TIMx not defined !
#endif

// sadly, #ifdef MICROS_TIMx == TIM3 does not work. So crude approach of
// explicit name and using ## concatenation.
#define _APPEND(x,y)          x ## y
#define APPEND(x,y)           _APPEND(x,y)

#define TXCLOCK_TIMx          MICROS_TIMx
#define TXCLOCK_IRQn          APPEND(MICROS_TIM_NAMEPREFIX,IRQn) // TIM3_IRQn
#define TXCLOCK_IRQHandler    APPEND(MICROS_TIM_NAMEPREFIX,IRQHandler) // TIM3_IRQHandler

#ifndef TXCLOCK_IRQ_PRIORITY
  #define TXCLOCK_IRQ_PRIORITY  15 // use lowest as default
#endif


//-------------------------------------------------------
// Tx Clock Class
//-------------------------------------------------------

class tTxClock
{
  public:
    void Init(void);

    void SetCC1Callback(void (*callback)(void));
    bool HasCC1Callback(void) { return (cc1_callback_ptr != nullptr); }
    void StartCC1Delay(uint16_t delay_us);

    // don't use
    void clear_cc1_isr(void);
    void (*cc1_callback_ptr)(void);
};


void tTxClock::Init(void)
{
    cc1_callback_ptr = nullptr;

    clear_cc1_isr();
    nvic_irq_enable_w_priority(TXCLOCK_IRQn, TXCLOCK_IRQ_PRIORITY);
}


void tTxClock::SetCC1Callback(void (*callback)(void))
{
    cc1_callback_ptr = callback;
}


void tTxClock::StartCC1Delay(uint16_t delay_us)
{
    if (cc1_callback_ptr == nullptr) return;

    // ensure CC1 interrupt is disabled and cleared from any previous state
    clear_cc1_isr();

    TXCLOCK_TIMx->CCR1 = (uint16_t)(TXCLOCK_TIMx->CNT + delay_us);

    LL_TIM_EnableIT_CC1(TXCLOCK_TIMx);
}


void tTxClock::clear_cc1_isr(void)
{
    LL_TIM_DisableIT_CC1(MICROS_TIMx);
    LL_TIM_ClearFlag_CC1(MICROS_TIMx);
}


tTxClock txclock;


//-------------------------------------------------------
// Tx Clock ISR
//-------------------------------------------------------

IRQHANDLER(
void TXCLOCK_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_CC1(TXCLOCK_TIMx)) {
        LL_TIM_ClearFlag_CC1(TXCLOCK_TIMx);
        LL_TIM_DisableIT_CC1(TXCLOCK_TIMx);
        // now call callback
        txclock.cc1_callback_ptr();
    }
})


#endif // CLOCK_TX_H
