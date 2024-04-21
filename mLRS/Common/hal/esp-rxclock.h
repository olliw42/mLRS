//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// ESP RxClock
//********************************************************
#ifndef ESP_RXCLOCK_H
#define ESP_RXCLOCK_H
#pragma once


#define CLOCK_SHIFT_10US          100 // 75 // 100 // 1 ms
#define CLOCK_CNT_1MS             100 // 10us interval 10us x 100 = 1000us        

volatile bool doPostReceive;

uint16_t CLOCK_PERIOD_10US; // does not change while isr is enabled, so no need for volatile

volatile uint32_t CNT_10us = 0;
volatile uint32_t CCR1 = CLOCK_PERIOD_10US;
volatile uint32_t CCR3 = CLOCK_PERIOD_10US;
volatile uint32_t MS_C = CLOCK_CNT_1MS;


//-------------------------------------------------------
// Clock ISR
//-------------------------------------------------------

IRQHANDLER(
void CLOCK_IRQHandler(void)
{
    CNT_10us++;

    // call HAL_IncTick every 1 ms
    if (CNT_10us == MS_C) {
        MS_C = CNT_10us + CLOCK_CNT_1MS; 
        HAL_IncTick();
    }

    // this is at about when RX was or was supposed to be received
    if (CNT_10us == CCR1) { 
        CCR3 = CNT_10us + CLOCK_SHIFT_10US; // next doPostReceive
        CCR1 = CNT_10us + CLOCK_PERIOD_10US; // next tick
    }

    // this is 1 ms after RX was or was supposed to be received
    if (CNT_10us == CCR3) { 
        doPostReceive = true;
    }

})


//-------------------------------------------------------
// RxClock Class
//-------------------------------------------------------

class RxClockBase
{
  public:
    void Init(uint16_t period_ms);
    void SetPeriod(uint16_t period_ms);
    void Reset(void);
    void disable_isr(void);
};


void RxClockBase::Init(uint16_t period_ms)
{
    CLOCK_PERIOD_10US = period_ms * 100; // frame rate in units of 10us
    doPostReceive = false;

    CNT_10us = 0;
    CCR1 = CLOCK_PERIOD_10US;
    CCR3 = CLOCK_SHIFT_10US;
    MS_C = CLOCK_CNT_1MS;

    // initialise the timer
    timer1_attachInterrupt(CLOCK_IRQHandler); 
    timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);
    timer1_write(50); // 5 MHz (5 ticks/us - 1677721.4 us max), 50 ticks = 10us
}


void RxClockBase::disable_isr(void)
{
    timer1_detachInterrupt(); 
}


void RxClockBase::SetPeriod(uint16_t period_ms)
{
    CLOCK_PERIOD_10US = period_ms * 100;
}


void RxClockBase::Reset(void)
{
    if (!CLOCK_PERIOD_10US) while (1) {}

    noInterrupts();
    CCR1 = CNT_10us + CLOCK_PERIOD_10US;
    CCR3 = CNT_10us + CLOCK_SHIFT_10US;
    MS_C = CNT_10us + CLOCK_CNT_1MS;
    interrupts();
}


#endif // ESP_RXCLOCK_H
