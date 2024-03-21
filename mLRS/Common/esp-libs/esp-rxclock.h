//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// ESP Clock
//********************************************************

#ifndef STDESP_RXCLOCK_H
#define STDESP_RXCLOCK_H

#pragma once

#define USING_TIM_DIV1 true

#if defined(ESP8266)
  #include "ESP8266TimerInterrupt.h"
  ESP8266Timer ITimer;
#elif defined(ESP32) 
  #include "ESP32TimerInterrupt.h"
  ESP32Timer ITimer(1);
#endif
// we use a 10us time base, so that overrun is 655 ms
// 65 ms would only be 3 packets
// I have tested it with 1us time base, and it also works fine, but hey
// Note that TIM2 may be 16 bit or 32 bit depending on which STM32 controller is used


#define CLOCK_SHIFT_10US          100 // 75 // 100 // 1 ms


volatile bool doPostReceive;

uint16_t CLOCK_PERIOD_10US; // does not change while isr is enabled, so no need for volatile

//-------------------------------------------------------
// Clock Handler - mimics some STM32 internals
//-------------------------------------------------------

class ClockHandler
{
  public:
    void Init(void){};
    void Do(void);

    uint32_t CNT = 0;
    uint32_t CCR1 = 0;
    uint32_t CCR3 = 0;
    bool CC1_FLAG = true;
    bool CC3_FLAG = true;
};

void ClockHandler::Do(void){
    CNT++;
    if (CNT == CCR1){
        CC1_FLAG = true;
    }
    if (CNT == CCR3){
        CC3_FLAG = true;
    }
}

ClockHandler espTIMER;

//-------------------------------------------------------
// Clock ISR
//-------------------------------------------------------

IRQHANDLER(
#if defined(ESP8266)
IRAM_ATTR void CLOCK_IRQHandler(void)
#elif defined(ESP32)
IRAM_ATTR bool CLOCK_IRQHandler(void * timerNo)
#endif
{
    espTIMER.Do();

    // Call HAL_IncTick every 1ms
    if ((espTIMER.CNT % 100) == 0) {HAL_IncTick();}
    
    if (espTIMER.CC1_FLAG) { // this is at about when RX was or was supposed to be received
        espTIMER.CC1_FLAG = false;
        espTIMER.CCR3 = espTIMER.CCR1 + CLOCK_SHIFT_10US; // next doPostReceive
        espTIMER.CCR1 = espTIMER.CCR1 + CLOCK_PERIOD_10US; // next tick
    }
    if (espTIMER.CC3_FLAG) { // this is 1 ms after RX was or was supposed to be received
        espTIMER.CC3_FLAG = false;
        doPostReceive = true;
    }

    #if defined(ESP32)
    return true;
    #endif

})


//-------------------------------------------------------
// Rx Clock Class
//-------------------------------------------------------

class RxClockBase
{
  public:
    void Init(uint16_t period_ms);
    void SetPeriod(uint16_t period_ms);
    void Reset(void);

    void init_isr_off(void);
    void enable_isr(void);

    uint16_t tim_10us(void);
};


void RxClockBase::Init(uint16_t period_ms)
{
    CLOCK_PERIOD_10US = period_ms * 100; // frame rate in units of 10us
    doPostReceive = false;

    init_isr_off();
    enable_isr();
}


void RxClockBase::SetPeriod(uint16_t period_ms)
{
    CLOCK_PERIOD_10US = period_ms * 100;
}


void RxClockBase::Reset(void)
{
    if (!CLOCK_PERIOD_10US) while (1) {}

    __disable_irq();
    uint32_t CNT = espTIMER.CNT; // works for both 16 and 32 bit timer
    espTIMER.CCR1 = CNT + CLOCK_PERIOD_10US;
    espTIMER.CCR3 = CNT + CLOCK_SHIFT_10US;
    espTIMER.CC1_FLAG = false; // important to do
    espTIMER.CC3_FLAG = false;
    __enable_irq();
}


void RxClockBase::init_isr_off(void)
{
    ITimer.attachInterruptInterval(10, CLOCK_IRQHandler);
    ITimer.disableTimer();
}


void RxClockBase::enable_isr(void)
{
    ITimer.enableTimer();
}


uint16_t RxClockBase::tim_10us(void)
{
    return espTIMER.CNT; // return 16 bit even for 32 bit timer
}

#endif // STDESP_RXCLOCK_H
