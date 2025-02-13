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

// interrupts on ESP32 are slow (~2 us) so having a 10 us / 100 kHz timer interrupt generates a lot of overhead
// 2 us * 100 kHz = 200000 us = 200 ms, every second !!!
// the arduino task (which is used by mLRS) runs on core 1, so move the interrupt to core 0
// but with the interrupt on the other core need to make sure all shared resources are protected
// to make things a little simpler, keep the 1 ms uWtick timer interrupt on core 1 (this has 100x less overhead, so ok)
// the following variables are shared across cores (used in the isr on core 0) and need to be protected with spinlocks:
//     CNT_10us, CCR1, CCR3, CLOCK_PERIOD_10US
// CLOCK_SHIFT_10US is a define, so not included
// doPostReceive is only potentially read / write by either Core at the same time, so okay to not use spinlock


#define CLOCK_SHIFT_10US          100 // 75 // 100 // 1 ms
#define CLOCK_CNT_1MS             100 // 10us interval 10us x 100 = 1000us


#ifdef ESP32
  static portMUX_TYPE esp32_spinlock = portMUX_INITIALIZER_UNLOCKED;
#endif


volatile bool doPostReceive;

uint16_t CLOCK_PERIOD_10US; // does not change while isr is enabled, so no need for volatile

volatile uint32_t CNT_10us = 0;
volatile uint32_t CCR1 = CLOCK_PERIOD_10US;
volatile uint32_t CCR3 = CLOCK_PERIOD_10US;
volatile uint32_t MS_C = CLOCK_CNT_1MS;


//-------------------------------------------------------
// Clock ISR
//-------------------------------------------------------

#ifdef ESP32
IRQHANDLER(
void CLOCK1MS_IRQHandler(void)
{
    HAL_IncTick();
})
    
IRQHANDLER(
void CLOCK10US_IRQHandler(void)
{
    taskENTER_CRITICAL_ISR(&esp32_spinlock);

    CNT_10us++;

    // this is at about when RX was or was supposed to be received
    if (CNT_10us == CCR1) {
        CCR3 = CNT_10us + CLOCK_SHIFT_10US; // next doPostReceive
        CCR1 = CNT_10us + CLOCK_PERIOD_10US; // next tick
    }

    // this is 1 ms after RX was or was supposed to be received
    if (CNT_10us == CCR3) {
        doPostReceive = true;
    }

    taskEXIT_CRITICAL_ISR(&esp32_spinlock);
})
#elif defined ESP8266
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
#endif


//-------------------------------------------------------
// RxClock Class
//-------------------------------------------------------

class tRxClock
{
  public:
    void Init(uint16_t period_ms);
    void SetPeriod(uint16_t period_ms);
    void Reset(void);

  private:
    bool initialized = false;
};


void tRxClock::Init(uint16_t period_ms)
{
    CLOCK_PERIOD_10US = period_ms * 100; // frame rate in units of 10us
    doPostReceive = false;

    CNT_10us = 0;
    CCR1 = CLOCK_PERIOD_10US;
    CCR3 = CLOCK_SHIFT_10US;
    MS_C = CLOCK_CNT_1MS;

    if (initialized) return;

    // initialize the timer(s)
#ifdef ESP32
    // initialize the 1 ms uwTick timer
    hw_timer_t* timer0_cfg = nullptr;
    timer0_cfg = timerBegin(0, 800, 1);  // Timer 0, APB clock is 80 Mhz | divide by 800 is 100 KHz / 10 us, count up
    timerAttachInterrupt(timer0_cfg, &CLOCK1MS_IRQHandler, true);
    timerAlarmWrite(timer0_cfg, 100, true); // 10 us * 100 = 1 ms
    timerAlarmEnable(timer0_cfg);

    // initialize the 10 us doPostReceive timer, put it on Core 0
    xTaskCreatePinnedToCore([](void *parameter) {
        hw_timer_t* timer1_cfg = nullptr;
        timer1_cfg = timerBegin(1, 800, 1);  // Timer 1, APB clock is 80 Mhz | divide by 800 is 100 KHz / 10 us, count up
        timerAttachInterrupt(timer1_cfg, &CLOCK10US_IRQHandler, true);
        timerAlarmWrite(timer1_cfg, 1, true);
        timerAlarmEnable(timer1_cfg);
        vTaskDelete(NULL);
    }, "TimerSetup", 2048, NULL, 1, NULL, 0);  // last argument here is Core 0, ignored on ESP32C3
#elif defined ESP8266
    timer1_attachInterrupt(CLOCK_IRQHandler);
    timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);
    timer1_write(50); // 5 MHz (5 ticks/us - 1677721.4 us max), 50 ticks = 10us
#endif
    initialized = true;
}


IRAM_ATTR void tRxClock::SetPeriod(uint16_t period_ms)
{
#ifdef ESP32
    taskENTER_CRITICAL(&esp32_spinlock);
#endif
    CLOCK_PERIOD_10US = period_ms * 100;
#ifdef ESP32
    taskEXIT_CRITICAL(&esp32_spinlock);
#endif
}


IRAM_ATTR void tRxClock::Reset(void)
{
    if (!CLOCK_PERIOD_10US) while(1){}

#ifdef ESP32
    taskENTER_CRITICAL(&esp32_spinlock);
#elif defined ESP8266
    noInterrupts();
#endif
    CCR1 = CNT_10us + CLOCK_PERIOD_10US;
    CCR3 = CNT_10us + CLOCK_SHIFT_10US;
    MS_C = CNT_10us + CLOCK_CNT_1MS;  // MS_C only used on ESP8266
#ifdef ESP32
    taskEXIT_CRITICAL(&esp32_spinlock);
#elif defined ESP8266
    interrupts();
#endif
}


#endif // ESP_RXCLOCK_H
