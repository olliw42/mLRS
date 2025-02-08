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

    // initialize the timer
#ifdef ESP32
    xTaskCreatePinnedToCore([](void *parameter)
    {
        hw_timer_t* timer0_cfg = nullptr;
        timer0_cfg = timerBegin(0, 800, 1);  // Timer 0, APB clock is 80 Mhz | divide by 800 is 100 KHz / 10 us, count up
        timerAttachInterrupt(timer0_cfg, &CLOCK_IRQHandler, true);
        timerAlarmWrite(timer0_cfg, 1, true);
        timerAlarmEnable(timer0_cfg);

        vTaskDelete(NULL);
    },
        "TimerSetup",  // Task name
        2048,          // Stack size
        NULL,          // Parameters
        1,             // Priority
        NULL,          // Task handle
        0              // Core ID, 1 is used by Arduino.  Ignored on ESP32C3
    );
#elif defined ESP8266
    timer1_attachInterrupt(CLOCK_IRQHandler);
    timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);
    timer1_write(50); // 5 MHz (5 ticks/us - 1677721.4 us max), 50 ticks = 10us
#endif
    initialized = true;
}


IRAM_ATTR void tRxClock::SetPeriod(uint16_t period_ms)
{
    CLOCK_PERIOD_10US = period_ms * 100;
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
    MS_C = CNT_10us + CLOCK_CNT_1MS;
#ifdef ESP32
    taskEXIT_CRITICAL(&esp32_spinlock);
#elif defined ESP8266
    interrupts();
#endif
}


#endif // ESP_RXCLOCK_H
