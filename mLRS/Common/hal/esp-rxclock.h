//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// ESP RxClock
//********************************************************
// architecture difference vs STM32 (rxclock.h):
// - STM32 uses two compare channels: CCR1 fires at frame period boundaries,
//   CCR3 fires CLOCK_SHIFT later to trigger doPostReceive
// - ESP simplifies this to a single event timer that multiplexes the 1ms HAL
//   tick and rx event, scheduling whichever is next
//
// benefits of on-demand scheduling vs fixed-interval polling:
// - old 10µs ISR fired 100k times/sec 
// - new approach fires no more than 1111 times/sec (1ms tick + rx events).
// - timer resources: uses 1 timer for both ESP32 and ESP8266
//
// underlying timer hardware:
// - ESP32: esp_timer uses a 64-bit µs counter
// - ESP8266: timer1 is a 23-bit countdown timer at 5MHz, 0.2µs resolution
//********************************************************
#ifndef ESP_RXCLOCK_H
#define ESP_RXCLOCK_H
#pragma once

#include <Arduino.h>

#ifdef ESP32
#include <esp_timer.h>
#endif

//-------------------------------------------------------
// Platform abstractions
//-------------------------------------------------------

#ifdef ESP32
  typedef int64_t tick_t;
  #define GET_MICROS()            esp_timer_get_time()
  #define ENTER_CRITICAL_ISR()    taskENTER_CRITICAL_ISR(&rx_spinlock)
  #define EXIT_CRITICAL_ISR()     taskEXIT_CRITICAL_ISR(&rx_spinlock)
  #define ENTER_CRITICAL_API()    taskENTER_CRITICAL(&rx_spinlock)
  #define EXIT_CRITICAL_API()     taskEXIT_CRITICAL(&rx_spinlock)
#elif defined ESP8266
  typedef uint32_t tick_t;        // wraps at ~71 minutes
  #define GET_MICROS()            micros()
  #define ENTER_CRITICAL_ISR()    // not needed in ISR on ESP8266
  #define EXIT_CRITICAL_ISR()
  #define ENTER_CRITICAL_API()    noInterrupts()
  #define EXIT_CRITICAL_API()     interrupts()
#endif

// time comparison: ESP32 uses simple >= (64-bit), ESP8266 needs signed diff for wraparound
#ifdef ESP32
  #define TIME_GE(now, target)  ((now) >= (target))
#elif defined ESP8266
  #define TIME_GE(now, target)  ((int32_t)((now) - (target)) >= 0)
#endif

//-------------------------------------------------------
// Constants and variables
//-------------------------------------------------------

#define CLOCK_SHIFT_US  1000 // 1 ms

volatile bool doPostReceive;

static uint32_t CLOCK_PERIOD_US;
static volatile tick_t next_tick_us;
static volatile tick_t next_rx_us;

#ifdef ESP32
static esp_timer_handle_t event_timer;
static portMUX_TYPE rx_spinlock = portMUX_INITIALIZER_UNLOCKED;
#endif

//-------------------------------------------------------
// Timer scheduling
//-------------------------------------------------------

static void IRAM_ATTR schedule_next(tick_t now)
{
#ifdef ESP32
    tick_t diff_tick = next_tick_us - now;
    tick_t diff_rx = next_rx_us - now;
#elif defined ESP8266
    // cast to signed: uint32_t subtraction can underflow near timer wrap
    int32_t diff_tick = (int32_t)(next_tick_us - now);
    int32_t diff_rx = (int32_t)(next_rx_us - now);
#endif

    if (diff_tick < 10) diff_tick = 10;
    if (diff_rx < 10) diff_rx = 10;

    tick_t delay_us = (diff_tick < diff_rx) ? diff_tick : diff_rx;

#ifdef ESP32
    esp_timer_start_once(event_timer, delay_us);
#elif defined ESP8266
    timer1_write(delay_us * 5);
#endif
}

//-------------------------------------------------------
// Timer callback (shared logic)
//-------------------------------------------------------

static void IRAM_ATTR event_callback(void)
{
    ENTER_CRITICAL_ISR();

    tick_t now = GET_MICROS();

    // fire tick if due
    if (TIME_GE(now, next_tick_us)) {
        HAL_IncTick();
        next_tick_us += 1000;
        while (TIME_GE(GET_MICROS(), next_tick_us)) {
            next_tick_us += 1000;
            HAL_IncTick();
        }
    }

    // fire rx event if due
    if (TIME_GE(now, next_rx_us)) {
        doPostReceive = true;
        next_rx_us += CLOCK_PERIOD_US;
        while (TIME_GE(GET_MICROS(), next_rx_us)) {
            next_rx_us += CLOCK_PERIOD_US;
        }
    }

    schedule_next(GET_MICROS());

    EXIT_CRITICAL_ISR();
}

// wrappers: esp_timer requires void(*)(void*), timer1 requires void(*)(void)
#ifdef ESP32
static void IRAM_ATTR esp32_timer_callback(void* arg) { event_callback(); }
#elif defined ESP8266
void IRAM_ATTR timer1_isr() { event_callback(); }
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
    CLOCK_PERIOD_US = period_ms * 1000;
    doPostReceive = false;

    if (initialized) return;
    initialized = true;

    tick_t now = GET_MICROS();
    next_tick_us = now + 1000;
    next_rx_us = now + CLOCK_PERIOD_US;

#ifdef ESP32
    const esp_timer_create_args_t args = {
            .callback = &esp32_timer_callback,
            .name = "rxclock"
    };
    if (esp_timer_create(&args, &event_timer) != ESP_OK) {
        while (1) {} // fatal: timer creation failed, halt
    }
#elif defined ESP8266
    timer1_attachInterrupt(timer1_isr);
    timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
#endif

    schedule_next(now);
}


IRAM_ATTR void tRxClock::SetPeriod(uint16_t period_ms)
{
    ENTER_CRITICAL_API();
    CLOCK_PERIOD_US = period_ms * 1000;
    EXIT_CRITICAL_API();
}


IRAM_ATTR void tRxClock::Reset(void)
{
    ENTER_CRITICAL_API();

#ifdef ESP32
    // esp_timer_start_once fails if already running; timer1_write just overwrites
    esp_timer_stop(event_timer);
#endif

    tick_t now = GET_MICROS();
    next_rx_us = now + CLOCK_SHIFT_US;
    schedule_next(now);

    EXIT_CRITICAL_API();
}


#endif // ESP_RXCLOCK_H
