//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// ESP Timer
//********************************************************
#ifndef ESP_TIMER_H
#define ESP_TIMER_H
#pragma once


//-------------------------------------------------------
// SysTask & millis32()  functions
//-------------------------------------------------------

#define SYSTICK_DELAY_MS(x)       (uint16_t)(((uint32_t)(x)*(uint32_t)1000)/SYSTICK_TIMESTEP)


// Two variables are required to avoid race with HAL_IncTick() when tasks may take longer than one tick
volatile uint32_t doSysTask = 0; // Only changed in ISR; incremented to signal tick
uint32_t doneSysTask = 0; // Never changed in ISR; incremented when (doSysTask != doneSysTask)

volatile uint32_t uwTick = 0;


IRAM_ATTR void HAL_IncTick(void)
{
    uwTick += 1;
    doSysTask++;
}


volatile uint32_t millis32(void)
{
    return uwTick;
}


#ifdef DEVICE_IS_TRANSMITTER
// for receiver it is done in esp-rxclock.h, for transmitter it is here

#ifdef ESP32
static portMUX_TYPE esp32_spinlock = portMUX_INITIALIZER_UNLOCKED;
static bool systick_millis_initialized = false;
#endif

#define CLOCK_CNT_1MS             100 // 10us interval 10us x 100 = 1000us        

volatile uint32_t CNT_10us = 0;
volatile uint32_t MS_C = 0;

IRQHANDLER(
void CLOCK_IRQHandler(void)
{
    HAL_IncTick();
})


void systick_millis_init(void)
{
    CNT_10us = 0;
    MS_C = CLOCK_CNT_1MS;

    // initialize the timer
#ifdef ESP32
    if (systick_millis_initialized) return;

    hw_timer_t* timer0_cfg = nullptr;
    timer0_cfg = timerBegin(0, 80, 1);  // Timer 0, APB clock is 80 Mhz | divide by 80 is 1 MHz / 1 us, count up
    timerAttachInterrupt(timer0_cfg, &CLOCK_IRQHandler, true);
    timerAlarmWrite(timer0_cfg, 1000, true);
    timerAlarmEnable(timer0_cfg);

    systick_millis_initialized = true;

#elif defined ESP8266
    timer1_attachInterrupt(CLOCK_IRQHandler); 
    timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);
    timer1_write(50); // 5 MHz (5 ticks/us - 1677721.4 us max), 50 ticks = 10us
#endif
}

#else

void systick_millis_init(void) {}

#endif


//-------------------------------------------------------
// Micros functions
//-------------------------------------------------------
// free running timer with 1us time base

void micros_init(void)
{
    // just a stub on ESP, handled by Arduino init()
}


uint16_t micros16(void)
{
    return (uint16_t)micros();
}


//-------------------------------------------------------
// Init function
//-------------------------------------------------------

void timer_init(void)
{
    doSysTask = 0;
    doneSysTask = 0;
    uwTick = 0;
    systick_millis_init();
    micros_init();
}


#endif // ESP_TIMER_H
