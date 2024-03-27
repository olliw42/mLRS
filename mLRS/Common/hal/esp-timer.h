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

volatile uint32_t doSysTask = 0;

typedef enum {
    HAL_TICK_FREQ_10HZ         = 100U,
    HAL_TICK_FREQ_100HZ        = 10U,
    HAL_TICK_FREQ_1KHZ         = 1U,
    HAL_TICK_FREQ_DEFAULT      = HAL_TICK_FREQ_1KHZ
} HAL_TickFreqTypeDef;

typedef enum {
    HAL_OK = 0x00,
    HAL_ERROR = 0x01,
    HAL_BUSY = 0x02,
    HAL_TIMEOUT = 0x03
} HAL_StatusTypeDef;

volatile uint32_t uwTick;
extern uint32_t uwTickPrio;
HAL_TickFreqTypeDef uwTickFreq = HAL_TICK_FREQ_1KHZ;  // For esp we will call tick increment every 1ms


void IRAM_ATTR HAL_IncTick(void) // overwrites __weak declaration in stm32yyxx_hal.c
{
    uwTick += uwTickFreq;
    doSysTask++;
}


volatile uint32_t millis32(void)
{
    return uwTick;
}


//-------------------------------------------------------
// Micros functions
//-------------------------------------------------------
// free running timer with 1us time base

#ifndef MICROS_TIMx
#error MICROS_TIMx not defined !
#endif


//-------------------------------------------------------
// Micros functions
//-------------------------------------------------------

void micros_init(void)
{
    // just a stub on ESP, handled by Arduino init()
}


uint16_t micros16(void)
{
    return micros();
}


//-------------------------------------------------------
// Init function
//-------------------------------------------------------

void timer_init(void)
{
    doSysTask = 0;
    micros_init();
}

#endif // ESP_TIMER_H