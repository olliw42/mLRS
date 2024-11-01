//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// Timer (millis, micros, systask)
//********************************************************
#ifndef TIMER_H
#define TIMER_H
#pragma once


//-------------------------------------------------------
// SysTask & millis32()  functions
//-------------------------------------------------------

#define SYSTICK_DELAY_MS(x)       (uint16_t)(((uint32_t)(x)*(uint32_t)1000)/SYSTICK_TIMESTEP)


volatile uint32_t doSysTask = 0;


void HAL_IncTick(void) // overwrites __weak declaration in stm32yyxx_hal.c
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


void micros_init(void)
{
    tim_init_1us_freerunning(MICROS_TIMx);
}


uint16_t micros16(void)
{
    return MICROS_TIMx->CNT;
}


// needs to be called not later than every 65 ms
// to ensure the overflow counter is updated properly
uint64_t micros64(void)
{
static uint64_t overflow_cnt = 0;
static uint16_t last_cnt;

    uint16_t cnt = MICROS_TIMx->CNT;
    if (cnt < last_cnt) {
        overflow_cnt += 0x10000;
    }
    last_cnt = cnt;
    return overflow_cnt + cnt;
}


//-------------------------------------------------------
// Init function
//-------------------------------------------------------

void timer_init(void)
{
    doSysTask = 0;
    micros_init();
}


#endif // TIMER_H
