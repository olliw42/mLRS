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
    uwTick = 0;
    micros_init();
}


#endif // ESP_TIMER_H