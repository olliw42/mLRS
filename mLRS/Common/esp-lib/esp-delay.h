//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// ESP Delay
//********************************************************
#ifndef ESPLIB_DELAY_H
#define ESPLIB_DELAY_H
#pragma once


static inline void delay_ns(uint32_t ns)
{
    // called only in SPI functions, we are almost certain this is not needed for operation on ESP
}

IRAM_ATTR static inline void delay_us(uint32_t us)
{
    delayMicroseconds(us);
}

IRAM_ATTR static inline void delay_ms(uint32_t ms) 
{
    delay(ms);
}


//-------------------------------------------------------
// INIT routines
//-------------------------------------------------------

void delay_init(void) 
{
    // not needed on the ESP chips, built into Arduino init
}


#endif // ESPLIB_DELAY_H