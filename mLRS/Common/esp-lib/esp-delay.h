//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// ESP Delay
//********************************************************
#ifndef ESPLIB_DELAY_H
#define ESPLIB_DELAY_H

void delay_init() 
{
    // Not needed on the ESP chips, this built into Arduino init.
}

static inline void delay_ns(uint32_t ns)
{
    // We are almost certain this is not needed for SPI operation on ESP
}

static inline void delay_us(uint32_t us)
{
    delayMicroseconds(us);
}

static inline void delay_ms(uint32_t ms) 
{
    delay(ms);
}

#endif // ESPLIB_DELAY_H