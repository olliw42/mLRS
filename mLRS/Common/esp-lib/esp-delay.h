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

const uint8_t CpuFreq = ESP.getCpuFreqMHz();
const float_t cyclesPerNs = float(CpuFreq) / 1000; // cycles per nanosecond

static inline void delay_ns(uint32_t ns)
{
    // This may need some kind of reset to avoid overflowing (28 seconds)
    // ESP.reset();
    uint32_t targetCycles = ESP.getCycleCount() + (ns * cyclesPerNs);
    while (ESP.getCycleCount() <= targetCycles){
        continue;
    }
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