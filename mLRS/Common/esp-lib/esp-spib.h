//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// ESP SPI Interface
//********************************************************
#ifndef ESPLIB_spib_H
#define ESPLIB_spib_H


#include <SPI.h>


//-- select functions

#ifdef SX2_CS_IO

IRAM_ATTR static inline void spib_select(void)
{
    gpio_low(SX2_CS_IO);
}

IRAM_ATTR static inline void spib_deselect(void)
{
    gpio_high(SX2_CS_IO);
}

#endif // #ifdef SX2_CS_IO


//-- transmit, transfer, read, write functions

// these are blocking
// utilize ESP SPI buffer
// transferBytes()
// - does while(SPI1CMD & SPIBUSY) {} as needed
// - if dataout or datain are not aligned, then it does memcpy to aligned buffer on stack 
// - sends 0xFFFFFFFF if no out data!! Problem: sx datasheet says 0x00

IRAM_ATTR static inline void spib_transfer(const uint8_t* dataout, uint8_t* datain, const uint8_t len)
{
#if defined(ESP32) 
    spiTransferBytesNL(SPI.bus(), dataout, datain, len);
#elif defined(ESP8266)
    SPI.transferBytes(dataout, datain, len);
#endif    
}


IRAM_ATTR static inline void spib_read(uint8_t* datain, const uint8_t len)
{
#if defined(ESP32) 
    spiTransferBytesNL(SPI.bus(), nullptr, datain, len);
#elif defined(ESP8266)
    SPI.transferBytes(nullptr, datain, len);
#endif   
}


IRAM_ATTR static inline void spib_write(const uint8_t* dataout, uint8_t len)
{
#if defined(ESP32) 
    spiTransferBytesNL(SPI.bus(), dataout, nullptr, len);
#elif defined(ESP8266)
    SPI.transferBytes(dataout, nullptr, len);
#endif 
}


//-------------------------------------------------------
// INIT routines
//-------------------------------------------------------

void spib_setnop(uint8_t nop)
{
    // currently not supported, should be 0x00 fo sx, is 0xFF per ESP SPI library
}


void spib_init(void)
{
#ifdef SX2_CS_IO
    gpio_init(SX2_CS_IO, IO_MODE_OUTPUT_PP_HIGH);
#endif    
}


#endif // ESPLIB_spib_H