//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// ESP SPI Interface
//********************************************************
#ifndef ESPLIB_SPI_H
#define ESPLIB_SPI_H


#include <SPI.h>


//-- select functions

#ifndef SPI_SELECT_PRE_DELAY
  #define SPI_SELECT_PRE_DELAY
#endif
#ifndef SPI_SELECT_POST_DELAY
  #define SPI_SELECT_POST_DELAY
#endif
#ifndef SPI_DESELECT_PRE_DELAY
  #define SPI_DESELECT_PRE_DELAY
#endif
#ifndef SPI_DESELECT_POST_DELAY
  #define SPI_DESELECT_POST_DELAY
#endif


#ifdef SPI_CS_IO

IRAM_ATTR static inline void spi_select(void)
{
    SPI_SELECT_PRE_DELAY;
    GPOC = (1 << SPI_CS_IO); // digitalWrite(SPI_CS_IO, LOW); // CS = low
    SPI_SELECT_POST_DELAY;
}


IRAM_ATTR static inline void spi_deselect(void)
{
    SPI_DESELECT_PRE_DELAY;
    GPOS = (1 << SPI_CS_IO); // digitalWrite(SPI_CS_IO, HIGH); // CS = high
    SPI_DESELECT_POST_DELAY;
}

#endif // #ifdef SPI_CS_IO


//-- transmit, transfer, read, write functions

// these are blocking
// utilize ESP SPI buffer
// transferBytes()
// - does while(SPI1CMD & SPIBUSY) {} as needed
// - if dataout or datain are not aligned, then it does memcpy to aligned buffer on stack 
// - sends 0xFFFFFFFF if no out data!! Problem: sx datasheet says 0x00

IRAM_ATTR static inline void spi_transfer(const uint8_t* dataout, uint8_t* datain, const uint8_t len)
{
    SPI.transferBytes(dataout, datain, len);
}


IRAM_ATTR static inline void spi_read(uint8_t* datain, const uint8_t len)
{
    SPI.transferBytes(nullptr, datain, len);
}


IRAM_ATTR static inline void spi_write(const uint8_t* dataout, uint8_t len)
{
    SPI.transferBytes(dataout, nullptr, len);
}


//-------------------------------------------------------
// INIT routines
//-------------------------------------------------------

void spi_setnop(uint8_t nop)
{
    // currently not supported, should be 0x00 fo sx, is 0xFF per ESP SPI library
}


void spi_init(void)
{
#ifdef SPI_CS_IO
    pinMode(SPI_CS_IO, OUTPUT);
#endif    

    SPI.begin();
    SPI.setFrequency(SPI_FREQUENCY);
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
}


#endif // ESPLIB_SPI_H