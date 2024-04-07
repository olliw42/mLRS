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

static inline void spi_select(void)
{
    SPI_SELECT_PRE_DELAY;
    digitalWrite(SPI_CS_IO, LOW); // CS = low
    SPI_SELECT_POST_DELAY;
}


static inline void spi_deselect(void)
{
    SPI_DESELECT_PRE_DELAY;
    digitalWrite(SPI_CS_IO, HIGH); // CS = high
    SPI_DESELECT_POST_DELAY;
}

#endif // #ifdef SPI_CS_IO


//-- transmit, transfer, read, write functions

// to utilize ESP SPI Buffer
IRAM_ATTR static inline void spi_transferbytes(uint8_t* dataout, uint8_t* datain, uint8_t len)
{
    SPI.transferBytes(dataout, datain, len);
}


// is blocking
IRAM_ATTR uint8_t spi_transmitchar(uint8_t c)
{
    return SPI.transfer(c);
}


// is blocking
IRAM_ATTR void spi_transfer(uint8_t* dataout, uint8_t* datain, uint16_t len)
{
    while (len) {
        *datain = spi_transmitchar(*dataout);
        dataout++;
        datain++;
        len--;
    }
}


//-------------------------------------------------------
// INIT routines
//-------------------------------------------------------

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