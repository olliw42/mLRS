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

#ifdef SPI_CS_IO

IRAM_ATTR static inline void spi_select(void)
{
    gpio_low(SPI_CS_IO);
}

IRAM_ATTR static inline void spi_deselect(void)
{
    gpio_high(SPI_CS_IO);
}

#endif // #ifdef SPI_CS_IO

#if defined DEVICE_HAS_DIVERSITY_SINGLE_SPI && defined SX2_CS_IO

IRAM_ATTR static inline void spi2_select(void)
{
    gpio_low(SX2_CS_IO);
}

IRAM_ATTR static inline void spi2_deselect(void)
{
    gpio_high(SX2_CS_IO);
}

#endif // #if defined DEVICE_HAS_DIVERSITY_SINGLE_SPI && defined SX2_CS_IO


//-- transmit, transfer, read, write functions

// these are blocking
// utilize ESP SPI buffer
// transferBytes()
// - does while(SPI1CMD & SPIBUSY) {} as needed
// - if dataout or datain are not aligned, then it does memcpy to aligned buffer on stack 
// - sends 0xFFFFFFFF if no out data!! Problem: sx datasheet says 0x00

IRAM_ATTR static inline void spi_transfer(const uint8_t* dataout, uint8_t* datain, const uint8_t len)
{
#if defined(ESP32) 
    spiTransferBytesNL(SPI.bus(), dataout, datain, len);
#elif defined(ESP8266)
    SPI.transferBytes(dataout, datain, len);
#endif    
}


IRAM_ATTR static inline void spi_read(uint8_t* datain, const uint8_t len)
{
#if defined(ESP32) 
    spiTransferBytesNL(SPI.bus(), nullptr, datain, len);
#elif defined(ESP8266)
    SPI.transferBytes(nullptr, datain, len);
#endif   
}


IRAM_ATTR static inline void spi_write(const uint8_t* dataout, uint8_t len)
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

void spi_setnop(uint8_t nop)
{
    // currently not supported, should be 0x00 fo sx, is 0xFF per ESP SPI library
}


void spi_init(void)
{
#ifdef SPI_CS_IO
    gpio_init(SPI_CS_IO, IO_MODE_OUTPUT_PP_HIGH);
#endif

#if defined DEVICE_HAS_DIVERSITY_SINGLE_SPI && defined SX2_CS_IO
    gpio_init(SX2_CS_IO, IO_MODE_OUTPUT_PP_HIGH);
#endif

#if defined(ESP32)
    spiEndTransaction(SPI.bus()); 
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_CS_IO);
#elif defined(ESP8266)
    SPI.begin();
#endif 
    SPI.setFrequency(SPI_FREQUENCY);
    SPI.setDataMode(SPI_MODE0);

#if defined(ESP32)
    SPI.setBitOrder(SPI_MSBFIRST);
#elif defined(ESP8266)
    SPI.setBitOrder(MSBFIRST);
#endif 


#if defined(ESP32)
    spiSimpleTransaction(SPI.bus());
#endif
    
}


#endif // ESPLIB_SPI_H