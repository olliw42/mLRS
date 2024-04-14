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

static inline IRAM_ATTR void spi_select(void)
{
#if defined(ESP32)
  SPI.beginTransaction(SPISettings(SPI_FREQUENCY, MSBFIRST, SPI_MODE0)); 
  GPIO.out_w1tc = ((uint32_t)1 << SX_NSS);
#elif defined(ESP8266)
  GPOC = (1 << SX_NSS);
#endif  
}

static inline IRAM_ATTR void spi_deselect(void)
{
#if defined(ESP32)
  GPIO.out_w1ts = ((uint32_t)1 << SX_NSS);
  SPI.endTransaction(); 
#elif defined(ESP8266)
  GPOS = (1 << SX_NSS);
#endif
}


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
  pinMode(SX_NSS, OUTPUT);
  digitalWrite(SX_NSS, HIGH);

#if defined(ESP32) 
  SPI.begin(SCK, MISO, MOSI, SX_NSS);
#elif defined(ESP8266)
  SPI.begin();
  SPI.setFrequency(SPI_FREQUENCY);
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
#endif 
}


#endif // ESPLIB_SPI_H