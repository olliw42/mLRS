//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// FIFO
//********************************************************
#ifndef FIFO_H
#define FIFO_H
#pragma once


#include <inttypes.h>


class FifoBase
{
  #define SX_FIFO_SIZE  512

  public:
    FifoBase(); // constructor

    void Init(void);

    bool putc(char c);
    void putbuf(void* buf, uint16_t len);

    bool available(void);
    char getc(void);

    void flush(void);

  private:
    uint16_t writepos; // pos at which the next byte will be stored
    uint16_t readpos; // pos at which the oldest byte is fetched
    uint16_t SIZEMASK;
    uint8_t buf[SX_FIFO_SIZE];
};


#endif // FIFO_H
