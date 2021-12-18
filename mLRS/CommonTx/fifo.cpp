//*******************************************************
// MLRS project
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// FIFO
//********************************************************

#include "fifo.h"


FifoBase::FifoBase() // constructor
{
  Init();
}


void FifoBase::Init(void)
{
  writepos = readpos = 0;
  SIZEMASK = SX_FIFO_SIZE - 1;
}


bool FifoBase::putc(char c)
{
  uint16_t next = (writepos + 1) & (SIZEMASK);
  if (next != readpos) { // fifo not full
    buf[writepos] = c;
    writepos = next;
    return true;
  }
  return false;
}


void FifoBase::putbuf(void* buf, uint16_t len)
{
uint16_t i;

  for (i = 0; i < len; i++) putc(((char*)buf)[i]);
}


bool FifoBase::available(void)
{
int16_t d;

  d = (int16_t)writepos - (int16_t)readpos;
  if (d < 0) return d + (SIZEMASK + 1);
  return d;
}


char FifoBase::getc(void)
{
  if (writepos != readpos) { // fifo not empty
    char c = buf[readpos];
    readpos = (readpos + 1) & (SIZEMASK);
    return c;
  }
  return 0;
}
