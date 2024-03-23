//*******************************************************
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// my stripped down UART standard library
// only TX, no RX, no halfduplex, no wait and tmo rx functions, no convenience functions, quite a number more strips
//*******************************************************
#ifndef ESPLIB_UARTC_H
#define ESPLIB_UARTC_H


#if defined(UARTC_USE_SERIAL)
#define UARTC_SERIAL_NO Serial
#elif defined(UARTC_USE_SERIAL1)
#define UARTC_SERIAL_NO Serial1
#endif

void uartc_init(void)
{
  UARTC_SERIAL_NO.begin(UARTC_BAUD);
}

uint16_t uartc_putc(char c)
{
  UARTC_SERIAL_NO.write(c);
  return 1;
}


#endif // ESPLIB_UARTC_H
