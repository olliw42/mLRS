//*******************************************************
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// my stripped down UART standard library
// only TX, no RX, no halfduplex, no wait and tmo rx functions, no convenience functions, quite a number more strips
//*******************************************************
// Interface:
//
// #define SWUART_USE_TIM1/2/3/4/5/15/16/17 // requires a timer, with 8 MHz or 10 MHz ticks, uses OC1/2 irq
// #define SWUART_TX_IO
//
// #define SWUART_DONOTSETUPTIMER     // do not set up timer in init()
// #define SWUART_DONOTSETUPTX        // do not set up TX pin in init()
//
// #define SWUART_BAUD
// #define SWUART_INVERTED
//
// #define SWUART_USE_TX
// #define SWUART_TXBUFSIZE
//
// #define SWUART_TIM_IRQ_PRIORITY
//
//*******************************************************

#ifndef STDESP8266_UARTSW_H
#define STDESP8266_UARTSW_H

#include <SoftwareSerial.h>

EspSoftwareSerial::UART swUart;

//-------------------------------------------------------
// Defines
//-------------------------------------------------------

#ifndef SWUART_BAUD
  #define SWUART_BAUD           57600
#endif

//-------------------------------------------------------
// TX routines
//-------------------------------------------------------
#ifdef SWUART_USE_TX

uint16_t swuart_putc(char c) {
  swUart.write(c);
  return 0;
}

#endif

void swuart_init(void)
{
  swUart.begin(57600, EspSoftwareSerial::SWSERIAL_8N1, -1, D3, false, 95);
  swUart.println("Serial init");
}

#endif // STDESP8266_UARTSW_H

