//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// ESP UartC
//********************************************************
#ifndef ESPLIB_UARTC_H
#define ESPLIB_UARTC_H


#if defined(UARTC_USE_SERIAL)
#define UARTC_SERIAL_NO Serial
#elif defined(UARTC_USE_SERIAL1)
#define UARTC_SERIAL_NO Serial1
#endif


uint16_t uartc_putc(char c)
{
    UARTC_SERIAL_NO.write(c);
    return 1;
}


//-------------------------------------------------------
// Init functions
//-------------------------------------------------------

void uartc_init(void)
{
    UARTC_SERIAL_NO.begin(UARTC_BAUD);
}


#endif // ESPLIB_UARTC_H
