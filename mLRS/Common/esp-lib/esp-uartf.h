//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// ESP UartF
//********************************************************
#ifndef ESPLIB_UARTF_H
#define ESPLIB_UARTF_H


#if defined(UARTF_USE_SERIAL)
#define UARTF_SERIAL_NO Serial
#elif defined(UARTF_USE_SERIAL1)
#define UARTF_SERIAL_NO Serial1
#endif


uint16_t uartf_putc(char c)
{
    UARTF_SERIAL_NO.write(c);
    return 1;
}


//-------------------------------------------------------
// INIT routines
//-------------------------------------------------------

void uartf_init(void)
{
    UARTF_SERIAL_NO.begin(UARTF_BAUD);
}


#endif // ESPLIB_UARTF_H
