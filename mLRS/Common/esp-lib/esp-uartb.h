//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// ESP UartB
//********************************************************

// these come from the STM files, not sure if they are needed
#define LL_USART_PARITY_NONE 1
#define LL_USART_PARITY_EVEN 1
#define LL_USART_PARITY_ODD 1
#define LL_USART_STOPBITS_0_5 1
#define LL_USART_STOPBITS_1 1
#define LL_USART_STOPBITS_2 1

typedef enum {
    XUART_PARITY_NO = LL_USART_PARITY_NONE, // XUART_xxx to avoid overlap with HAL
    XUART_PARITY_EVEN = LL_USART_PARITY_EVEN,
    XUART_PARITY_ODD = LL_USART_PARITY_ODD,
    XUART_PARITY_MAKEITU32 = UINT32_MAX,
} UARTPARITYENUM;

typedef enum {
    UART_STOPBIT_0_5 = LL_USART_STOPBITS_0_5,
    UART_STOPBIT_1 = LL_USART_STOPBITS_1,
    UART_STOPBIT_2 = LL_USART_STOPBITS_2,
    UART_STOPBIT_MAKEITU32 = UINT32_MAX,
} UARTSTOPBITENUM;


#ifndef ESPLIB_UARTB_H
#define ESPLIB_UARTB_H


#if defined(UARTB_USE_SERIAL)
#define UARTB_SERIAL_NO Serial
#elif defined(UARTB_USE_SERIAL1)
#define UARTB_SERIAL_NO Serial1
#endif


uint16_t uartb_putc(char c)
{
    UARTB_SERIAL_NO.write(c);
    return 1;
}


char uartb_getc(void)
{
    return (char)UARTB_SERIAL_NO.read();
}


static inline void uartb_rx_flush(void)
{
    while (UARTB_SERIAL_NO.available() > 0) UARTB_SERIAL_NO.read();
}


static inline void uartb_tx_flush(void)
{
    UARTB_SERIAL_NO.flush();
}


uint16_t uartb_rx_bytesavailable(void)
{
    return (UARTB_SERIAL_NO.available() > 0) ? UARTB_SERIAL_NO.available() : 0;
}


static inline uint16_t uartb_rx_available(void)
{
    return (UARTB_SERIAL_NO.available() > 0) ? 1 : 0;
}


//-------------------------------------------------------
// Init functions
//-------------------------------------------------------

void uartb_setprotocol(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
    UARTB_SERIAL_NO.begin(baud);
}


void uartb_init(void)
{
    UARTB_SERIAL_NO.setRxBufferSize(UARTB_RXBUFSIZE);
    UARTB_SERIAL_NO.begin(UARTB_BAUD);
}


#endif // ESPLIB_UARTB_H
