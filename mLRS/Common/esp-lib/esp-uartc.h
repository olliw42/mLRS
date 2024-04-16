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

// These come from the STM files, not sure if they are needed
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


#if defined(UARTC_USE_SERIAL)
#define UARTC_SERIAL_NO Serial
#elif defined(UARTC_USE_SERIAL1)
#define UARTC_SERIAL_NO Serial1
#endif

void uartc_init(void)
{
  UARTC_SERIAL_NO.begin(UARTC_BAUD);
}

void uartc_setprotocol(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
   UARTC_SERIAL_NO.begin(baud);
}

uint16_t uartc_putc(char c)
{
  UARTC_SERIAL_NO.write(c);
  return 1;
}

char uartc_getc(void)
{
  return (char)UARTC_SERIAL_NO.read();
}

static inline void uartc_rx_flush(void)
{
  while (UARTC_SERIAL_NO.available() > 0) UARTC_SERIAL_NO.read();
}

static inline void uartc_tx_flush(void)
{
  UARTC_SERIAL_NO.flush();
}

uint16_t uartc_rx_bytesavailable(void)
{
  return (UARTC_SERIAL_NO.available() > 0) ? UARTC_SERIAL_NO.available() : 0;
}

static inline uint16_t uartc_rx_available(void)
{
  return (UARTC_SERIAL_NO.available() > 0) ? 1 : 0;
}

#endif // ESPLIB_UARTC_H
