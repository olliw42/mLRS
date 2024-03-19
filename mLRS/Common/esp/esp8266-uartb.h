//*******************************************************
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// my stripped down UART standard library
// only TX, no RX, no halfduplex, no wait and tmo rx functions, no convenience functions, quite a number more strips
//*******************************************************

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

#if defined(UARTB_USE_SERIAL)
#define UARTB_SERIAL_NO Serial
#elif defined(UARTB_USE_SERIAL1)
#define UARTB_SERIAL_NO Serial1
#endif

void uartb_init(void)
{
  UARTB_SERIAL_NO.setRxBufferSize(UARTB_RXBUFSIZE);
  UARTB_SERIAL_NO.begin(UARTB_BAUD);
}

void uartb_setprotocol(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
   UARTB_SERIAL_NO.begin(baud);
}

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
  while (UARTB_SERIAL_NO.available()) UARTB_SERIAL_NO.read();
}

static inline void uartb_tx_flush(void)
{
  UARTB_SERIAL_NO.flush();
}

uint16_t uartb_rx_bytesavailable(void)
{
  return UARTB_SERIAL_NO.available();
}

static inline uint16_t uartb_rx_available(void)
{
  if (UARTB_SERIAL_NO.available() > 0) {
    return true;
  } else {
    return false;
  }
}
