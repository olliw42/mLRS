
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

void uartb_init(void)
{

}

void uartb_setprotocol(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{

}

uint16_t uartb_putc(char c)
{
  return 1;
}


char uartb_getc(void)
{
  return 'a';
}

static inline void uartb_rx_flush(void)
{

}

static inline void uartb_tx_flush(void)
{

}

uint16_t uartb_rx_bytesavailable(void)
{
  return 1;
}



static inline uint16_t uartb_rx_available(void)
{
  return 1;
}
