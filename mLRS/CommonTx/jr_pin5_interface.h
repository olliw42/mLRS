//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// JR Pin5 Interface Header
//********************************************************
#ifndef JRPIN5_INTERFACE_H
#define JRPIN5_INTERFACE_H
#pragma once


#include "..\Common\hal\hal.h" // not needed but helps editor to get defines correct LOL


//-------------------------------------------------------
// Interface Implementation

void uart_rx_callback(uint8_t c);
void uart_tc_callback(void);

#define UART_RX_CALLBACK_FULL(c)    uart_rx_callback(c)
#define UART_TC_CALLBACK()          uart_tc_callback()

#include "..\modules\stm32ll-lib\src\stdstm32-uart.h"


void uart_putc_tobuf(char c)
{
  uint16_t next = (uart_txwritepos + 1) & UART_TXBUFSIZEMASK;
  if (uart_txreadpos != next) { // fifo not full //this is isr safe, works also if readpos has changed in the meanwhile
      uart_txbuf[next] = c;
      uart_txwritepos = next;
  }
}


void uart_tx_start(void)
{
  LL_USART_EnableIT_TXE(UART_UARTx); // initiates transmitting
}


class tPin5BridgeBase
{
  public:
    void Init(void);

    // interface to the uart hardware peripheral used for the bridge
    void transmit_enable(bool enable_flag);
    bool mb_rx_available(void) { return uart_rx_available(); }
    char mb_getc(void) { return uart_getc(); }
    void mb_putc(char c) { uart_putc_tobuf(c); }

    // for in-isr processing
    virtual void parse_nextchar(uint8_t c, uint16_t tnow_us);
    virtual bool transmit_start(void); // returns true if transmission should be started

    typedef enum {
      STATE_IDLE = 0,

      STATE_MBRIDGE_RECEIVE_STX2,
      STATE_MBRIDGE_RECEIVE_LEN,
      STATE_MBRIDGE_RECEIVE_SERIALPACKET,
      STATE_MBRIDGE_RECEIVE_CHANNELPACKET,
      STATE_MBRIDGE_RECEIVE_COMMANDPACKET,

      STATE_RECEIVE_CRSF_LEN,
      STATE_RECEIVE_CRSF_PAYLOAD,
      STATE_RECEIVE_CRSF_CRC,

      STATE_TRANSMIT_START,
      STATE_TRANSMITING,
    } STATE_ENUM;

    uint8_t state;
    uint8_t len;
    uint8_t cnt;
    uint16_t tlast_us;
};


void tPin5BridgeBase::Init(void)
{
#if defined MBRIDGE_TX_XOR || defined MBRIDGE_RX_XOR
  gpio_init(MBRIDGE_TX_XOR, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_VERYFAST);
  gpio_init(MBRIDGE_RX_XOR, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_VERYFAST);
  MBRIDGE_TX_SET_INVERTED;
  MBRIDGE_RX_SET_INVERTED;
#endif

#if defined MBRIDGE_TX_OE
  gpio_init(MBRIDGE_TX_OE, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_VERYFAST);
  MBRIDGE_TX_OE_DISABLED;
#endif

  uart_init_isroff();

#if defined MBRIDGE_RX_TX_INVERT_INTERNAL
  LL_USART_Disable(UART_UARTx);
  LL_USART_SetTXPinLevel(MBRIDGE_UARTx, LL_USART_TXPIN_LEVEL_INVERTED);
  LL_USART_SetRXPinLevel(MBRIDGE_UARTx, LL_USART_RXPIN_LEVEL_INVERTED);
  LL_USART_Enable(UART_UARTx);
#endif

  transmit_enable(false);

  state = STATE_IDLE;
  len = 0;
  cnt = 0;
  tlast_us = 0;
};


void tPin5BridgeBase::transmit_enable(bool enable_flag)
{
  if (enable_flag) {
      uart_rx_enableisr(DISABLE);

#if defined MBRIDGE_TX_OE
      MBRIDGE_TX_OE_ENABLED;
#endif

  } else {
#if defined MBRIDGE_TX_OE
      MBRIDGE_TX_OE_DISABLED;
#endif

      uart_rx_enableisr(ENABLE);
  }
}



#endif // JRPIN5_INTERFACE_H
