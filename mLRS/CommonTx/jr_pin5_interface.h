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
    void pin5_transmit_enable(bool enable_flag);
    //bool pin5_rx_available(void) { return uart_rx_available(); }
    //char pin5_getc(void) { return uart_getc(); }
    void pin5_tx_start(void) { uart_tx_start(); }
    void pin5_putc(char c) { uart_putc_tobuf(c); }

    // for in-isr processing
    virtual void parse_nextchar(uint8_t c, uint16_t tnow_us);
    virtual bool transmit_start(void); // returns true if transmission should be started

    typedef enum {
      STATE_IDLE = 0,

      STATE_RECEIVE_MBRIDGE_STX2,
      STATE_RECEIVE_MBRIDGE_LEN,
      STATE_RECEIVE_MBRIDGE_SERIALPACKET,
      STATE_RECEIVE_MBRIDGE_CHANNELPACKET,
      STATE_RECEIVE_MBRIDGE_COMMANDPACKET,

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
// TX & RX XOR method
#if defined JRPIN5_TX_XOR && defined JRPIN5_RX_XOR
  gpio_init(JRPIN5_TX_XOR, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_VERYFAST);
  gpio_init(JRPIN5_RX_XOR, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_VERYFAST);
  JRPIN5_TX_SET_INVERTED;
  JRPIN5_RX_SET_INVERTED;
#endif

// TX & RX inverter with TX buffer method
#if defined JRPIN5_TX_OE
  gpio_init(JRPIN5_TX_OE, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_VERYFAST);
  JRPIN5_TX_OE_DISABLED;
#endif

  uart_init_isroff();

// internal peripheral inverter method
#if defined JRPIN5_RX_TX_INVERT_INTERNAL
  LL_USART_Disable(JRPIN5_UARTx);
  LL_USART_SetTXPinLevel(JRPIN5_UARTx, LL_USART_TXPIN_LEVEL_INVERTED);
  LL_USART_SetRXPinLevel(JRPIN5_UARTx, LL_USART_RXPIN_LEVEL_INVERTED);
  LL_USART_Enable(JRPIN5_UARTx);
#endif

  pin5_transmit_enable(false);

  state = STATE_IDLE;
  len = 0;
  cnt = 0;
  tlast_us = 0;
};


void tPin5BridgeBase::pin5_transmit_enable(bool enable_flag)
{
  if (enable_flag) {
      uart_rx_enableisr(DISABLE);

#if defined JRPIN5_TX_OE
      JRPIN5_TX_OE_ENABLED;
#endif

  } else {
#if defined JRPIN5_TX_OE
      JRPIN5_TX_OE_DISABLED;
#endif

      uart_rx_enableisr(ENABLE);
  }
}



#endif // JRPIN5_INTERFACE_H
