//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// MBridge Interface Header
//********************************************************
#ifndef MBRIDGE_INTERFACE_H
#define MBRIDGE_INTERFACE_H
#pragma once

#if (SETUP_TX_USE_MBRIDGE == 1)

#include "mbridge.h"


//-------------------------------------------------------
// some helper definitions


// mBridge: ch0-13    0 .. 1024 .. 2047, 11 bits
//          ch14-15:  0 .. 512 .. 1023, 10 bits
//          ch16-17:  0 .. 1, 1 bit
// rcData:            0 .. 1024 .. 2047, 11 bits
void fill_rcdata_from_mbridge(tRcData* rc, tMBridgeChannelBuffer* mbuf)
{
  rc->ch[0] = mbuf->ch0;
  rc->ch[1] = mbuf->ch1;
  rc->ch[2] = mbuf->ch2;
  rc->ch[3] = mbuf->ch3;
  rc->ch[4] = mbuf->ch4;
  rc->ch[5] = mbuf->ch5;
  rc->ch[6] = mbuf->ch6;
  rc->ch[7] = mbuf->ch7;
  rc->ch[8] = mbuf->ch8;
  rc->ch[9] = mbuf->ch9;
  rc->ch[10] = mbuf->ch10;
  rc->ch[11] = mbuf->ch11;
  rc->ch[12] = mbuf->ch12;
  rc->ch[13] = mbuf->ch13;
  rc->ch[14] = mbuf->ch14 * 2;
  rc->ch[15] = mbuf->ch15 * 2;
  rc->ch[16] = (mbuf->ch16) ? 2047 : 0;
  rc->ch[17] = (mbuf->ch17) ? 2047 : 0;
}


typedef enum {
  MBRIDGE_CMD_TX_LINK_STATS = 0x02,
} MBRIDGE_CMD_TX_ENUM;


PACKED(
typedef struct
{
  uint8_t rssi;
  uint8_t LQ;
  int8_t snr;
  uint8_t rssi2; // in case of 2nd antenna, invalid = UINT8_MAX

  uint8_t receiver_rssi;
  uint8_t receiver_LQ;
  int8_t receiver_snr; // invalid = INT8_MAX
  uint8_t receiver_rssi2; // in case of 2nd antenna, invalid = UINT8_MAX

  uint8_t ant_no : 1; // 0: antenna 1, 1: antenna 2
  uint8_t receiver_ant_no : 1; // 0: antenna 1, 1: antenna 2
  uint8_t spare_bits : 6;

  uint8_t LQ_frames_received;
  uint8_t LQ_received;
  uint8_t LQ_valid_received;
}) tMBridgeLinkStats;


STATIC_ASSERT(sizeof(tMBridgeLinkStats) <= MBRIDGE_COMMANDPACKET_TX_SIZE, "tMBridgeLinkStats len missmatch")


//-------------------------------------------------------
// Interface Implementation

uint16_t micros(void);

void uart_rx_callback(uint8_t c);
void uart_tc_callback(void);

#define UART_RX_CALLBACK_FULL(c)    uart_rx_callback(c)
#define UART_TC_CALLBACK()          uart_tc_callback()

#include "..\modules\stm32ll-lib\src\stdstm32-uart.h"
#include "fifo.h"


uint16_t uart_putc_tobuf(char c)
{
  uint16_t next = (uart_txwritepos + 1) & UART_TXBUFSIZEMASK;
  if (uart_txreadpos != next) { // fifo not full //this is isr safe, works also if readpos has changed in the meanwhile
    uart_txbuf[next] = c;
    uart_txwritepos = next;
    return 1;
  }
  return 0;
}


void uart_tx_start(void)
{
  LL_USART_EnableIT_TXE(UART_UARTx); // initiates transmitting
}


class tMBridge : public tMBridgeBase, tSerialBase
{
  public:

    void Init(void)
    {
        tMBridgeBase::Init();
        tSerialBase::Init();
        transmit_enable(false);
        gpio_init(TX1_XOR, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_VERYFAST);
        gpio_init(RX1_XOR, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_VERYFAST);
        TX1_SET_INVERTED;
        RX1_SET_INVERTED;
        uart_init_isroff();
        sx_tx_fifo.Init();
        sx_rx_fifo.Init();
    }

    // front end to communicate with mbridge

    void putc(char c) { sx_rx_fifo.putc(c); }
    void putbuf(void* buf, uint16_t len) { sx_rx_fifo.putbuf(buf, len); }
    bool available(void) { return sx_tx_fifo.available(); }
    char getc(void) { return sx_tx_fifo.getc(); }

    // backend
    FifoBase sx_tx_fifo;
    FifoBase sx_rx_fifo;

    uint16_t tim_us(void) override { return micros(); }

    void transmit_enable(bool flag) override { uart_rx_enableisr((flag) ? DISABLE : ENABLE); }
    bool mb_rx_available(void) override { return uart_rx_available(); }
    char mb_getc(void) override { return uart_getc(); }
    void mb_putc(char c) override { uart_putc_tobuf(c); }

    void serial_putc(char c) override { sx_tx_fifo.putc(c); }
    bool serial_rx_available(void) override { return sx_rx_fifo.available(); }
    char serial_getc(void) override { return sx_rx_fifo.getc(); }
};

tMBridge bridge;


// we do not add a delay here as with SpinOnce()
// the logic analyzer shows this gives a 30-35 us gap nevertheless, which is perfect

void uart_rx_callback(uint8_t c)
{
  LED_RIGHT_GREEN_ON;
  if (bridge.tx_state >= tMBridgeBase::TXSTATE_TRANSMIT_START) { // recover in case something went wrong
      bridge.tx_state = tMBridgeBase::TXSTATE_IDLE;
  }

  bridge.parse_nextchar(c);

  if (bridge.tx_state == tMBridgeBase::TXSTATE_TRANSMIT_START) {
      bridge.transmit_start();
      uart_tx_start();
  }
  LED_RIGHT_GREEN_OFF;
}


void uart_tc_callback(void)
{
  bridge.transmit_enable(DISABLE);
  bridge.tx_state = tMBridge::TXSTATE_IDLE;
}


#endif // if (SETUP_TX_USE_MBRIDGE == 1)

#endif // MBRIDGE_INTERFACE_H
