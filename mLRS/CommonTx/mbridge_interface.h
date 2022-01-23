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

#if (!defined DEVICE_HAS_MBRIDGE) && (defined USE_MBRIDGE)
  #undef SETUP_TX_SERIAL_DESTINATION
  #define SETUP_TX_SERIAL_DESTINATION  0
  #undef SETUP_TX_CHANNELS_SOURCE
  #define SETUP_TX_CHANNELS_SOURCE  0
  #undef USE_MBRIDGE
  #warning Device does not support mBridge, so mBridge has been disabled !
#endif

#if (defined USE_MBRIDGE) && (defined DEVICE_HAS_MBRIDGE)

#include "mbridge.h"


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


class tMBridgeBase2 : public tMBridgeBase, tSerialBase
{
  public:
    void Init(void);

    void transmit_enable(bool flag) override { uart_rx_enableisr((flag) ? DISABLE : ENABLE); }
    bool mb_rx_available(void) override { return uart_rx_available(); }
    char mb_getc(void) override { return uart_getc(); }
    void mb_putc(char c) override { uart_putc_tobuf(c); }
};


class tMBridge : public tMBridgeBase2
{
  public:
    void Init(void);

    // front end to communicate with mbridge
    void putc(char c) { sx_rx_fifo.putc(c); }
    void putbuf(void* buf, uint16_t len) { sx_rx_fifo.putbuf(buf, len); }
    bool available(void) { return sx_tx_fifo.available(); }
    char getc(void) { return sx_tx_fifo.getc(); }
    void flush(void) { sx_tx_fifo.flush(); }

    // backend
    void serial_putc(char c) override { sx_tx_fifo.putc(c); }
    bool serial_rx_available(void) override { return sx_rx_fifo.available(); }
    char serial_getc(void) override { return sx_rx_fifo.getc(); }

    FifoBase sx_tx_fifo;
    FifoBase sx_rx_fifo;
};

tMBridge bridge;


// we do not add a delay here as with SpinOnce()
// the logic analyzer shows this gives a 30-35 us gap nevertheless, which is perfect

void uart_rx_callback(uint8_t c)
{
  LED_RIGHT_GREEN_ON;
  if (bridge.state >= tMBridgeBase::STATE_TRANSMIT_START) { // recover in case something went wrong
      bridge.state = tMBridgeBase::STATE_IDLE;
  }

  uint16_t tnow_us = micros();
  bridge.parse_nextchar(c, tnow_us);

  if (bridge.state == tMBridgeBase::STATE_TRANSMIT_START) {
      bridge.transmit_start();
      uart_tx_start();
  }
  LED_RIGHT_GREEN_OFF;
}


void uart_tc_callback(void)
{
  bridge.transmit_enable(false);
  bridge.state = tMBridgeBase::STATE_IDLE;
}


void tMBridgeBase2::Init(void)
{
    tMBridgeBase::Init();
    tSerialBase::Init();

    transmit_enable(false);

#if defined MBRIDGE_TX_XOR || defined MBRIDGE_RX_XOR
    gpio_init(MBRIDGE_TX_XOR, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_VERYFAST);
    gpio_init(MBRIDGE_RX_XOR, IO_MODE_OUTPUT_PP_HIGH, IO_SPEED_VERYFAST);
    MBRIDGE_TX_SET_INVERTED;
    MBRIDGE_RX_SET_INVERTED;
#endif

    uart_init_isroff();

#if defined MBRIDGE_RX_TX_INVERT_INTERNAL
    LL_USART_Disable(UART_UARTx);
    LL_USART_SetTXPinLevel(MBRIDGE_UARTx, LL_USART_TXPIN_LEVEL_INVERTED);
    LL_USART_SetRXPinLevel(MBRIDGE_UARTx, LL_USART_RXPIN_LEVEL_INVERTED);
    LL_USART_Enable(UART_UARTx);
#endif
}


void tMBridge::Init(void)
{
    tMBridgeBase2::Init();

    sx_tx_fifo.Init();
    sx_rx_fifo.Init();
}


//-------------------------------------------------------
// convenience helper

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
  int8_t rssi;
  uint8_t LQ;
  int8_t snr; // invalid = INT8_MAX
  int8_t rssi2; // in case of 2nd antenna, invalid = INT8_MAX

  int8_t receiver_rssi;
  uint8_t receiver_LQ;
  int8_t receiver_snr; // invalid = INT8_MAX
  int8_t receiver_rssi2; // in case of 2nd antenna, invalid = INT8_MAX

  uint8_t ant_no : 1; // 0: antenna 1, 1: antenna 2
  uint8_t receiver_ant_no : 1; // 0: antenna 1, 1: antenna 2
  uint8_t spare_bits : 6;

  uint8_t LQ_received_ma;
  uint8_t LQ_received;
  uint8_t LQ_valid_received;
}) tMBridgeLinkStats;


STATIC_ASSERT(sizeof(tMBridgeLinkStats) <= MBRIDGE_TX_COMMAND_PAYLOAD_LEN, "tMBridgeLinkStats len missmatch")


void mbridge_send_LinkStats(void)
{
tMBridgeLinkStats lstats = {0};

  lstats.rssi = txstats.GetRssi();
  lstats.LQ = txstats.GetLQ();
  lstats.snr = stats.last_rx_snr;
  lstats.rssi2 = INT8_MAX;
  lstats.ant_no = 0;
  lstats.receiver_rssi = stats.received_rssi;
  lstats.receiver_LQ = stats.received_LQ;
  lstats.receiver_snr = INT8_MAX;
  lstats.receiver_rssi2 = INT8_MAX;
  lstats.receiver_ant_no = 0;
  lstats.LQ_received_ma = stats.GetTransmitBandwidthUsage();
  lstats.LQ_received = stats.LQ_frames_received;
  lstats.LQ_valid_received = stats.GetReceiveBandwidthUsage();

  bridge.SendCommand(MBRIDGE_CMD_TX_LINK_STATS, (uint8_t*)&lstats, sizeof(tMBridgeLinkStats));
}



#endif // if (defined USE_MBRIDGE) && (defined DEVICE_HAS_MBRIDGE)

#endif // MBRIDGE_INTERFACE_H
