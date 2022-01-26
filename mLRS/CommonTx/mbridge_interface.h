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

#include "mbridge_protocol.h"


//-------------------------------------------------------
// Interface Implementation

uint16_t micros(void);

void uart_rx_callback(uint8_t c);
void uart_tc_callback(void);

#define UART_RX_CALLBACK_FULL(c)    uart_rx_callback(c)
#define UART_TC_CALLBACK()          uart_tc_callback()

#include "..\modules\stm32ll-lib\src\stdstm32-uart.h"
#include "fifo.h"


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
    void transmit_enable(bool flag) { uart_rx_enableisr((flag) ? DISABLE : ENABLE); }
    bool mb_rx_available(void) { return uart_rx_available(); }
    char mb_getc(void) { return uart_getc(); }
    void mb_putc(char c) { uart_putc_tobuf(c); }

    virtual void parse_nextchar(uint8_t c, uint16_t tnow_us);
    virtual bool transmit_start(void); // returns true if transmission should be started

    typedef enum {
      STATE_IDLE = 0,

      STATE_MBRIDGE_RECEIVE_STX2,
      STATE_MBRIDGE_RECEIVE_LEN,
      STATE_MBRIDGE_RECEIVE_SERIALPACKET,
      STATE_MBRIDGE_RECEIVE_CHANNELPACKET,
      STATE_MBRIDGE_RECEIVE_COMMANDPACKET,

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

  state = STATE_IDLE;
  len = 0;
  cnt = 0;
  tlast_us = 0;
};


class tMBridge : public tPin5BridgeBase, public tSerialBase
{
  public:
    void Init(void);
    void GetCommand(uint8_t* cmd, uint8_t* payload);
    void SendCommand(uint8_t cmd, uint8_t* payload, uint8_t payload_len);

    // for in-isr processing
    void parse_nextchar(uint8_t c, uint16_t tnow_us) override;
    bool transmit_start(void) override; // returns true if transmission should be started
    uint8_t send_serial(void);
    void send_command(void);

    uint8_t type;

    volatile bool channels_received;
    tMBridgeChannelBuffer channels;

    volatile bool cmd_received;
    uint8_t cmd_rx_frame[MBRIDGE_RX_COMMAND_FRAME_LEN];

    volatile uint8_t cmd_tx_available;
    uint8_t cmd_tx_frame[MBRIDGE_TX_COMMAND_FRAME_LEN];

    // front end to communicate with mbridge
    void putc(char c) { sx_rx_fifo.putc(c); }
    void putbuf(void* buf, uint16_t len) { sx_rx_fifo.putbuf(buf, len); }
    bool available(void) { return sx_tx_fifo.available(); }
    char getc(void) { return sx_tx_fifo.getc(); }
    void flush(void) { sx_tx_fifo.flush(); }

    // backend
    // mimics a serial interface to the main code, usually two fifo
    void serial_putc(char c) { sx_tx_fifo.putc(c); }
    bool serial_rx_available(void) { return sx_rx_fifo.available(); }
    char serial_getc(void) { return sx_rx_fifo.getc(); }

    FifoBase sx_tx_fifo;
    FifoBase sx_rx_fifo;
};

tMBridge bridge;


// we do not add a delay here as with SpinOnce()
// the logic analyzer shows this gives a 30-35 us gap nevertheless, which is perfect

void uart_rx_callback(uint8_t c)
{
  LED_RIGHT_GREEN_ON;

  if (bridge.state >= tPin5BridgeBase::STATE_TRANSMIT_START) { // recover in case something went wrong
      bridge.state = tPin5BridgeBase::STATE_IDLE;
  }

  uint16_t tnow_us = micros();
  bridge.parse_nextchar(c, tnow_us);

  if (bridge.transmit_start()) {
      uart_tx_start();
  }

  LED_RIGHT_GREEN_OFF;
}


void uart_tc_callback(void)
{
  bridge.transmit_enable(false);
  bridge.state = tPin5BridgeBase::STATE_IDLE;
}


bool tMBridge::transmit_start(void)
{
uint8_t tx_available = 0;

  if (state < STATE_TRANSMIT_START) return false; // we are in receiving

  if (state != STATE_TRANSMIT_START) {
      state = STATE_IDLE;
      return false;
  }

  if (cmd_tx_available) {
      tx_available = cmd_tx_available;
      cmd_tx_available = 0;
      send_command();
  } else {
      tx_available = send_serial();
  }

  if (!tx_available) {
      state = STATE_IDLE;
      return false;
  }

  transmit_enable(false);

  state = STATE_TRANSMITING;
  return true;
}


uint8_t tMBridge::send_serial(void)
{
  uint8_t max_count = (type >= MBRIDGE_TYPE_CHANNELPACKET) ? MBRIDGE_TX_SERIAL_PAYLOAD_LEN_LIM : MBRIDGE_TX_SERIAL_PAYLOAD_LEN_MAX;
  uint8_t count = 0;
  uint8_t payload[MBRIDGE_TX_SERIAL_PAYLOAD_LEN_MAX];
  for (uint8_t i = 0; i < max_count; i++) {
      if (!serial_rx_available()) break;
      payload[count++] = serial_getc();
  }
  if (count > 0) {
      mb_putc(MBRIDGE_SERIALPACKET_STX); // send type byte
      for (uint8_t i = 0; i < count; i++) {
          uint8_t c = payload[i];
          mb_putc(c);
      }
  }
  return count;
}


void tMBridge::send_command(void)
{
  for (uint8_t i = 0; i < MBRIDGE_TX_COMMAND_FRAME_LEN; i++) {
      uint8_t c = cmd_tx_frame[i];
      mb_putc(c);
  }
}


void tMBridge::Init(void)
{
  tSerialBase::Init();
  tPin5BridgeBase::Init();

  type = MBRIDGE_TYPE_NONE;
  channels_received = false;
  cmd_received = false;
  cmd_tx_available = 0;

  sx_tx_fifo.Init();
  sx_rx_fifo.Init();
}


//-------------------------------------------------------
// MBridge

#define MBRIDGE_TMO_US  250


void tMBridge::parse_nextchar(uint8_t c, uint16_t tnow_us)
{
  if (state != STATE_IDLE) {
      uint16_t dt = tnow_us - tlast_us;
      if (dt > MBRIDGE_TMO_US) state = STATE_IDLE; // timeout error
  }

  tlast_us = tnow_us;

  switch (state) {
  case STATE_IDLE:
      if (c == MBRIDGE_STX1) state = STATE_MBRIDGE_RECEIVE_STX2;
      break;

  case STATE_MBRIDGE_RECEIVE_STX2:
      if (c == MBRIDGE_STX2) state = STATE_MBRIDGE_RECEIVE_LEN; else state = STATE_IDLE; // error
      break;
  case STATE_MBRIDGE_RECEIVE_LEN:
      cnt = 0;
      if (c == MBRIDGE_CHANNELPACKET_STX) {
          len = MBRIDGE_CHANNELPACKET_SIZE;
          type = MBRIDGE_TYPE_CHANNELPACKET;
          state = STATE_MBRIDGE_RECEIVE_CHANNELPACKET;
      } else
      if (c >= MBRIDGE_COMMANDPACKET_STX) {
          cmd_rx_frame[cnt++] = (c & ~MBRIDGE_COMMANDPACKET_MASK);
          len = MBRIDGE_RX_COMMAND_PAYLOAD_LEN;
          type = MBRIDGE_TYPE_COMMANDPACKET;
          state = STATE_MBRIDGE_RECEIVE_COMMANDPACKET;
      } else
      if (c > MBRIDGE_RX_SERIAL_PAYLOAD_LEN_MAX) {
          state = STATE_IDLE; // error
      } else
      if (c > 0) {
          len = c;
          type = MBRIDGE_TYPE_SERIALPACKET;
          state = STATE_MBRIDGE_RECEIVE_SERIALPACKET;
      } else {
          type = MBRIDGE_TYPE_NONE;
          state = STATE_TRANSMIT_START; // tx_len = 0, no payload
      }
      break;
  case STATE_MBRIDGE_RECEIVE_SERIALPACKET:
      serial_putc(c);
      cnt++;
      if (cnt >= len) state = STATE_TRANSMIT_START;
      break;
  case STATE_MBRIDGE_RECEIVE_CHANNELPACKET:
      channels.c[cnt++] = c;
      if (cnt >= len) {
          channels_received = true;
          state = STATE_TRANSMIT_START;
      }
      break;
  case STATE_MBRIDGE_RECEIVE_COMMANDPACKET:
      cmd_rx_frame[cnt++] = c;
      if (cnt >= len + 1) {
          cmd_received = true;
          state = STATE_TRANSMIT_START;
      }
      break;
  }
}


void tMBridge::GetCommand(uint8_t* cmd, uint8_t* payload)
{
  *cmd = cmd_rx_frame[0];
  memcpy(payload, &(cmd_rx_frame[1]), MBRIDGE_RX_COMMAND_PAYLOAD_LEN);
}


void tMBridge::SendCommand(uint8_t cmd, uint8_t* payload, uint8_t payload_len)
{
  memset(cmd_tx_frame, 0, MBRIDGE_TX_COMMAND_FRAME_LEN);
  if (payload_len > MBRIDGE_TX_COMMAND_PAYLOAD_LEN) payload_len = MBRIDGE_TX_COMMAND_PAYLOAD_LEN;

  cmd_tx_frame[0] = MBRIDGE_COMMANDPACKET_STX + (cmd &~ MBRIDGE_COMMANDPACKET_MASK);
  memcpy(&(cmd_tx_frame[1]), payload, payload_len);

  cmd_tx_available = MBRIDGE_TX_COMMAND_FRAME_LEN;
}


//-------------------------------------------------------
// convenience helper

STATIC_ASSERT(sizeof(tMBridgeLinkStats) <= MBRIDGE_TX_COMMAND_PAYLOAD_LEN, "tMBridgeLinkStats len missmatch")


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

  bridge.SendCommand(MBRIDGE_TX_CMD_LINK_STATS, (uint8_t*)&lstats, sizeof(tMBridgeLinkStats));
}



#endif // if (defined USE_MBRIDGE) && (defined DEVICE_HAS_MBRIDGE)

#endif // MBRIDGE_INTERFACE_H
