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
    uint8_t cmd_r2m_frame[MBRIDGE_R2M_COMMAND_FRAME_LEN_MAX];

    volatile uint8_t cmd_m2r_available;
    uint8_t cmd_m2r_frame[MBRIDGE_M2R_COMMAND_FRAME_LEN_MAX];

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
uint8_t available = 0;

  if (state < STATE_TRANSMIT_START) return false; // we are in receiving

  if (state != STATE_TRANSMIT_START) {
      state = STATE_IDLE;
      return false;
  }

  if (cmd_m2r_available) {
      send_command(); // uses cmd_m2r_available
      available = cmd_m2r_available;
      cmd_m2r_available = 0;
  } else {
      available = send_serial();
  }

  if (!available) {
      state = STATE_IDLE;
      return false;
  }

  transmit_enable(false);

  state = STATE_TRANSMITING;
  return true;
}


uint8_t tMBridge::send_serial(void)
{
  uint8_t count = 0;
  uint8_t payload[MBRIDGE_M2R_SERIAL_PAYLOAD_LEN_MAX];
  for (uint8_t i = 0; i < MBRIDGE_M2R_SERIAL_PAYLOAD_LEN_MAX; i++) {
      if (!serial_rx_available()) break;
      payload[count++] = serial_getc();
  }
  if (count > 0) {
      mb_putc(0x00); // we can send anything we want which is not a command, send 0xoo so it is easy to recognize
      for (uint8_t i = 0; i < count; i++) {
          uint8_t c = payload[i];
          mb_putc(c);
      }
  }
  return count;
}


void tMBridge::send_command(void)
{
  for (uint8_t i = 0; i < cmd_m2r_available; i++) {
      uint8_t c = cmd_m2r_frame[i];
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
  cmd_m2r_available = 0;

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
          uint8_t cmd = c & (~MBRIDGE_COMMANDPACKET_MASK);
          cmd_r2m_frame[cnt++] = cmd;
          len = mbridge_cmd_payload_len(cmd);
          type = MBRIDGE_TYPE_COMMANDPACKET;
          state = STATE_MBRIDGE_RECEIVE_COMMANDPACKET;
      } else
      if (c > MBRIDGE_R2M_SERIAL_PAYLOAD_LEN_MAX) {
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
      cmd_r2m_frame[cnt++] = c;
      if (cnt >= len + 1) {
          cmd_received = true;
          state = STATE_TRANSMIT_START;
      }
      break;
  }
}


void tMBridge::GetCommand(uint8_t* cmd, uint8_t* payload)
{
  if ((cmd_r2m_frame[0] & MBRIDGE_COMMANDPACKET_MASK) != MBRIDGE_COMMANDPACKET_STX) return; // not a command, should not happen, but play it safe

  *cmd = cmd_r2m_frame[0] & (~MBRIDGE_COMMANDPACKET_MASK) ;

  uint8_t payload_len = mbridge_cmd_payload_len(*cmd);
  memcpy(payload, &(cmd_r2m_frame[1]), payload_len);
}


void tMBridge::SendCommand(uint8_t cmd, uint8_t* payload, uint8_t payload_len)
{
  memset(cmd_m2r_frame, 0, MBRIDGE_M2R_COMMAND_FRAME_LEN_MAX);

  if (payload_len != mbridge_cmd_payload_len(cmd)) return; // should never happen but play it safe

  cmd_m2r_frame[0] = MBRIDGE_COMMANDPACKET_STX + (cmd & (~MBRIDGE_COMMANDPACKET_MASK));
  memcpy(&(cmd_m2r_frame[1]), payload, payload_len);

  cmd_m2r_available = payload_len + 1;
}


//-------------------------------------------------------
// convenience helper

STATIC_ASSERT(sizeof(tMBridgeLinkStats) == MBRIDGE_CMD_TX_LINK_STATS_LEN, "tMBridgeLinkStats len missmatch")


// mBridge: ch0-15    0 .. 1024 .. 2047, 11 bits
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
  rc->ch[14] = mbuf->ch14;
  rc->ch[15] = mbuf->ch15;
  rc->ch[16] = (mbuf->ch16) ? 2047 : 0;
  rc->ch[17] = (mbuf->ch17) ? 2047 : 0;
}


void mbridge_send_LinkStats(void)
{
tMBridgeLinkStats lstats = {0};

  lstats.LQ = txstats.GetLQ(); // it's the same as GetLQ_serial_data() // = LQ_valid_received; // number of valid packets received on transmitter side
  lstats.rssi_instantaneous = stats.last_rx_rssi;
  lstats.snr_instantaneous = (stats.last_rx_antenna) ? stats.last_rx_snr : stats.last_rx_snr2;
  lstats.rssi2_instantaneous = stats.last_rx_rssi2;
  lstats.ant_no = stats.last_rx_antenna;

  lstats.rssi_filtered = INT8_MAX;
  lstats.snr_filtered = INT8_MAX;
  lstats.rssi2_filtered = INT8_MAX;

  // receiver side of things

  lstats.receiver_LQ = stats.received_LQ; // = receiver_LQ_crc1_received; // number of rc data packets received on receiver side
  lstats.receiver_LQ_serial = stats.received_LQ_serial_data; // = receiver_LQ_valid_received; // number of completely valid packets received on receiver side
  lstats.receiver_rssi_instantaneous = stats.received_rssi;
  lstats.receiver_snr_instantaneous = INT8_MAX;
  lstats.receiver_rssi2_instantaneous = INT8_MAX;
  lstats.receiver_ant_no = stats.received_antenna;

  lstats.receiver_rssi_filtered = INT8_MAX;
  lstats.receiver_snr_filtered = INT8_MAX;
  lstats.receiver_rssi2_filtered = INT8_MAX;

  // further stats acquired on transmitter side

  lstats.LQ_fresh_serial_packets_transmitted = stats.fresh_serial_data_transmitted.GetLQ();
  lstats.bytes_per_sec_transmitted = stats.GetTransmitBandwidthUsage();

  lstats.LQ_valid_received = stats.valid_frames_received.GetLQ(); // number of completely valid packets received per sec
  lstats.LQ_fresh_serial_packets_received = stats.fresh_serial_data_received.GetLQ();
  lstats.bytes_per_sec_received = stats.GetReceiveBandwidthUsage();

  lstats.LQ_received = stats.frames_received.GetLQ(); // number of packets received per sec, not practically relevant

  bridge.SendCommand(MBRIDGE_CMD_TX_LINK_STATS, (uint8_t*)&lstats, sizeof(tMBridgeLinkStats));
}



#endif // if (defined USE_MBRIDGE) && (defined DEVICE_HAS_MBRIDGE)

#endif // MBRIDGE_INTERFACE_H
